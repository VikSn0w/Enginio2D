#include "sim/Engine.h"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

constexpr double kPi     = 3.14159265358979323846;
constexpr double kDegRad = kPi / 180.0;
// Ambient to the fourth, for the radiation term in the exhaust wall balance.
constexpr double kAmbient4 = Thermo::tAmb * Thermo::tAmb * Thermo::tAmb * Thermo::tAmb;

double wrap720(double a) {
    a = std::fmod(a, 720.0);
    return a < 0.0 ? a + 720.0 : a;
}

// True if the cycle angle lies in the (possibly wrapping) interval.
bool inWindow(double a, double open, double close) {
    if (open <= close) return a >= open && a <= close;
    return a >= open || a <= close;
}

// -------------------------------------------------------------------------
// Compressible-flow function, tabulated.
//
//   mdot = A * p_up / sqrt(R T_up) * Psi(p_down / p_up)
//   Psi  = sqrt( 2g/(g-1) * (pr^(2/g) - pr^((g+1)/g)) ), frozen at the
//          critical ratio once the restriction chokes.
// -------------------------------------------------------------------------
constexpr int kPsiBins = 1024;

struct PsiTable {
    double v[kPsiBins + 1];
    PsiTable() {
        constexpr double g = Thermo::gammaFlow;
        const double critical = std::pow(2.0 / (g + 1.0), g / (g - 1.0));
        for (int i = 0; i <= kPsiBins; ++i) {
            double pr = static_cast<double>(i) / kPsiBins;
            pr = std::clamp(pr, critical, 1.0);   // choked below the critical ratio
            const double term = (2.0 * g / (g - 1.0)) *
                                (std::pow(pr, 2.0 / g) - std::pow(pr, (g + 1.0) / g));
            v[i] = term > 0.0 ? std::sqrt(term) : 0.0;
        }
    }
    double operator()(double pr) const {
        if (pr <= 0.0) return v[0];
        if (pr >= 1.0) return 0.0;
        const double x = pr * kPsiBins;
        const int    i = static_cast<int>(x);
        const double f = x - i;
        return v[i] * (1.0 - f) + v[i + 1] * f;
    }
};

const PsiTable g_psi;

// -------------------------------------------------------------------------
// Discharge coefficient against dimensionless lift, referenced to
// min(curtain area, throat area). Follows the shape of measured flow-bench
// data: best just off the seat, tailing off as the jet detaches at high lift.
// -------------------------------------------------------------------------
double dischargeCoefficient(double liftOverDiameter) {
    static constexpr double kTable[] = {0.70, 0.72, 0.70, 0.66, 0.62, 0.58, 0.55, 0.53};
    constexpr double kStep = 0.05;
    const double x = std::clamp(liftOverDiameter / kStep, 0.0, 7.0);
    const int    i = static_cast<int>(x);
    if (i >= 7) return kTable[7];
    const double f = x - i;
    return kTable[i] * (1.0 - f) + kTable[i + 1] * f;
}

// One control volume, referenced so flows can move mass, energy and
// composition between any two of them.
struct Node {
    double* mass;
    double* energy;
    double* burned;
    double  p, T, y;
};

} // namespace

double Thermo::temperature(double e, double y) {
    // Solve  b/2 x^2 + a x - e = 0  with x = T - T0.
    const double a = cvA(y);
    const double b = cvB(y);
    const double disc = a * a + 2.0 * b * e;
    const double x = disc > 0.0 ? (-a + std::sqrt(disc)) / b : -a / b;
    return std::clamp(T0 + x, 150.0, 3300.0);
}

// ---------------------------------------------------------------------------
// Cam profile. A raised cosine has too pointed a nose to pass for a real cam;
// raising it to a power below one flattens the top and steepens the flanks,
// which is roughly what a polydyne profile looks like. How far below one the
// exponent can go is a property of the valvetrain, not of the cam grinder's
// imagination: a flat tappet runs out of contact patch long before a roller
// does, and only a desmodromic or pneumatic head can follow a near-square
// profile.
// ---------------------------------------------------------------------------
double valveLiftAt(const ValveTiming& v, double phaseDeg) {
    double span = v.closeDeg - v.openDeg;
    if (span < 0.0) span += 720.0;
    if (span <= 1.0) return 0.0;
    const double phase = wrap720(phaseDeg);
    if (!inWindow(phase, wrap720(v.openDeg), wrap720(v.closeDeg))) return 0.0;
    double x = phase - v.openDeg;
    if (x < 0.0) x += 720.0;
    if (x > span) return 0.0;
    const double hump = 0.5 * (1.0 - std::cos(2.0 * kPi * x / span));
    return v.maxLift * std::pow(hump, v.profileExp);
}

Engine::Engine(EngineParams params) : m_p(std::move(params)) {
    rebuild(false);
}

void Engine::reconfigure(const EngineParams& params) {
    m_p = params;
    // The gas states cannot be carried across a change of geometry - the
    // volumes they were computed for no longer exist - so they are re-seeded at
    // ambient and the engine breathes its way back to a running state within a
    // cycle or two. Crank speed, oil temperature and the driver's inputs do
    // survive, which is what stops an edit from feeling like a restart.
    rebuild(true);
}

void Engine::rebuild(bool preserveMotion) {
    m_p.cylinders = std::clamp(m_p.cylinders, 1, static_cast<int>(kMaxCylinders));
    m_p.compressionRatio = std::max(4.0, m_p.compressionRatio);

    m_pistonArea   = kPi * 0.25 * m_p.bore * m_p.bore;
    m_displacement = m_pistonArea * m_p.stroke;
    m_clearance    = m_displacement / (m_p.compressionRatio - 1.0);

    buildLiftTable(m_intakeLift, m_p.intake);
    buildLiftTable(m_exhaustLift, m_p.exhaust);

    const double runnerVol   = m_p.intakeRunner.length * m_p.intakeRunner.area;
    const double exRunnerVol = m_p.exhaustRunner.length * m_p.exhaustRunner.area;

    // Wetted area of the exhaust, for the wall heat transfer. The primary is a
    // pipe of the runner's own cross-section; the collector is taken as a can
    // three times as long as it is wide, which is what most of them are.
    const double exDia = std::sqrt(4.0 * m_p.exhaustRunner.area / kPi);
    m_exRunnerWallArea = kPi * exDia * m_p.exhaustRunner.length;
    const double colDia = std::cbrt(4.0 * m_p.collectorVolume / (3.0 * kPi));
    m_collectorWallArea = 3.0 * kPi * colDia * colDia;

    if (!preserveMotion) {
        m_crankAngle = 0.0;
        m_omega      = 0.0;
        m_throttle   = 0.0;
        m_oilTemp    = m_p.oilStartTemp;
    }
    m_spool = 0.0;
    m_boostPressure = Thermo::pAmb;
    m_boostTemp     = Thermo::tAmb;
    m_knockLevel = m_knockRetard = 0.0;
    m_knockFuelTerm = 17.68e-3 * std::pow(std::max(m_p.fuel.octane, 40.0) * 0.01, 3.402);

    m_cyl.assign(static_cast<std::size_t>(m_p.cylinders), Cylinder{});
    for (std::size_t i = 0; i < m_cyl.size(); ++i) {
        const double offset = i < m_p.phaseOffsets.size() ? m_p.phaseOffsets[i] : 0.0;
        Cylinder& c   = m_cyl[i];
        c.phase       = wrap720(m_crankAngle + offset);
        c.volume      = cylinderVolume(c.phase);
        c.pressure    = Thermo::pAmb;
        c.temperature = Thermo::tAmb;
        c.mass        = c.pressure * c.volume / (Thermo::R * c.temperature);
        c.energy      = c.mass * Thermo::energy(c.temperature, 0.0);

        c.runnerMass   = Thermo::pAmb * runnerVol / (Thermo::R * Thermo::tAmb);
        c.runnerEnergy = c.runnerMass * Thermo::energy(Thermo::tAmb, 0.0);
        c.exRunnerMass   = Thermo::pAmb * exRunnerVol / (Thermo::R * Thermo::tAmb);
        c.exRunnerEnergy = c.exRunnerMass * Thermo::energy(Thermo::tAmb, 1.0);
        c.exRunnerBurned = c.exRunnerMass;

        c.intakeWave.configure(m_p.intakeRunner.length, m_p.intakeRunner.area,
                               m_p.intakeReflect, m_p.waveDamping, m_p.waveStrength);
        c.exhaustWave.configure(m_p.exhaustRunner.length, m_p.exhaustRunner.area,
                                m_p.exhaustReflect, m_p.waveDamping, m_p.waveStrength);
    }

    m_plenumMass   = Thermo::pAmb * m_p.plenumVolume / (Thermo::R * Thermo::tAmb);
    m_plenumEnergy = m_plenumMass * Thermo::energy(Thermo::tAmb, 0.0);
    m_plenumBurned = 0.0;

    // Each bank gets its own collector, and they share the volume the design
    // asks for rather than each getting all of it.
    m_bankCount = std::clamp(static_cast<std::size_t>(std::max(1, m_p.exhaustBanks)),
                             std::size_t{1}, kMaxBanks);
    m_collectorVolEach = std::max(1.0e-5, m_p.collectorVolume / static_cast<double>(m_bankCount));
    for (std::size_t b = 0; b < kMaxBanks; ++b) {
        m_collectorMass[b]     = Thermo::pAmb * m_collectorVolEach / (Thermo::R * Thermo::tAmb);
        m_collectorEnergy[b]   = m_collectorMass[b] * Thermo::energy(Thermo::tAmb, 1.0);
        m_collectorBurned[b]   = m_collectorMass[b];
        m_collectorTemp[b]     = Thermo::tAmb;
        m_collectorPressure[b] = Thermo::pAmb;
        m_collectorWall[b]     = Thermo::tAmb;
    }

    if (!preserveMotion)
        for (auto& t : m_trace) t.store(0.0f, std::memory_order_relaxed);
}

void Engine::setThrottle(double t) { m_throttle = std::clamp(t, 0.0, 1.0); }

void Engine::setSpeedHold(bool on, double rpmHold) {
    m_speedHold = on;
    m_holdOmega = rpmHold * 2.0 * kPi / 60.0;
    if (on) m_omega = m_holdOmega;
}

void Engine::buildLiftTable(std::vector<double>& table, const ValveTiming& v) const {
    table.assign(kLiftBins, 0.0);
    for (std::size_t i = 0; i < kLiftBins; ++i)
        table[i] = valveLiftAt(v, static_cast<double>(i) * 720.0 / kLiftBins);
}

double Engine::lookupLift(const std::vector<double>& table, double phaseDeg) {
    const double x = wrap720(phaseDeg) * (kLiftBins / 720.0);
    const std::size_t i = static_cast<std::size_t>(x) % kLiftBins;
    const std::size_t j = (i + 1) % kLiftBins;
    const double f = x - std::floor(x);
    return table[i] * (1.0 - f) + table[j] * f;
}

double Engine::volumeFromTrig(double sinTh, double cosTh) const {
    const double a = 0.5 * m_p.stroke;
    const double l = m_p.rodLength;
    const double s = a * sinTh;
    const double x = a * cosTh + std::sqrt(l * l - s * s);
    return m_clearance + m_pistonArea * (l + a - x);
}

double Engine::cylinderVolume(double phaseDeg) const {
    const double th = phaseDeg * kDegRad;
    return volumeFromTrig(std::sin(th), std::cos(th));
}

double Engine::dVolumeFromTrig(double st, double ct) const {
    const double a    = 0.5 * m_p.stroke;
    const double l    = m_p.rodLength;
    const double root = std::sqrt(l * l - a * a * st * st);
    return m_pistonArea * a * (st + a * st * ct / root);
}

double Engine::dVolumeDTheta(double phaseDeg) const {
    const double th = phaseDeg * kDegRad;
    return dVolumeFromTrig(std::sin(th), std::cos(th));
}

double Engine::portArea(const ValveTiming& v, double lift) const {
    if (lift <= 0.0) return 0.0;
    const double curtain = kPi * v.diameter * lift;              // valve curtain
    const double seat    = kPi * 0.25 * v.diameter * v.diameter; // port throat
    const double cd      = v.cdScale * dischargeCoefficient(lift / v.diameter);
    return v.count * cd * std::min(curtain, seat);
}

double Engine::orificeFlow(double pUp, double tUp, double pDown, double area) {
    if (area <= 0.0 || pUp <= pDown || tUp <= 0.0) return 0.0;
    return area * pUp * g_psi(pDown / pUp) / std::sqrt(Thermo::R * tUp);
}

// ---------------------------------------------------------------------------
// Compressor. Everything upstream of the throttle plate lives here: on a
// naturally aspirated engine that is just the atmosphere, on a blown one it is
// the compressor discharge after the intercooler.
// ---------------------------------------------------------------------------
void Engine::updateCharger(double dt, double throttleFlow) {
    m_throttleFlow += (throttleFlow - m_throttleFlow) * std::min(1.0, 30.0 * dt);

    if (m_p.charger == 0 || m_p.boostTarget <= 0.0) {
        m_boostPressure = Thermo::pAmb;
        m_boostTemp     = Thermo::tAmb;
        m_spool         = 0.0;
        m_blowerTorque  = 0.0;
        return;
    }

    const double speed = rpm();
    double target = 0.0;
    switch (m_p.charger) {
        case 1: {
            // Turbo: the turbine is driven by exhaust flow, so boost follows
            // how much air the engine is already moving - which is exactly why
            // it cannot make boost before it is making power. The first-order
            // lag is the rotor's inertia.
            const double refFlow = 1.18 * m_displacement * m_p.cylinders *
                                   (std::max(m_p.spoolRpm, 800.0) / 120.0);
            const double drive = std::clamp(m_throttleFlow / std::max(refFlow, 1e-6), 0.0, 1.0);
            // A steady-state dyno point is settled by definition, so the rotor
            // inertia is not part of the measurement.
            const double tau = m_speedHold ? 0.02 : std::max(m_p.turboLag, 0.05);
            m_spool += (drive - m_spool) * std::min(1.0, dt / tau);
            target = m_spool;
            break;
        }
        case 2:
            // Roots blower: a positive-displacement pump geared to the crank.
            // It makes its boost from just off idle, which is the whole point
            // of one, and it takes its drive power the whole time.
            m_spool = std::clamp((speed - 500.0) / std::max(600.0, m_p.spoolRpm * 0.4), 0.0, 1.0);
            target = m_spool;
            break;
        default: {
            // Centrifugal: pressure rises with the square of impeller speed, so
            // it is worth nothing low down and everything at the top.
            const double f = speed / std::max(m_p.spoolRpm, 800.0);
            m_spool = std::clamp(f * f, 0.0, 1.0);
            target = m_spool;
            break;
        }
    }

    const double ratio = 1.0 + (m_p.boostTarget * 1.0e5 / Thermo::pAmb) * target;
    m_boostPressure = Thermo::pAmb * ratio;

    // Isentropic compression plus the compressor's own inefficiency, then
    // whatever the intercooler can give back. Charge temperature is what limits
    // boost on a real engine long before the compressor runs out.
    const double tIdeal = Thermo::tAmb * std::pow(ratio, 0.2857);
    const double t2 = Thermo::tAmb + (tIdeal - Thermo::tAmb) /
                                     std::clamp(m_p.compressorEff, 0.35, 0.95);
    m_boostTemp = t2 - std::clamp(m_p.interCoolerEff, 0.0, 0.95) * (t2 - Thermo::tAmb);

    // A belt blower is paid for out of crank torque; a turbo is paid for in
    // backpressure, which the exhaust outlet area already carries.
    if (m_p.charger != 1) {
        const double power = m_throttleFlow * 1005.0 * std::max(0.0, t2 - Thermo::tAmb) /
                             std::clamp(m_p.blowerDriveEff, 0.4, 0.98);
        m_blowerTorque = power / std::max(m_omega, 20.0);
    } else {
        m_blowerTorque = 0.0;
    }
}

void Engine::advance(double dt) {
    // Two constraints on the sub-step, and the looser one wins.
    //
    // The explicit valve and runner integration needs a short enough step in
    // *time* - around 8 us, which is comfortably inside what one unsubdivided
    // 44.1 kHz sample already is at idle. It also needs enough resolution in
    // crank angle to see a valve event at all, which is what the 0.25 degree
    // cap is for. Insisting on the angle cap at every speed makes a twelve
    // cylinder engine at 10000 rpm cost three times what it needs to, and the
    // physics core has to keep up with the audio thread it runs on.
    constexpr double kMinStep = 8.0e-6;   // s
    constexpr double kMaxDeg  = 0.25;
    const double degPerStep = std::abs(m_omega) / kDegRad * dt;
    int steps = static_cast<int>(std::ceil(degPerStep / kMaxDeg));
    steps = std::min(steps, static_cast<int>(std::ceil(dt / kMinStep)));
    steps = std::clamp(steps, 1, 48);
    const double h = dt / steps;
    for (int i = 0; i < steps; ++i) step(h);
}

void Engine::step(double dt) {
    const double omegaDeg = m_omega / kDegRad;   // deg/s
    m_crankAngle = wrap720(m_crankAngle + omegaDeg * dt);

    const double runnerVol   = m_p.intakeRunner.length * m_p.intakeRunner.area;
    const double exRunnerVol = m_p.exhaustRunner.length * m_p.exhaustRunner.area;
    const bool   diesel      = m_p.fuel.compressionIgnition;

    // Move mass, energy and composition from the higher-pressure node to the
    // lower one through a restriction. Returns the flow from a to b (kg/s).
    auto exchange = [&](Node a, Node b, double area) -> double {
        if (area <= 0.0) return 0.0;
        const bool aUp = a.p >= b.p;
        Node& up = aUp ? a : b;
        Node& dn = aUp ? b : a;
        const double f = orificeFlow(up.p, up.T, dn.p, area);
        if (f <= 0.0) return 0.0;
        const double dm = std::min(f * dt, 0.25 * *up.mass);
        const double h  = Thermo::enthalpy(up.T, up.y);
        const double db = dm * up.y;
        *up.mass -= dm; *up.energy -= dm * h; *up.burned -= db;
        *dn.mass += dm; *dn.energy += dm * h; *dn.burned += db;
        return (aUp ? dm : -dm) / dt;
    };

    // Transport a known mass flow (sign gives the direction) between nodes.
    auto convect = [&](Node a, Node b, double mdot) {
        const bool aUp = mdot >= 0.0;
        Node& up = aUp ? a : b;
        Node& dn = aUp ? b : a;
        const double dm = std::min(std::abs(mdot) * dt, 0.25 * *up.mass);
        const double h  = Thermo::enthalpy(up.T, up.y);
        const double db = dm * up.y;
        *up.mass -= dm; *up.energy -= dm * h; *up.burned -= db;
        *dn.mass += dm; *dn.energy += dm * h; *dn.burned += db;
        return dm;
    };

    // ---- Exhaust pipe heat loss --------------------------------------------
    // An exhaust manifold is a radiator, and a hot one is losing kilowatts.
    // Without this the gas arrives at the collector at port temperature, which
    // reads hundreds of degrees above anything a real probe sees; with it, a
    // cold pipe at light load also reads properly cool, because the wall has
    // not had time to come up.
    //
    // The wall settles where convection in balances convection and radiation
    // out. Radiation is what dominates once it glows, and it is the term that
    // pins a manifold near 800 C at full load however hard you drive it.
    auto pipeLoss = [&](double& gasEnergy, double gasMass, double gasTemp,
                        double& wallTemp, double area, double mdot) {
        if (area <= 1e-9 || gasMass <= 1e-9) return;
        const double h = 90.0 + 2400.0 * std::pow(std::abs(mdot), 0.8);
        const double q = h * area * (gasTemp - wallTemp);
        gasEnergy -= q * dt;
        const double t4 = wallTemp * wallTemp * wallTemp * wallTemp;
        const double out = 22.0 * area * (wallTemp - Thermo::tAmb) +
                           4.54e-8 * area * (t4 - kAmbient4);
        // Thin steel: about 5.9 kJ per square metre per kelvin, which is what
        // gives a manifold its half-minute of thermal lag.
        wallTemp += (q - out) / (5850.0 * area) * dt;
        wallTemp = std::clamp(wallTemp, Thermo::tAmb, 1600.0);
    };

    // ---- Ambient / compressor reservoir ------------------------------------
    double ambMass = 1.0e9, ambEnergy = 0.0, ambBurned = 0.0;
    auto ambientNode = [&](double y) {
        ambMass = 1.0e9;
        ambEnergy = 0.0;
        ambBurned = 1.0e9 * y;
        return Node{&ambMass, &ambEnergy, &ambBurned, Thermo::pAmb, Thermo::tAmb, y};
    };
    auto upstreamNode = [&] {
        ambMass = 1.0e9;
        ambEnergy = 0.0;
        ambBurned = 0.0;
        return Node{&ambMass, &ambEnergy, &ambBurned, m_boostPressure, m_boostTemp, 0.0};
    };

    auto plenumNode = [&] {
        const double y = std::clamp(m_plenumBurned / std::max(m_plenumMass, 1e-12), 0.0, 1.0);
        return Node{&m_plenumMass, &m_plenumEnergy, &m_plenumBurned,
                    m_plenumPressure, m_plenumTemp, y};
    };
    auto collector = [&](std::size_t b) {
        return Node{&m_collectorMass[b], &m_collectorEnergy[b], &m_collectorBurned[b],
                    m_collectorPressure[b], m_collectorTemp[b], 1.0};
    };

    // ---- Throttle: upstream <-> plenum --------------------------------------
    const double throttleMaxArea = kPi * 0.25 * m_p.throttleBore * m_p.throttleBore;

    // Idle control: PI on the bypass valve, active only near closed throttle.
    // The integral term is what lets the engine hold a steady idle as the load
    // changes. On a compression-ignition engine there is no throttle plate at
    // all, so the same loop trims fuel instead of air - which is exactly what a
    // diesel governor does.
    // The crank speed ripples within every cycle, so the loop reads a filtered
    // speed the way a real controller reads a filtered wheel count.
    const double rpmPrev = m_rpmFilt;
    m_rpmFilt += (rpm() - m_rpmFilt) * std::min(1.0, 12.0 * dt);
    if (dt > 0.0)
        m_rpmRate += ((m_rpmFilt - rpmPrev) / dt - m_rpmRate) * std::min(1.0, 20.0 * dt);
    if (m_throttle < 0.02) {
        // PID. The derivative term is what matters here: an idling engine is
        // almost unloaded, so a small change in air is a large change in
        // acceleration, and proportional-integral alone limit-cycles.
        // Air is the slow half: it carries the steady state and little else.
        // The fast proportional and derivative action lives on the spark trim
        // below, because torque follows timing within one firing where it takes
        // several cycles to follow the manifold. Running both loops hard makes
        // them fight, and the engine hunts by hundreds of rpm.
        const double err  = (m_p.idleTargetRpm - m_rpmFilt) / m_p.idleTargetRpm;
        const double derr = -m_rpmRate / m_p.idleTargetRpm;
        m_idleIntegral = std::clamp(m_idleIntegral + err * dt * 0.25, 0.0, 1.0);
        m_idleCmd = std::clamp(m_idleIntegral + 0.35 * err + 0.25 * derr, 0.0, 1.0);
    } else {
        m_idleCmd = std::clamp(m_idleCmd - dt * 2.0, 0.0, 1.0);
        m_idleIntegral = std::clamp(m_idleIntegral, 0.25, 1.0);
    }
    // Anti-stall dashpot. Deep vacuum means a charge so dilute it will not
    // burn, so the valve is floored open whenever manifold pressure collapses -
    // which is what stops a real engine stalling as it comes off the throttle.
    // The threshold sits well below the normal idle operating point so this
    // never fights the PI loop.
    const double antiStall = std::clamp((14.0e3 - m_plenumPressure) / 8.0e3, 0.0, 1.0);
    m_idleCmd = std::max(m_idleCmd, antiStall);
    const double bypass = m_p.idleBypassMin +
                          m_idleCmd * (m_p.idleBypassMax - m_p.idleBypassMin);
    // A butterfly's effective area is very non-linear off its seat; squaring the
    // pedal command approximates that. The bypass is an area fraction, so it is
    // added after the curve rather than fed through it.
    const double openFraction = diesel ? 1.0
                                       : bypass + (1.0 - bypass) * m_throttle * m_throttle;
    const double throttleArea = 0.9 * throttleMaxArea * openFraction;
    double throttleFlow = 0.0;
    {
        Node up = upstreamNode();
        throttleFlow = exchange(up, plenumNode(), throttleArea);
        const double y = std::clamp(m_plenumBurned / std::max(m_plenumMass, 1e-9), 0.0, 1.0);
        m_plenumTemp = Thermo::temperature(m_plenumEnergy / std::max(m_plenumMass, 1e-9), y);
        m_plenumPressure = m_plenumMass * Thermo::R * m_plenumTemp / m_p.plenumVolume;
    }
    updateCharger(dt, std::max(0.0, throttleFlow));

    double gasTorque   = 0.0;
    double inertiaTorque = 0.0;
    double peakForFriction = 0.0;

    const double meanPistonSpeed = 2.0 * m_p.stroke * std::abs(m_omega) / (2.0 * kPi);
    const bool   fuelCut = !m_ignition || rpm() > m_p.redline;

    // ---- Cam phaser ----------------------------------------------------------
    // Advanced at low speed, where an early intake close traps the most charge;
    // returned as speed rises, where the column's own momentum does that job and
    // overlap is worth more.
    {
        const double f = std::clamp((rpm() - m_p.vvtLowRpm) /
                                    std::max(200.0, m_p.vvtHighRpm - m_p.vvtLowRpm), 0.0, 1.0);
        const double want = m_p.vvtRange * (1.0 - f);
        m_camAdvance += (want - m_camAdvance) * std::min(1.0, 6.0 * dt);
    }

    // ---- Valve float ---------------------------------------------------------
    // Past the float speed the spring can no longer keep the follower against
    // the cam. Lift collapses, and with it the engine's ability to breathe -
    // this is the wall a pushrod engine hits and a pneumatic-valve one does not.
    m_floatLoss = std::clamp((rpm() - m_p.valveFloatRpm) / 900.0, 0.0, 1.0);
    const double liftScale = 1.0 - 0.80 * m_floatLoss;

    // Spark and fuel maps. Advance climbs with speed and, because a light
    // charge burns slowly, with vacuum; mixture is stoichiometric on part
    // throttle and enriched at full load to hold peak pressure down.
    const double loadFrac = std::clamp(m_plenumPressure / Thermo::pAmb, 0.0, 1.6);
    const double speedFrac = std::clamp((rpm() - 1000.0) / 4500.0, 0.0, 1.0);
    const double loadForMix = std::clamp(loadFrac, 0.0, 1.0);
    m_lambda = m_p.lambdaCruise + (m_p.lambdaPower - m_p.lambdaCruise) *
                                  std::clamp((loadFrac - 0.55) / 0.45, 0.0, 1.0);

    // A carburettor is not told what to deliver: it meters fuel by the
    // depression a venturi makes, which goes with the square of airflow while
    // the fuel it draws goes more nearly linearly, so it richens as it is asked
    // for more. Nothing corrects it, and the mixture wanders with speed as well
    // as load. Injection holds the number it was given.
    if (m_p.fuelSystem == 0) {
        const double flowFrac = std::clamp(loadFrac * rpm() / std::max(m_p.redline, 1.0),
                                           0.0, 1.2);
        m_lambda *= 1.0 - 0.16 * flowFrac;              // richens with flow
        // ... and goes lean the instant the throttle is opened, because the air
        // arrives before the fuel film in the port does.
        const double opening = std::max(0.0, m_throttle - m_throttleLagged);
        m_throttleLagged += (m_throttle - m_throttleLagged) * std::min(1.0, 6.0 * dt);
        m_lambda *= 1.0 + 0.9 * opening;
        m_lambda = std::clamp(m_lambda, 0.62, 1.35);
    }
    m_afr = m_lambda * m_p.fuel.stoichAfr;

    // Knock feedback. A knock sensor cannot tell the ECU how to avoid knock,
    // only that it is happening, so the correction is reactive: pull timing
    // fast, give it back slowly.
    m_knockLevel *= std::exp(-dt * 1.5);
    if (m_p.knockControl) {
        const double want = std::clamp(m_knockLevel * 3.0, 0.0, 1.0) * m_p.knockRetardMax;
        if (want > m_knockRetard) m_knockRetard += (want - m_knockRetard) * std::min(1.0, 25.0 * dt);
        else                      m_knockRetard = std::max(0.0, m_knockRetard - dt * 1.2);
    } else {
        m_knockRetard = 0.0;
    }

    // Every boosted spark map pulls timing as manifold pressure climbs past
    // ambient: the charge is denser, the flame faster, and the end gas that
    // much closer to lighting on its own. A diesel has the opposite problem -
    // it wants its fuel in early - so this applies only where there is a spark.
    const double boostRetard = diesel ? 0.0
        : 8.0 * std::max(0.0, m_plenumPressure / Thermo::pAmb - 1.0);
    // Spark is the fast half of idle control. Air takes several cycles to
    // arrive through the manifold; timing changes the torque of the very next
    // firing, so a real ECU holds the base advance a little retarded at idle
    // and trims it either way to catch a disturbance. Without it a long-duration
    // cam - which idles on a dilute, slow-burning charge anyway - hunts by
    // hundreds of rpm while the air loop chases it.
    // Only once it is actually running, though. Advancing the spark at cranking
    // speed makes the engine fight itself well before top dead centre and it
    // will never catch - which is exactly why a real ECU cranks retarded.
    double idleSparkTrim = 0.0;
    if (!diesel && m_throttle < 0.02 &&
        m_rpmFilt > m_p.idleTargetRpm * 0.55 && m_rpmFilt < m_p.idleTargetRpm * 1.8) {
        const double err  = (m_p.idleTargetRpm - m_rpmFilt) / m_p.idleTargetRpm;
        const double derr = m_rpmRate / m_p.idleTargetRpm;
        idleSparkTrim = std::clamp(9.0 * err - 5.0 * derr, -8.0, 8.0);
    }

    m_sparkAdvance = m_p.sparkIdle +
                     (m_p.sparkHighSpeed - m_p.sparkIdle) * speedFrac +
                     m_p.sparkPartLoad * (1.0 - loadForMix) - m_knockRetard -
                     boostRetard + idleSparkTrim;
    m_sparkAdvance = std::clamp(m_sparkAdvance, -10.0, 60.0);
    const double sparkPhase = wrap720(720.0 - m_sparkAdvance);
    const double a  = 0.5 * m_p.stroke;
    const double l  = m_p.rodLength;
    const double boreFactor = std::pow(m_p.bore, -0.2);

    // Fuel demand on a compression-ignition engine is the pedal itself: there
    // is no throttle, the rack just meters more fuel into the same air.
    // The governor needs enough authority to carry the engine s own friction at
    // idle, which on a big diesel with a heavy flywheel is a good deal more fuel
    // than a token trickle.
    const double fuelDemand = std::clamp(std::max(m_throttle, m_idleCmd * 0.45), 0.0, 1.0);

    // Charge cooling. Port-injected fuel evaporates in the runner and takes its
    // latent heat out of the air going past it. On petrol this is worth a few
    // degrees; on methanol it is worth a hundred, and it is most of why alcohol
    // fuels make power.
    const double coolPerKgAir = diesel ? 0.0
                                       : m_p.fuel.vaporHeat * m_p.fuel.portEvap /
                                         std::max(m_afr, 3.0);

    for (std::size_t ci = 0; ci < m_cyl.size(); ++ci) {
        Cylinder& c = m_cyl[ci];
        const double offset    = ci < m_p.phaseOffsets.size() ? m_p.phaseOffsets[ci] : 0.0;
        const double prevPhase = c.phase;
        c.phase = wrap720(m_crankAngle + offset);

        const double Vprev = c.volume;
        // Volume, its derivative and the reciprocating inertia all want the
        // sine and cosine of the same angle. At twelve cylinders and six
        // sub-steps a sample that is a lot of trigonometry to repeat.
        const double th = c.phase * kDegRad;
        const double sinTh = std::sin(th);
        const double cosTh = std::cos(th);
        c.volume = volumeFromTrig(sinTh, cosTh);
        const double dV = c.volume - Vprev;

        c.intakeLift  = lookupLift(m_intakeLift, c.phase + m_camAdvance) * liftScale;
        c.exhaustLift = lookupLift(m_exhaustLift, c.phase) * liftScale;
        const double aInt = portArea(m_p.intake,  c.intakeLift);
        const double aExh = portArea(m_p.exhaust, c.exhaustLift);

        auto cylNode = [&] {
            const double y = std::clamp(c.burnedMass / std::max(c.mass, 1e-12), 0.0, 1.0);
            return Node{&c.mass, &c.energy, &c.burnedMass, c.pressure, c.temperature, y};
        };
        auto runNode = [&] {
            const double y = std::clamp(c.runnerBurned / std::max(c.runnerMass, 1e-12), 0.0, 1.0);
            return Node{&c.runnerMass, &c.runnerEnergy, &c.runnerBurned,
                        c.runnerPressure, c.runnerTemp, y};
        };
        auto exRunNode = [&] {
            return Node{&c.exRunnerMass, &c.exRunnerEnergy, &c.exRunnerBurned,
                        c.exRunnerPressure, c.exRunnerTemp, 1.0};
        };

        // ---- Runner columns --------------------------------------------------
        // A runner is not a restriction but an inertance: the gas column has to
        // be accelerated, which is what makes ram charging and the exhaust
        // extraction pulse possible.
        // Momentum of the gas column: dm/dt = A/L * (dp - losses). Without the
        // loss terms the column rings forever; real runners lose energy to
        // form drag at the bends and to wall friction.
        auto accelerate = [&](double& mdot, double pUp, double pDn, double rho,
                              const RunnerSpec& spec) {
            const double v2loss = 0.5 * spec.zeta * mdot * std::abs(mdot) /
                                  (std::max(rho, 0.05) * spec.area * spec.area);
            const double vloss = spec.viscous * mdot;
            const double dmdot = (spec.area / spec.length) * (pUp - pDn - v2loss - vloss);
            mdot += dmdot * dt;
        };

        const double rhoRunner = c.runnerPressure / (Thermo::R * c.runnerTemp);
        accelerate(c.intakeFlowRate, m_plenumPressure, c.runnerPressure,
                   rhoRunner, m_p.intakeRunner);
        const double intoRunner = convect(plenumNode(), runNode(), c.intakeFlowRate);
        // The injector sits in the port, so the evaporation happens here.
        if (c.intakeFlowRate > 0.0 && coolPerKgAir > 0.0)
            c.runnerEnergy -= intoRunner * coolPerKgAir;

        const double rhoEx = c.exRunnerPressure / (Thermo::R * c.exRunnerTemp);
        const std::size_t bank = ci < m_p.cylinderBank.size()
                               ? std::min(static_cast<std::size_t>(std::max(0, m_p.cylinderBank[ci])),
                                          m_bankCount - 1)
                               : 0;
        accelerate(c.exhaustFlowRate, c.exRunnerPressure, m_collectorPressure[bank],
                   rhoEx, m_p.exhaustRunner);
        convect(exRunNode(), collector(bank), c.exhaustFlowRate);
        pipeLoss(c.exRunnerEnergy, c.exRunnerMass, c.exRunnerTemp, c.exWallTemp,
                 m_exRunnerWallArea, c.exhaustFlowRate);

        // Refresh the runner states before the valves see them.
        auto refreshRunner = [&] {
            const double y = std::clamp(c.runnerBurned / std::max(c.runnerMass, 1e-12), 0.0, 1.0);
            c.runnerTemp = Thermo::temperature(c.runnerEnergy / std::max(c.runnerMass, 1e-12), y);
            c.runnerPressure = c.runnerMass * Thermo::R * c.runnerTemp / runnerVol;
            c.exRunnerTemp = Thermo::temperature(c.exRunnerEnergy / std::max(c.exRunnerMass, 1e-12), 1.0);
            c.exRunnerPressure = c.exRunnerMass * Thermo::R * c.exRunnerTemp / exRunnerVol;
        };
        refreshRunner();

        // ---- Travelling waves -------------------------------------------------
        // Each port launches a wave into its runner and hears one come back a
        // round trip later. What returns is added to the static runner pressure
        // when the valves are worked out below, so a pipe's *length* decides
        // when it helps - the extraction pulse that scavenges an exhaust port,
        // and the returning compression that rams an intake one.
        {
            const double cIn = std::sqrt(1.4 * Thermo::R * std::max(120.0, c.runnerTemp));
            const double cEx = std::sqrt(1.33 * Thermo::R * std::max(120.0, c.exRunnerTemp));
            // Flow into the pipe at the valve: the exhaust is blown into, the
            // intake is drawn out of, and the sign of the launched wave follows.
            c.intakeWavePa  = c.intakeWave.step(dt, -c.valveFlowIntake, cIn,
                                                c.runnerPressure);
            c.exhaustWavePa = c.exhaustWave.step(dt, c.valveFlowExhaust, cEx,
                                                 c.exRunnerPressure);
        }

        // ---- Valves ----------------------------------------------------------
        Node runPort = runNode();
        Node exPort  = exRunNode();
        runPort.p = std::max(500.0, runPort.p + c.intakeWavePa);
        exPort.p  = std::max(500.0, exPort.p + c.exhaustWavePa);
        const double intoCyl = exchange(runPort, cylNode(), aInt);
        const double outCyl  = -exchange(exPort, cylNode(), aExh);
        c.valveFlowIntake  = intoCyl;
        c.valveFlowExhaust = outCyl;
        if (intoCyl > 0.0) {
            const double yRun = std::clamp(c.runnerBurned / std::max(c.runnerMass, 1e-12), 0.0, 1.0);
            c.freshTrapped += intoCyl * (1.0 - yRun) * dt;
        }
        refreshRunner();

        // ---- Charge state after the flows ------------------------------------
        auto refreshCylinder = [&] {
            c.mass = std::max(c.mass, 1e-9);
            c.burnedMass = std::clamp(c.burnedMass, 0.0, c.mass);
            const double y = c.burnedMass / c.mass;
            c.temperature = Thermo::temperature(c.energy / c.mass, y);
            c.pressure = c.mass * Thermo::R * c.temperature / c.volume;
        };
        refreshCylinder();
        const double yCyl = c.burnedMass / c.mass;

        // ---- Ignition ---------------------------------------------------------
        const bool crossedSpark = prevPhase != c.phase && inWindow(sparkPhase, prevPhase, c.phase);
        if (crossedSpark) {
            // Residual dilution slows the flame and, past roughly a third of
            // the charge, puts it out entirely.
            // Dilution does not switch combustion off, it degrades it: the
            // flame gets slower and less complete until it fails to propagate.
            const double dilutionEff = std::clamp(1.0 - 1.6 * std::max(0.0, yCyl - 0.25), 0.0, 1.0);
            const bool misfire = yCyl > m_p.misfireLimit;
            // A diesel has no spark: it needs the charge to already be hot
            // enough to light the injected fuel, which is why a cold one will
            // crank all day without catching.
            const bool willLight = !diesel || c.temperature > m_p.fuel.autoIgnitionK * 0.72;
            if (!fuelCut && !misfire && willLight && fuelDemand > 1e-3) {
                c.burning       = true;
                c.burnFraction  = 0.0;
                c.chargeAtSpark = c.mass;
                c.knockIntegral = 0.0;

                if (diesel) {
                    // Ignition delay is a chemical clock: the hotter the
                    // charge, the sooner it goes off. A cold engine's long
                    // delay is what makes a diesel rattle on a winter morning.
                    const double tRatio = std::clamp(m_p.fuel.autoIgnitionK / c.temperature, 0.4, 3.0);
                    const double delay  = m_p.ignitionDelay * tRatio * tRatio;
                    c.burnStart = wrap720(sparkPhase + delay);
                    // Fuelling is limited by air, not by air by fuelling: past
                    // the smoke limit more fuel is just soot.
                    const double maxFuel = c.airTrapped / std::max(m_p.fuel.smokeAfr, 8.0);
                    c.fuelMass  = maxFuel * fuelDemand;
                    m_afr    = c.airTrapped / std::max(c.fuelMass, 1e-9);
                    m_lambda = m_afr / m_p.fuel.stoichAfr;
                    // Diffusion burning is slower than a premixed flame and
                    // gets slower the more fuel there is to find air for.
                    c.burnSpan = m_p.burnDuration * (0.75 + 0.45 * fuelDemand) /
                                 std::max(m_p.fuel.flameSpeed, 0.2);
                } else {
                    c.burnStart = wrap720(sparkPhase + m_p.ignitionDelay);
                    // Burn duration in crank angle grows slowly with speed and
                    // sharply with dilution, and a mixture away from its
                    // fastest-burning point (slightly rich) burns slower still.
                    const double speedTerm = 0.80 + 0.20 * std::min(2.5, rpm() / 3000.0);
                    const double mixTerm = 1.0 + 1.8 * (m_lambda - 0.9) * (m_lambda - 0.9);
                    c.burnSpan = m_p.burnDuration * speedTerm * (1.0 + 1.6 * yCyl) * mixTerm /
                                 std::max(m_p.fuel.flameSpeed, 0.2);
                    // Only the unburned part of the charge carries fuel.
                    c.fuelMass = c.mass * (1.0 - yCyl) / std::max(m_afr, 3.0);
                }
                // Below stoichiometric there is not enough oxygen to burn all
                // of the fuel, and what cannot burn leaves as carbon monoxide.
                // Releasing all of it anyway is what made a rich full-load
                // mixture produce more heat than the air could support - too
                // much peak pressure, too much knock, and exhaust temperatures
                // hundreds of degrees above anything real.
                const double oxygenLimit = std::min(1.0, m_lambda);
                c.fuelEnergy = c.fuelMass * m_p.fuel.lhv * m_p.combustionEff *
                               dilutionEff * oxygenLimit;
                m_fuelAccum += c.fuelMass;
            }
        }

        double dQcomb = 0.0;
        if (c.burning) {
            double since = c.phase - c.burnStart;
            if (since < 0.0) since += 720.0;
            if (since > 0.0 && since < 360.0) {
                const double u = since / c.burnSpan;
                if (u >= 1.0) {
                    dQcomb         = (1.0 - c.burnFraction) * c.fuelEnergy;
                    c.burnFraction = 1.0;
                    c.burning      = false;
                } else {
                    const double xb = 1.0 - std::exp(-5.0 * u * u * u);  // Wiebe, a=5, m=2
                    dQcomb          = (xb - c.burnFraction) * c.fuelEnergy;
                    // The charge that burns becomes product.
                    c.burnedMass   += (xb - c.burnFraction) * c.chargeAtSpark * (1.0 - yCyl);
                    c.burnFraction  = xb;
                }
            }
        }

        // ---- Knock ------------------------------------------------------------
        // The end gas is compressed by the flame front as much as by the piston.
        // Douaud-Eyzat gives its induction time; Livengood-Wu integrates the
        // reciprocal of that time, and when the integral reaches one the end gas
        // lights on its own - all of it at once, which is the noise.
        // Once the flame has crossed most of the chamber there is no end gas
        // left to detonate, and the pressure peak that follows belongs to gas
        // that has already burned. Integrating through it was most of why this
        // model heard knock everywhere: the largest contributions were being
        // collected exactly where the term no longer applies.
        if (!diesel && c.burning && c.knockIntegral >= 0.0 &&
            c.burnFraction < 0.85 && c.pressure > 5.0e5) {
            const double tu = std::clamp(
                c.refTemp * std::pow(c.pressure / std::max(c.refPressure, 1.0e4), 0.2857),
                300.0, 1400.0);
            const double pAtm = c.pressure / 101325.0;
            // The octane term depends on nothing that changes while running, so
            // it is worked out once when the engine is built.
            double tau = m_knockFuelTerm * std::pow(pAtm, -1.7) * std::exp(3800.0 / tu);
            // Knock resistance is a bowl, not a slope. It is worst just lean of
            // stoichiometric, where the flame is hottest; enrichment cools the
            // charge, and running properly lean drops peak temperature far
            // enough that a hydrogen or lean-burn engine barely knocks at all.
            const double dl = m_lambda - 1.05;
            tau *= std::clamp(0.90 + 6.0 * dl * dl + (dl < 0.0 ? -1.6 * dl : 0.0), 0.85, 5.0);
            // Douaud and Eyzat fitted their correlation to one engine on one
            // rig, and it predicts knock early on anything else. Like Woschni
            // above, it gets the calibration multiplier such correlations are
            // normally used with - set so a stock 10.5:1 engine on 95 RON is
            // knock-limited around 30 degrees of advance, which is where a real
            // one is.
            tau *= m_p.knockScale;
            c.knockIntegral += dt / std::max(tau, 1.0e-5);
            if (c.knockIntegral >= 1.0) {
                const double remaining = std::max(0.0, 1.0 - c.burnFraction);
                // What is left of the charge goes off in one step.
                const double burst = 0.7 * remaining;
                dQcomb += burst * c.fuelEnergy;
                c.burnedMass += burst * c.chargeAtSpark * (1.0 - yCyl);
                c.burnFraction = std::min(1.0, c.burnFraction + burst);
                c.knockShock = std::max(c.knockShock, remaining);
                // The gauge tracks how hard each event is, not how many there
                // have been - accumulating would peg it at full scale after a
                // second of the mildest detonation.
                m_knockLevel += (remaining - m_knockLevel) * 0.35;
                c.knockIntegral = -1.0;   // one event per cycle
            }
        }
        c.knockShock *= std::exp(-dt * 30.0);

        // ---- Wall heat transfer (Woschni) -------------------------------------
        // The second velocity term is what makes heat loss during combustion
        // realistic: without it, peak pressure comes out far too high.
        double w = 2.28 * meanPistonSpeed + 0.6;
        if (c.motoredPressure > 0.0 && c.pressure > c.motoredPressure) {
            w += 0.00324 * (m_displacement * c.refTemp) / (c.refPressure * c.refVolume) *
                 (c.pressure - c.motoredPressure);
        }
        const double hc = m_p.heatTransferScale * 3.26 * boreFactor *
                          std::pow(c.pressure * 0.001, 0.8) *
                          std::pow(c.temperature, -0.55) * std::pow(w, 0.8);
        const double area = 2.0 * m_pistonArea + kPi * m_p.bore * (c.volume / m_pistonArea);
        // Detonation scrubs the boundary layer off the chamber walls, which is
        // what actually melts pistons.
        const double dQwall = hc * area * (c.temperature - m_p.wallTemp) * dt *
                              (1.0 + 2.0 * c.knockShock);

        // Motored pressure reference, integrated polytropically. dV per sub-step
        // is tiny, so the first-order expansion of (V_prev/V)^gamma is exact
        // enough and avoids a pow().
        if (c.motoredPressure > 0.0)
            c.motoredPressure *= 1.0 - 1.35 * dV / c.volume;

        // ---- First law --------------------------------------------------------
        c.energy += dQcomb - dQwall - c.pressure * dV;
        refreshCylinder();

        // Gas torque: dW = p dV, so tau = p dV/dtheta. The underside of the
        // piston sits at crankcase (ambient) pressure, hence the offset.
        gasTorque += (c.pressure - Thermo::pAmb) * dVolumeFromTrig(sinTh, cosTh);

        // Reciprocating inertia: the piston and small end have to be stopped
        // and restarted twice a revolution. It integrates to zero over a cycle
        // but is what gives the crank its within-cycle speed ripple.
        {
            // Double-angle identities rather than two more trig calls.
            const double sin2 = 2.0 * sinTh * cosTh;
            const double cos2 = 1.0 - 2.0 * sinTh * sinTh;
            const double dx  = -a * sinTh - (a * a / (2.0 * l)) * sin2;
            const double ddx = -a * cosTh - (a * a / l) * cos2;
            inertiaTorque -= m_p.recipMass * m_omega * m_omega * dx * ddx;
        }

        peakForFriction = std::max(peakForFriction, c.pressure);

        // ---- Per-cycle bookkeeping at intake valve closing --------------------
        const double ivc = wrap720(m_p.intake.closeDeg - m_camAdvance);
        if (prevPhase != c.phase && inWindow(ivc, prevPhase, c.phase)) {
            c.refPressure     = c.pressure;
            c.refVolume       = c.volume;
            c.refTemp         = c.temperature;
            c.motoredPressure = c.pressure;
            c.airTrapped      = c.mass * (1.0 - c.burnedMass / c.mass);
            if (ci == 0) {
                const double rhoRef = Thermo::pAmb / (Thermo::R * Thermo::tAmb);
                m_volEff   = c.freshTrapped / (rhoRef * m_displacement);
                m_residual = c.burnedMass / c.mass;
            }
            c.freshTrapped = 0.0;
        }

        if (ci == 0) {
            const double bar = c.pressure * 1e-5;
            m_peakAccum = std::max(m_peakAccum, bar);
            if (c.phase < prevPhase) { m_peakPressure = m_peakAccum; m_peakAccum = 0.0; }
            const std::size_t bin = static_cast<std::size_t>(c.phase) % kTraceBins;
            m_trace[bin].store(static_cast<float>(bar), std::memory_order_relaxed);
        }
    }

    // ---- Collector vents to atmosphere ---------------------------------------
    // A turbine sits in this path on a turbocharged engine, and the exhaust has
    // to be pushed through it: that restriction is the price of the boost.
    {
        Node amb = ambientNode(1.0);
        // Each bank vents through its share of the outlet.
        const double outlet = m_p.outletArea / static_cast<double>(m_bankCount) *
                              (m_p.charger == 1 ? std::clamp(m_p.turbineRestrict, 0.2, 1.0) : 1.0);
        for (std::size_t b = 0; b < m_bankCount; ++b) {
            Node amb2 = amb;
            exchange(collector(b), amb2, outlet);
        }
    }
    for (std::size_t b = 0; b < m_bankCount; ++b) {
        m_collectorMass[b] = std::max(m_collectorMass[b], 1e-9);
        m_collectorTemp[b] = Thermo::temperature(m_collectorEnergy[b] / m_collectorMass[b], 1.0);
        // Everything a bank breathes passes through its collector, so that
        // bank's own share of the throughput sets the film coefficient.
        pipeLoss(m_collectorEnergy[b], m_collectorMass[b], m_collectorTemp[b],
                 m_collectorWall[b], m_collectorWallArea / static_cast<double>(m_bankCount),
                 m_throttleFlow / static_cast<double>(m_bankCount));
        m_collectorTemp[b] = Thermo::temperature(m_collectorEnergy[b] / m_collectorMass[b], 1.0);
        m_collectorPressure[b] = m_collectorMass[b] * Thermo::R * m_collectorTemp[b] /
                                 m_collectorVolEach;
    }

    // ---- Oil -----------------------------------------------------------------
    // Oil warms towards a load-dependent temperature, and its viscosity there
    // is what sets both the hydrodynamic friction and the pressure the pump can
    // build. Cold oil costs power; hot thin oil costs oil pressure.
    {
        const double duty = std::clamp(rpm() / 2600.0, 0.0, 1.0) * (0.45 + 0.55 * loadForMix);
        const double target = m_p.oilStartTemp + (m_p.oilTempTarget - m_p.oilStartTemp) *
                                                 std::clamp(0.25 + duty, 0.0, 1.15);
        m_oilTemp += (target - m_oilTemp) * std::min(1.0, dt / 55.0);
    }
    // Viscosity between the two grade points, log-interpolated and extrapolated
    // past them - which is exactly how a multigrade is specified.
    const double viscMix = std::clamp((m_oilTemp - 313.0) / 60.0, -0.9, 1.8);
    const double visc = std::max(0.15, m_p.oil.viscosity40 *
        std::pow(std::max(m_p.oil.viscosity100, 0.05) / std::max(m_p.oil.viscosity40, 0.05), viscMix));
    m_oilPressure = std::min(5.2e5, 0.30e5 + rpm() * 100.0 * std::pow(visc, 0.8));

    // ---- Crank dynamics --------------------------------------------------------
    // Chen-Flynn: friction rises with peak cylinder pressure and with the
    // square of piston speed, which is what rolls the torque curve off at both
    // ends instead of leaving it flat.
    // Friction responds to the cycle's peak pressure, not to the pressure at
    // this instant, so the peak is held and bled off slowly.
    m_pmaxHold = std::max(peakForFriction, m_pmaxHold * (1.0 - 3.0 * dt));
    const double pmaxBar = m_pmaxHold * 1e-5;
    const double viscTerm = std::pow(visc, 0.6) * m_p.valvetrainDrag;
    const double boundary = m_p.cfA * (0.55 + 0.45 / std::clamp(m_p.oil.filmStrength, 0.4, 2.0));
    const double fmepBar = boundary + m_p.cfB * pmaxBar +
                           (m_p.cfC * meanPistonSpeed +
                            m_p.cfD * meanPistonSpeed * meanPistonSpeed) * viscTerm;
    m_fmep = fmepBar * 1e5;
    const double totalDisp = m_displacement * m_p.cylinders;
    // FMEP is a mean pressure per swept volume per cycle; a four-stroke turns
    // 4*pi radians per cycle.
    const double frictionTorque = m_fmep * totalDisp / (4.0 * kPi) + m_p.accessoryTorque +
                                  m_blowerTorque;

    const double dir = m_omega > 0.0 ? 1.0 : 0.0;   // no braking torque at rest
    double torque = gasTorque + inertiaTorque - frictionTorque * dir;

    if (m_starter && rpm() < 320.0) torque += m_p.starterTorque;

    // The load is whatever the clutch is holding back: in neutral that is
    // nothing, in gear it is the car.
    torque += m_drive.step(dt, m_omega);

    if (m_speedHold) {
        // On the dyno the brake holds the speed and the torque it has to absorb
        // is the number being measured, so the crank does not accelerate at all.
        m_omega = m_holdOmega;
    } else {
        m_omega += torque / m_p.inertia * dt;
        if (m_omega < 0.0) m_omega = 0.0;   // the crank never spins backwards here
    }

    m_netTorque = torque;
    const double brakeTorque = gasTorque - frictionTorque * dir;
    m_torqueFilt += (brakeTorque - m_torqueFilt) * std::min(1.0, 6.0 * dt);

    // Fuel flow, averaged over a window long enough to cover a whole cycle at
    // any speed so the readout does not flicker with each firing.
    m_fuelWindow += dt;
    if (m_fuelWindow > 0.25) {
        m_fuelRate += (m_fuelAccum / m_fuelWindow - m_fuelRate) * 0.5;
        m_fuelAccum = 0.0;
        m_fuelWindow = 0.0;
    }
}

Snapshot Engine::snapshot() const {
    Snapshot s;
    s.rpm              = static_cast<float>(rpm());
    s.manifoldPressure = static_cast<float>(m_plenumPressure * 0.001);
    s.torque           = static_cast<float>(m_torqueFilt);
    s.power            = static_cast<float>(m_torqueFilt * m_omega * 0.001);
    s.fmep             = static_cast<float>(m_fmep * 1e-5);
    s.throttle         = static_cast<float>(m_throttle);
    s.gear             = m_drive.gear();
    s.gearCount        = m_drive.gearCount();
    s.speedKph         = static_cast<float>(m_drive.speedKph());
    s.clutchLock       = static_cast<float>(m_drive.clutchLock());
    s.clutchSlip       = static_cast<float>(m_drive.slip() * 30.0 / kPi);
    s.wheelTorque      = static_cast<float>(m_drive.wheelTorque());
    s.brake            = static_cast<float>(m_drive.brake());
    s.crankAngle       = static_cast<float>(m_crankAngle);
    s.peakPressure     = static_cast<float>(m_peakPressure);
    s.exhaustTemp      = static_cast<float>(m_collectorTemp[0]);
    s.backPressure     = static_cast<float>(m_collectorPressure[0] * 0.001);
    s.volumetricEff    = static_cast<float>(m_volEff);
    s.residualFraction = static_cast<float>(m_residual);
    s.sparkAdvance     = static_cast<float>(m_sparkAdvance);
    s.afr              = static_cast<float>(m_afr);
    s.lambda           = static_cast<float>(m_lambda);
    s.idleValve        = static_cast<float>(m_idleCmd);
    s.boost            = static_cast<float>((m_boostPressure - Thermo::pAmb) * 0.001);
    s.chargeTemp       = static_cast<float>(m_boostTemp);
    s.knock            = static_cast<float>(std::clamp(m_knockLevel, 0.0, 1.0));
    s.knockRetard      = static_cast<float>(m_knockRetard);
    s.oilTemp          = static_cast<float>(m_oilTemp);
    s.oilPressure      = static_cast<float>(m_oilPressure * 1e-5);
    s.camAdvance       = static_cast<float>(m_camAdvance);
    s.valveFloat       = static_cast<float>(m_floatLoss);
    s.fuelFlow         = static_cast<float>(m_fuelRate * 3600.0);
    // Fuel per unit of work only means anything when there is work: on the
    // limiter, or overrunning, the engine is still burning fuel for no output.
    const double kw = m_torqueFilt * m_omega * 0.001;
    s.bsfc             = kw > 3.0 ? static_cast<float>(m_fuelRate * 3.6e6 / kw) : 0.0f;
    s.cylinderCount    = static_cast<int>(m_cyl.size());
    for (std::size_t i = 0; i < m_cyl.size() && i < s.cyl.size(); ++i) {
        const Cylinder& c = m_cyl[i];
        CylinderView& v = s.cyl[i];
        v.phase          = static_cast<float>(c.phase);
        v.pressure       = static_cast<float>(c.pressure * 1e-5);
        v.temperature    = static_cast<float>(c.temperature);
        v.intakeLift     = static_cast<float>(c.intakeLift / std::max(m_p.intake.maxLift, 1e-6));
        v.exhaustLift    = static_cast<float>(c.exhaustLift / std::max(m_p.exhaust.maxLift, 1e-6));
        v.burnFraction   = static_cast<float>(c.burning ? c.burnFraction : 0.0);
        v.burnedGas      = static_cast<float>(c.burnedMass / std::max(c.mass, 1e-12));
        v.runnerPressure = static_cast<float>(c.runnerPressure * 0.001);
        v.exRunnerPressure = static_cast<float>(c.exRunnerPressure * 0.001);
        v.knock          = static_cast<float>(std::clamp(c.knockShock, 0.0, 1.0));
        v.bank           = i < m_p.cylinderBank.size() ? m_p.cylinderBank[i] : 0;
    }
    return s;
}

} // namespace sim
