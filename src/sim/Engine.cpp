#include "sim/Engine.h"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

constexpr double kPi     = 3.14159265358979323846;
constexpr double kDegRad = kPi / 180.0;

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

Engine::Engine(EngineParams params) : m_p(std::move(params)) {
    m_pistonArea   = kPi * 0.25 * m_p.bore * m_p.bore;
    m_displacement = m_pistonArea * m_p.stroke;
    m_clearance    = m_displacement / (m_p.compressionRatio - 1.0);

    buildLiftTable(m_intakeLift, m_p.intake);
    buildLiftTable(m_exhaustLift, m_p.exhaust);

    const double runnerVol   = m_p.intakeRunner.length * m_p.intakeRunner.area;
    const double exRunnerVol = m_p.exhaustRunner.length * m_p.exhaustRunner.area;

    m_cyl.resize(static_cast<std::size_t>(m_p.cylinders));
    for (std::size_t i = 0; i < m_cyl.size(); ++i) {
        const double offset = i < m_p.phaseOffsets.size() ? m_p.phaseOffsets[i] : 0.0;
        Cylinder& c   = m_cyl[i];
        c.phase       = wrap720(offset);
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
    }

    m_plenumMass   = Thermo::pAmb * m_p.plenumVolume / (Thermo::R * Thermo::tAmb);
    m_plenumEnergy = m_plenumMass * Thermo::energy(Thermo::tAmb, 0.0);

    m_collectorMass   = Thermo::pAmb * m_p.collectorVolume / (Thermo::R * Thermo::tAmb);
    m_collectorEnergy = m_collectorMass * Thermo::energy(Thermo::tAmb, 1.0);
    m_collectorBurned = m_collectorMass;

    for (auto& t : m_trace) t.store(0.0f, std::memory_order_relaxed);
}

void Engine::setThrottle(double t) { m_throttle = std::clamp(t, 0.0, 1.0); }

// ---------------------------------------------------------------------------
// Cam profile. A raised cosine has too pointed a nose to pass for a real cam;
// raising it to a power below one flattens the top and steepens the flanks,
// which is roughly what a polydyne profile looks like.
// ---------------------------------------------------------------------------
void Engine::buildLiftTable(std::vector<double>& table, const ValveTiming& v) const {
    table.assign(kLiftBins, 0.0);
    double span = v.closeDeg - v.openDeg;
    if (span < 0.0) span += 720.0;
    for (std::size_t i = 0; i < kLiftBins; ++i) {
        const double phase = static_cast<double>(i) * 720.0 / kLiftBins;
        if (!inWindow(phase, v.openDeg, v.closeDeg)) continue;
        double x = phase - v.openDeg;
        if (x < 0.0) x += 720.0;
        const double u = x / span;
        const double hump = 0.5 * (1.0 - std::cos(2.0 * kPi * u));
        table[i] = v.maxLift * std::pow(hump, 0.82);
    }
}

double Engine::lookupLift(const std::vector<double>& table, double phaseDeg) {
    const double x = wrap720(phaseDeg) * (kLiftBins / 720.0);
    const std::size_t i = static_cast<std::size_t>(x) % kLiftBins;
    const std::size_t j = (i + 1) % kLiftBins;
    const double f = x - std::floor(x);
    return table[i] * (1.0 - f) + table[j] * f;
}

double Engine::cylinderVolume(double phaseDeg) const {
    const double th = phaseDeg * kDegRad;
    const double a  = 0.5 * m_p.stroke;
    const double l  = m_p.rodLength;
    const double s  = a * std::sin(th);
    const double x  = a * std::cos(th) + std::sqrt(l * l - s * s);
    return m_clearance + m_pistonArea * (l + a - x);
}

double Engine::dVolumeDTheta(double phaseDeg) const {
    const double th   = phaseDeg * kDegRad;
    const double a    = 0.5 * m_p.stroke;
    const double l    = m_p.rodLength;
    const double st   = std::sin(th);
    const double ct   = std::cos(th);
    const double root = std::sqrt(l * l - a * a * st * st);
    return m_pistonArea * a * (st + a * st * ct / root);
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

void Engine::advance(double dt) {
    // Cap the crank travel per sub-step so the valve and runner integration
    // stays stable: at 8000 rpm one 44.1 kHz sample already spans ~1.1 deg.
    const double degPerStep = std::abs(m_omega) / kDegRad * dt;
    int steps = static_cast<int>(std::ceil(degPerStep / 0.25));
    steps = std::clamp(steps, 1, 32);
    const double h = dt / steps;
    for (int i = 0; i < steps; ++i) step(h);
}

void Engine::step(double dt) {
    const double omegaDeg = m_omega / kDegRad;   // deg/s
    m_crankAngle = wrap720(m_crankAngle + omegaDeg * dt);

    const double runnerVol   = m_p.intakeRunner.length * m_p.intakeRunner.area;
    const double exRunnerVol = m_p.exhaustRunner.length * m_p.exhaustRunner.area;

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
    };

    // ---- Ambient reservoir -------------------------------------------------
    double ambMass = 1.0e9, ambEnergy = 0.0, ambBurned = 0.0;
    auto ambientNode = [&](double y) {
        ambMass = 1.0e9;
        ambEnergy = 0.0;
        ambBurned = 1.0e9 * y;
        return Node{&ambMass, &ambEnergy, &ambBurned, Thermo::pAmb, Thermo::tAmb, y};
    };

    auto plenumNode = [&] {
        const double y = std::clamp(m_plenumBurned / std::max(m_plenumMass, 1e-12), 0.0, 1.0);
        return Node{&m_plenumMass, &m_plenumEnergy, &m_plenumBurned,
                    m_plenumPressure, m_plenumTemp, y};
    };
    auto collector = [&] {
        return Node{&m_collectorMass, &m_collectorEnergy, &m_collectorBurned,
                    m_collectorPressure, m_collectorTemp, 1.0};
    };

    // ---- Throttle: ambient <-> plenum --------------------------------------
    const double throttleMaxArea = kPi * 0.25 * m_p.throttleBore * m_p.throttleBore;

    // Idle air control: PI on the bypass valve, active only near closed
    // throttle. The integral term is what lets the engine hold a steady idle
    // as the load changes.
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
        const double err  = (m_p.idleTargetRpm - m_rpmFilt) / m_p.idleTargetRpm;
        const double derr = -m_rpmRate / m_p.idleTargetRpm;
        m_idleIntegral = std::clamp(m_idleIntegral + err * dt * 0.15, 0.0, 1.0);
        m_idleCmd = std::clamp(m_idleIntegral + 0.7 * err + 0.8 * derr, 0.0, 1.0);
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
    const double openFraction = bypass + (1.0 - bypass) * m_throttle * m_throttle;
    const double throttleArea = 0.9 * throttleMaxArea * openFraction;
    {
        Node amb = ambientNode(0.0);
        exchange(amb, plenumNode(), throttleArea);
        const double y = std::clamp(m_plenumBurned / std::max(m_plenumMass, 1e-9), 0.0, 1.0);
        m_plenumTemp = Thermo::temperature(m_plenumEnergy / std::max(m_plenumMass, 1e-9), y);
        m_plenumPressure = m_plenumMass * Thermo::R * m_plenumTemp / m_p.plenumVolume;
    }

    double gasTorque   = 0.0;
    double inertiaTorque = 0.0;
    double peakForFriction = 0.0;

    const double meanPistonSpeed = 2.0 * m_p.stroke * std::abs(m_omega) / (2.0 * kPi);
    const bool   fuelCut = !m_ignition || rpm() > m_p.redline;

    // Spark and fuel maps. Advance climbs with speed and, because a light
    // charge burns slowly, with vacuum; mixture is stoichiometric on part
    // throttle and enriched at full load to hold peak pressure down.
    const double loadFrac = std::clamp(m_plenumPressure / Thermo::pAmb, 0.0, 1.0);
    const double speedFrac = std::clamp((rpm() - 1000.0) / 4500.0, 0.0, 1.0);
    m_sparkAdvance = m_p.sparkIdle +
                     (m_p.sparkHighSpeed - m_p.sparkIdle) * speedFrac +
                     m_p.sparkPartLoad * (1.0 - loadFrac);
    m_afr = m_p.afrCruise + (m_p.afrPower - m_p.afrCruise) *
                            std::clamp((loadFrac - 0.55) / 0.45, 0.0, 1.0);
    const double sparkPhase = wrap720(720.0 - m_sparkAdvance);
    const double a  = 0.5 * m_p.stroke;
    const double l  = m_p.rodLength;
    const double boreFactor = std::pow(m_p.bore, -0.2);

    for (std::size_t ci = 0; ci < m_cyl.size(); ++ci) {
        Cylinder& c = m_cyl[ci];
        const double offset    = ci < m_p.phaseOffsets.size() ? m_p.phaseOffsets[ci] : 0.0;
        const double prevPhase = c.phase;
        c.phase = wrap720(m_crankAngle + offset);

        const double Vprev = c.volume;
        c.volume = cylinderVolume(c.phase);
        const double dV = c.volume - Vprev;

        c.intakeLift  = lookupLift(m_intakeLift, c.phase);
        c.exhaustLift = lookupLift(m_exhaustLift, c.phase);
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
        convect(plenumNode(), runNode(), c.intakeFlowRate);

        const double rhoEx = c.exRunnerPressure / (Thermo::R * c.exRunnerTemp);
        accelerate(c.exhaustFlowRate, c.exRunnerPressure, m_collectorPressure,
                   rhoEx, m_p.exhaustRunner);
        convect(exRunNode(), collector(), c.exhaustFlowRate);

        // Refresh the runner states before the valves see them.
        auto refreshRunner = [&] {
            const double y = std::clamp(c.runnerBurned / std::max(c.runnerMass, 1e-12), 0.0, 1.0);
            c.runnerTemp = Thermo::temperature(c.runnerEnergy / std::max(c.runnerMass, 1e-12), y);
            c.runnerPressure = c.runnerMass * Thermo::R * c.runnerTemp / runnerVol;
            c.exRunnerTemp = Thermo::temperature(c.exRunnerEnergy / std::max(c.exRunnerMass, 1e-12), 1.0);
            c.exRunnerPressure = c.exRunnerMass * Thermo::R * c.exRunnerTemp / exRunnerVol;
        };
        refreshRunner();

        // ---- Valves ----------------------------------------------------------
        const double intoCyl = exchange(runNode(), cylNode(), aInt);
        const double outCyl  = -exchange(exRunNode(), cylNode(), aExh);
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

        // ---- Combustion ------------------------------------------------------
        const bool crossedSpark = prevPhase != c.phase && inWindow(sparkPhase, prevPhase, c.phase);
        if (crossedSpark) {
            // Residual dilution slows the flame and, past roughly a third of
            // the charge, puts it out entirely.
            // Dilution does not switch combustion off, it degrades it: the
            // flame gets slower and less complete until it fails to propagate.
            const double dilutionEff = std::clamp(1.0 - 1.6 * std::max(0.0, yCyl - 0.25), 0.0, 1.0);
            const bool misfire = yCyl > m_p.misfireLimit;
            if (!fuelCut && !misfire) {
                c.burning       = true;
                c.burnFraction  = 0.0;
                c.chargeAtSpark = c.mass;
                c.burnStart     = wrap720(sparkPhase + m_p.ignitionDelay);
                // Burn duration in crank angle grows slowly with speed and
                // sharply with dilution.
                const double speedTerm = 0.80 + 0.20 * std::min(2.5, rpm() / 3000.0);
                c.burnSpan = m_p.burnDuration * speedTerm * (1.0 + 1.6 * yCyl);
                // Only the unburned part of the charge carries fuel.
                c.fuelEnergy = (c.mass * (1.0 - yCyl) / m_afr) * m_p.fuelLHV *
                               m_p.combustionEff * dilutionEff;
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
        const double dQwall = hc * area * (c.temperature - m_p.wallTemp) * dt;

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
        gasTorque += (c.pressure - Thermo::pAmb) * dVolumeDTheta(c.phase);

        // Reciprocating inertia: the piston and small end have to be stopped
        // and restarted twice a revolution. It integrates to zero over a cycle
        // but is what gives the crank its within-cycle speed ripple.
        {
            const double th = c.phase * kDegRad;
            const double dx  = -a * std::sin(th) - (a * a / (2.0 * l)) * std::sin(2.0 * th);
            const double ddx = -a * std::cos(th) - (a * a / l) * std::cos(2.0 * th);
            inertiaTorque -= m_p.recipMass * m_omega * m_omega * dx * ddx;
        }

        peakForFriction = std::max(peakForFriction, c.pressure);

        // ---- Per-cycle bookkeeping at intake valve closing --------------------
        if (prevPhase != c.phase && inWindow(m_p.intake.closeDeg, prevPhase, c.phase)) {
            c.refPressure     = c.pressure;
            c.refVolume       = c.volume;
            c.refTemp         = c.temperature;
            c.motoredPressure = c.pressure;
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
    {
        Node amb = ambientNode(1.0);
        exchange(collector(), amb, m_p.outletArea);
    }
    m_collectorMass = std::max(m_collectorMass, 1e-9);
    m_collectorTemp = Thermo::temperature(m_collectorEnergy / m_collectorMass, 1.0);
    m_collectorPressure = m_collectorMass * Thermo::R * m_collectorTemp / m_p.collectorVolume;

    // ---- Crank dynamics --------------------------------------------------------
    // Chen-Flynn: friction rises with peak cylinder pressure and with the
    // square of piston speed, which is what rolls the torque curve off at both
    // ends instead of leaving it flat.
    // Friction responds to the cycle's peak pressure, not to the pressure at
    // this instant, so the peak is held and bled off slowly.
    m_pmaxHold = std::max(peakForFriction, m_pmaxHold * (1.0 - 3.0 * dt));
    const double pmaxBar = m_pmaxHold * 1e-5;
    const double fmepBar = m_p.cfA + m_p.cfB * pmaxBar +
                           m_p.cfC * meanPistonSpeed +
                           m_p.cfD * meanPistonSpeed * meanPistonSpeed;
    m_fmep = fmepBar * 1e5;
    const double totalDisp = m_displacement * m_p.cylinders;
    // FMEP is a mean pressure per swept volume per cycle; a four-stroke turns
    // 4*pi radians per cycle.
    const double frictionTorque = m_fmep * totalDisp / (4.0 * kPi) + m_p.accessoryTorque;

    const double dir = m_omega > 0.0 ? 1.0 : 0.0;   // no braking torque at rest
    double torque = gasTorque + inertiaTorque - frictionTorque * dir;

    if (m_starter && rpm() < 320.0) torque += m_p.starterTorque;

    // The load is whatever the clutch is holding back: in neutral that is
    // nothing, in gear it is the car.
    torque += m_drive.step(dt, m_omega);

    m_omega += torque / m_p.inertia * dt;
    if (m_omega < 0.0) m_omega = 0.0;   // the crank never spins backwards here

    m_netTorque = torque;
    const double brakeTorque = gasTorque - frictionTorque * dir;
    m_torqueFilt += (brakeTorque - m_torqueFilt) * std::min(1.0, 6.0 * dt);
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
    s.exhaustTemp      = static_cast<float>(m_collectorTemp);
    s.backPressure     = static_cast<float>(m_collectorPressure * 0.001);
    s.volumetricEff    = static_cast<float>(m_volEff);
    s.residualFraction = static_cast<float>(m_residual);
    s.sparkAdvance     = static_cast<float>(m_sparkAdvance);
    s.afr              = static_cast<float>(m_afr);
    s.idleValve        = static_cast<float>(m_idleCmd);
    s.cylinderCount    = static_cast<int>(m_cyl.size());
    for (std::size_t i = 0; i < m_cyl.size() && i < s.cyl.size(); ++i) {
        const Cylinder& c = m_cyl[i];
        CylinderView& v = s.cyl[i];
        v.phase          = static_cast<float>(c.phase);
        v.pressure       = static_cast<float>(c.pressure * 1e-5);
        v.temperature    = static_cast<float>(c.temperature);
        v.intakeLift     = static_cast<float>(c.intakeLift / m_p.intake.maxLift);
        v.exhaustLift    = static_cast<float>(c.exhaustLift / m_p.exhaust.maxLift);
        v.burnFraction   = static_cast<float>(c.burning ? c.burnFraction : 0.0);
        v.burnedGas      = static_cast<float>(c.burnedMass / std::max(c.mass, 1e-12));
        v.runnerPressure = static_cast<float>(c.runnerPressure * 0.001);
        v.exRunnerPressure = static_cast<float>(c.exRunnerPressure * 0.001);
    }
    return s;
}

} // namespace sim
