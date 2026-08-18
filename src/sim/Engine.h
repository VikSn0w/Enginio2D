#pragma once
#include "sim/Drivetrain.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

namespace sim {

// Everything downstream sizes its arrays off this, so a W16 fits.
inline constexpr std::size_t kMaxCylinders = 16;

// ---------------------------------------------------------------------------
// Working fluid.
//
// The charge is a two-component mixture of unburned air/fuel and burned
// products, each with a temperature-dependent specific heat. A constant-gamma
// model overestimates peak pressure badly - gamma really falls from ~1.40 cold
// to ~1.22 at flame temperature - so internal energy is integrated as
//
//     u(T) = a (T - T0) + b/2 (T - T0)^2 ,  cv(T) = a + b (T - T0)
//
// which inverts to a temperature in closed form.
// ---------------------------------------------------------------------------
struct Thermo {
    static constexpr double R    = 287.0;    // J/(kg K)
    static constexpr double T0   = 300.0;
    static constexpr double pAmb = 101325.0; // Pa
    static constexpr double tAmb = 293.0;    // K

    // cv coefficients: unburned (air-like) and burned products.
    static constexpr double cvU0 = 718.0, cvUk = 0.105;
    static constexpr double cvB0 = 880.0, cvBk = 0.190;

    // Used only by the compressible-flow function, which is barely sensitive
    // to it.
    static constexpr double gammaFlow = 1.35;

    static double cvA(double y) { return cvU0 + (cvB0 - cvU0) * y; }
    static double cvB(double y) { return cvUk + (cvBk - cvUk) * y; }

    // Specific internal energy at T for a mixture with burned fraction y.
    static double energy(double T, double y) {
        const double x = T - T0;
        return cvA(y) * x + 0.5 * cvB(y) * x * x;
    }
    // Inverse of energy(): temperature from specific internal energy.
    static double temperature(double e, double y);
    static double enthalpy(double T, double y) { return energy(T, y) + R * T; }
};

// Cycle angle convention used everywhere below: 0 deg = TDC firing,
// 180 = BDC, 360 = TDC overlap, 540 = BDC, 720 = back to TDC firing.
struct ValveTiming {
    double openDeg;   // cycle angle at which the valve cracks open
    double closeDeg;  // cycle angle at which it seats again
    double maxLift;   // m
    double diameter;  // m, valve head diameter (sets the curtain area)
    int    count;     // valves of this type per cylinder
    double cdScale;   // multiplies the lift-dependent discharge coefficient
    // Shape of the flank. A raised cosine has too pointed a nose to pass for a
    // real cam; an exponent below one flattens the nose and steepens the
    // flanks, which is what a fast road or race profile looks like. How far it
    // can be pushed is set by the valvetrain: a flat tappet cannot follow what
    // a roller or a desmo mechanism can.
    double profileExp = 0.82;
};

// Lift in metres at a cycle angle, straight from the profile. Both the engine's
// baked table and the timing diagram in the UI go through this, so the drawing
// can never disagree with the simulation.
double valveLiftAt(const ValveTiming& v, double phaseDeg);

// One gas column between a plenum and a port: an inertance (the mass of gas
// that has to be accelerated) plus a small volume. With the cylinder these
// form the resonator responsible for ram charging and exhaust scavenging.
struct RunnerSpec {
    double length;    // m
    double area;      // m^2, cross-section
    double zeta;      // form-loss coefficient (bends, plenum entry)
    double viscous;   // Pa per (kg/s), wall friction and thermal damping
};

// ---------------------------------------------------------------------------
// Fuel. What actually distinguishes one fuel from another in a cycle
// simulation is: how much energy a kilogram carries, how much air that
// kilogram needs, how much heat it steals from the charge as it evaporates,
// how fast the flame crosses the chamber, and how hard it is to detonate.
// ---------------------------------------------------------------------------
struct FuelSpec {
    double lhv        = 44.0e6;   // J/kg
    double stoichAfr  = 14.7;     // kg air per kg fuel
    double vaporHeat  = 350.0e3;  // J/kg, latent heat - this is the charge cooling
    double portEvap   = 0.55;     // fraction that evaporates in the port, not the bore
    double octane     = 95.0;     // RON, feeds the knock induction time
    double flameSpeed = 1.0;      // multiplies burn rate (methanol burns fast, LPG slow)
    bool   compressionIgnition = false;
    double autoIgnitionK = 850.0; // K, charge temperature a diesel needs to light
    double smokeAfr   = 18.0;     // CI only: richest mixture before it just makes soot
};

// Oil. Viscosity sets the hydrodynamic part of engine friction and the oil
// pressure the pump can build; film strength is what stops the boundary term
// climbing when the oil is hot and thin.
struct OilSpec {
    double viscosity40  = 1.00;  // relative to a 5W-30 at 40 C
    double viscosity100 = 1.00;  // ... and at 100 C
    double filmStrength = 1.00;
};

struct EngineParams {
    // Geometry (defaults: ~2.0 L inline-four)
    double bore             = 0.086;   // m
    double stroke           = 0.086;   // m
    double rodLength        = 0.145;   // m
    double compressionRatio = 10.5;
    int    cylinders        = 4;
    int    banks            = 1;       // 1 for an inline, 2 for a V or flat, 4 for a W
    double bankAngle        = 0.0;     // deg, included angle
    // Crank angle at which each cylinder sits, relative to cylinder 1.
    // Inline-four, firing order 1-3-4-2.
    std::vector<double> phaseOffsets{0.0, 540.0, 180.0, 360.0};
    // Which bank each cylinder belongs to. The physics runs one collector, but
    // the exhaust synthesis needs this: a crossplane V8 only burbles because
    // each bank's pulse train is uneven on its own.
    std::vector<int> cylinderBank{0, 0, 0, 0};

    ValveTiming intake { 350.0, 580.0, 0.0100, 0.032, 2, 1.00, 0.82 };
    ValveTiming exhaust{ 130.0, 375.0, 0.0092, 0.028, 2, 0.92, 0.82 };

    // Valvetrain. Above the float speed the springs can no longer keep the
    // follower on the cam, and lift collapses - which is the real reason a
    // pushrod engine stops making power where a pneumatic-valve one is still
    // pulling.
    double valveFloatRpm  = 8600.0;
    double valvetrainDrag = 1.0;    // multiplies the speed-dependent friction terms
    // Cam phaser: intake centreline is advanced at low speed for cylinder
    // filling and returned at high speed for overlap, as a real phaser does.
    double vvtRange   = 0.0;        // deg of authority, 0 disables
    double vvtLowRpm  = 2200.0;
    double vvtHighRpm = 6200.0;

    // Combustion. Spark advance and fuelling follow small maps rather than
    // being fixed, the way an ECU would run them.
    double sparkIdle      = 14.0;   // deg BTDC at idle
    double sparkHighSpeed = 34.0;   // deg BTDC at high speed, full load
    double sparkPartLoad  = 12.0;   // extra advance at light load
    double burnDuration   = 46.0;   // deg at 3000 rpm, scaled with speed
    double ignitionDelay  = 8.0;    // deg between spark and first heat release
    double lambdaCruise   = 1.00;   // commanded lambda at light load
    double lambdaPower    = 0.87;   // ... and at full load
    FuelSpec fuel;
    double combustionEff  = 0.94;
    double wallTemp       = 450.0;  // K
    // Single-zone models always under-predict wall heat loss (they see none of
    // the crevices, the quench layer or the blowby), so Woschni gets the
    // calibration multiplier such models are normally used with.
    double heatTransferScale = 1.8;
    double misfireLimit   = 0.60;   // burned fraction at which the flame dies

    // Knock. The unburned charge ahead of the flame is compressed and heated by
    // the flame itself; if it reaches its induction time before the flame gets
    // there, it goes off on its own.
    bool   knockControl   = true;   // ECU pulls timing when it hears knock
    double knockRetardMax = 12.0;   // deg

    // Mechanics
    double inertia         = 0.18;  // kg m^2, crank + flywheel + clutch
    double recipMass       = 0.55;  // kg per cylinder, piston + small end
    double starterTorque   = 110.0; // N m
    // Chen-Flynn friction: fmep[bar] = A + B*Pmax[bar] + C*Sp + D*Sp^2
    double cfA = 0.55, cfB = 0.008, cfC = 0.020, cfD = 0.0016;
    double accessoryTorque = 8.0;   // N m, alternator/water pump/oil pump drag
    OilSpec oil;
    double oilTempTarget   = 368.0; // K, oil temperature once warm
    double oilStartTemp    = 293.0; // K, temperature at key-on

    // Induction
    double plenumVolume   = 0.0022; // m^3, manifold behind the throttle
    double throttleBore   = 0.052;  // m
    // Idle air control: a bypass around the throttle plate, trimmed by a PI
    // loop to hold the target idle speed, as an idle valve on a real engine
    // does. Without it a closed throttle either stalls or hunts.
    double idleBypassMin  = 0.0012; // fraction of throttle area, valve shut
    double idleBypassMax  = 0.0220; // fraction of throttle area, valve open
    double idleTargetRpm  = 850.0;
    RunnerSpec intakeRunner { 0.30, 0.00110, 1.1, 9.0e4 };

    // Forced induction. 0 none, 1 turbo, 2 roots blower, 3 centrifugal blower.
    int    charger        = 0;
    double boostTarget    = 0.0;    // bar gauge at the wastegate/bypass setting
    double spoolRpm       = 3000.0; // where a belt blower or turbo is on song
    double turboLag       = 0.9;    // s, first-order spool time
    double interCoolerEff = 0.70;   // 0 = no intercooler, 1 = charge back to ambient
    double compressorEff  = 0.70;   // isentropic
    double blowerDriveEff = 0.85;   // belt + rotor losses on a supercharger
    double turbineRestrict= 1.00;   // multiplies exhaust outlet area (turbine backpressure)

    // Exhaust
    RunnerSpec exhaustRunner{ 0.42, 0.00090, 1.5, 7.0e4 };
    double collectorVolume = 0.0016; // m^3
    double outletArea      = 0.0022; // m^2, collector to the rest of the system

    // Acoustic properties of the exhaust system. These feed the pipe model in
    // the synthesiser rather than the gas dynamics, which is why they sit here
    // and not in RunnerSpec: two systems can flow the same and sound nothing
    // alike.
    double primaryMismatch = 0.04;  // spread of primary lengths, 0 = equal length
    double tailLength      = 0.72;  // m
    double mufflerDamping  = 0.34;  // one-pole coefficient, higher is brighter
    double exhaustLoudness = 1.00;

    double redline = 7600.0;        // rpm, fuel cut above this
};

struct Cylinder {
    double phase        = 0.0;   // cycle angle, deg
    double volume       = 0.0;   // m^3
    double mass         = 0.0;   // kg of trapped charge
    double burnedMass   = 0.0;   // kg of that which is combustion products
    double energy       = 0.0;   // J, internal energy of the charge
    double temperature  = Thermo::tAmb;
    double pressure     = Thermo::pAmb;

    double burnFraction  = 0.0;  // Wiebe mass fraction burned
    double fuelEnergy    = 0.0;  // J for this combustion event
    double fuelMass      = 0.0;  // kg burnt this event, for the fuel-flow readout
    double burnStart     = 0.0;  // cycle angle of first heat release
    double burnSpan      = 50.0; // deg, this event's duration
    double chargeAtSpark = 0.0;  // kg, for tracking product formation
    bool   burning       = false;
    bool   armed         = true; // spark not yet fired this cycle
    double knockIntegral = 0.0;  // Livengood-Wu; 1.0 means the end gas lights
    double knockShock    = 0.0;  // decaying marker of a knock event, for audio

    // Intake runner: momentum state plus a small volume at the port.
    double intakeFlowRate  = 0.0; // kg/s in the runner column
    double runnerMass      = 0.0;
    double runnerEnergy    = 0.0;
    double runnerBurned    = 0.0; // reversion carries products back up the port
    double runnerPressure  = Thermo::pAmb;
    double runnerTemp      = Thermo::tAmb;

    // Exhaust runner.
    double exhaustFlowRate  = 0.0; // kg/s in the runner column
    double exRunnerMass     = 0.0;
    double exRunnerEnergy   = 0.0;
    double exRunnerBurned   = 0.0;
    double exRunnerPressure = Thermo::pAmb;
    double exRunnerTemp     = Thermo::tAmb;

    // Reference state captured at intake valve closing, for the Woschni
    // correlation's motored-pressure term and for the unburned-gas temperature
    // the knock model needs.
    double refPressure     = Thermo::pAmb;
    double refVolume       = 1.0;
    double refTemp         = Thermo::tAmb;
    double motoredPressure = 0.0;

    double valveFlowIntake  = 0.0; // kg/s through the intake valve, into cylinder
    double valveFlowExhaust = 0.0; // kg/s through the exhaust valve, out
    double intakeLift       = 0.0; // m
    double exhaustLift      = 0.0; // m
    double freshTrapped     = 0.0; // kg of fresh charge taken in this cycle
    double airTrapped       = 0.0; // kg of air at IVC, for the diesel's fuelling
};

// Per-cylinder data for the views.
struct CylinderView {
    float phase          = 0.0f;  // cycle angle, deg
    float pressure       = 0.0f;  // bar
    float temperature    = 0.0f;  // K
    float intakeLift     = 0.0f;  // 0..1
    float exhaustLift    = 0.0f;  // 0..1
    float burnFraction   = 0.0f;
    float burnedGas      = 0.0f;  // mass fraction of the charge that is products
    float runnerPressure = 0.0f;  // kPa, intake runner
    float exRunnerPressure = 0.0f;// kPa, exhaust runner
    float knock          = 0.0f;  // 0..1, this cylinder's last knock event
    int   bank           = 0;
};

// A consistent copy of the state for the render thread.
struct Snapshot {
    float rpm              = 0.0f;
    float manifoldPressure = 0.0f; // kPa absolute
    float torque           = 0.0f; // N m, brake torque (gas minus friction)
    float power            = 0.0f; // kW
    float fmep             = 0.0f; // bar
    float throttle         = 0.0f;
    float crankAngle       = 0.0f; // deg, 0..720
    float peakPressure     = 0.0f; // bar, last cycle of cylinder 1
    float exhaustTemp      = 0.0f; // K, collector
    float backPressure     = 0.0f; // kPa, collector
    float volumetricEff    = 0.0f; // 0..1+, cylinder 1
    float residualFraction = 0.0f; // burned mass left at IVC, cylinder 1
    float sparkAdvance     = 0.0f; // deg BTDC
    float afr              = 0.0f;
    float lambda           = 0.0f;
    float idleValve        = 0.0f; // 0..1
    float boost            = 0.0f; // kPa gauge upstream of the throttle
    float chargeTemp       = 0.0f; // K, after the intercooler
    float knock            = 0.0f; // 0..1, smoothed
    float knockRetard      = 0.0f; // deg the ECU has pulled
    float oilTemp          = 0.0f; // K
    float oilPressure      = 0.0f; // bar
    float camAdvance       = 0.0f; // deg of intake phaser authority in use
    float valveFloat       = 0.0f; // 0 = following the cam, 1 = fully floating
    float fuelFlow         = 0.0f; // kg/h
    float bsfc             = 0.0f; // g/kWh
    int   gear             = 0;    // 0 = neutral
    int   gearCount        = 6;
    float speedKph         = 0.0f;
    float clutchLock       = 0.0f; // 0..1
    float clutchSlip       = 0.0f; // rpm across the clutch
    float wheelTorque      = 0.0f; // N m
    float brake            = 0.0f;
    int   cylinderCount    = 0;
    std::array<CylinderView, kMaxCylinders> cyl{};
};

class Engine {
public:
    explicit Engine(EngineParams params);

    // Advance by dt seconds. Sub-steps internally so no sub-step covers more
    // than ~0.25 deg of crank rotation, which keeps the explicit integration of
    // the valve and runner flows stable at high rpm.
    void advance(double dt);

    // Swap in a new specification without stopping. Geometry, cams and gas
    // volumes are rebuilt; crank speed and angle survive, so the engine keeps
    // running through the change instead of restarting.
    void reconfigure(const EngineParams& params);

    void setThrottle(double t);
    void setStarter(bool on)    { m_starter = on; }
    void setIgnition(bool on)   { m_ignition = on; }
    bool ignition() const       { return m_ignition; }

    // Dyno mode: the crank is held at a fixed speed by an ideal brake, and the
    // torque it takes to do that is the measurement.
    void setSpeedHold(bool on, double rpm);
    double heldTorque() const { return m_torqueFilt; }
    // Skip the warm-up. A dyno pull is done on a hot engine, never a cold one.
    void warmUp() { m_oilTemp = m_p.oilTempTarget; }

    double rpm() const { return m_omega * 60.0 / (2.0 * 3.14159265358979323846); }

    // Acoustic sources, in Pa relative to ambient: the exhaust collector and
    // the intake plenum. The pipe models downstream turn these into sound.
    double collectorGauge() const { return m_collectorPressure - Thermo::pAmb; }
    double intakeGauge()    const { return m_plenumPressure - Thermo::pAmb; }

    Drivetrain&       drivetrain()       { return m_drive; }
    const Drivetrain& drivetrain() const { return m_drive; }

    Snapshot snapshot() const;
    const EngineParams& params() const { return m_p; }
    // Read-only access for the audio layer, which needs per-cylinder detail
    // that the render snapshot does not carry.
    const std::vector<Cylinder>& cylinders() const { return m_cyl; }

    // Gauge pressure (Pa) in one cylinder's exhaust runner: the acoustic
    // source for that cylinder's primary pipe.
    double exhaustRunnerGauge(std::size_t i) const {
        return i < m_cyl.size() ? m_cyl[i].exRunnerPressure - Thermo::pAmb : 0.0;
    }
    int cylinderBank(std::size_t i) const {
        return i < m_p.cylinderBank.size() ? m_p.cylinderBank[i] : 0;
    }

    // Cylinder-1 pressure in bar, binned by crank angle, for the P-theta plot.
    // Written by the audio thread, read by the render thread.
    static constexpr std::size_t kTraceBins = 720;
    float traceAt(std::size_t bin) const { return m_trace[bin].load(std::memory_order_relaxed); }

    // Valve lift in metres at a cycle angle, for the timing diagram.
    double intakeLiftAt(double phaseDeg) const  { return lookupLift(m_intakeLift, phaseDeg); }
    double exhaustLiftAt(double phaseDeg) const { return lookupLift(m_exhaustLift, phaseDeg); }

private:
    void   step(double dt);
    void   rebuild(bool preserveMotion);
    double cylinderVolume(double phaseDeg) const;
    double dVolumeDTheta(double phaseDeg) const;  // m^3 per radian
    double portArea(const ValveTiming& v, double lift) const;
    static double orificeFlow(double pUp, double tUp, double pDown, double area);
    void   updateCharger(double dt, double throttleFlow);

    // Cam profiles are baked into a 0.1 deg table at construction: faster than
    // evaluating the profile, and the shape can then be arbitrary.
    static constexpr std::size_t kLiftBins = 7200;
    void buildLiftTable(std::vector<double>& table, const ValveTiming& v) const;
    static double lookupLift(const std::vector<double>& table, double phaseDeg);

    EngineParams m_p;
    std::vector<Cylinder> m_cyl;
    std::vector<double> m_intakeLift, m_exhaustLift;

    double m_crankAngle = 0.0;    // deg, 0..720
    double m_omega      = 0.0;    // rad/s
    double m_throttle   = 0.0;
    bool   m_starter    = false;
    bool   m_ignition   = true;
    bool   m_speedHold  = false;
    double m_holdOmega  = 0.0;

    // Intake plenum behind the throttle plate.
    double m_plenumMass     = 0.0;
    double m_plenumEnergy   = 0.0;
    double m_plenumBurned   = 0.0;   // reversion pushes products back up here
    double m_plenumTemp     = Thermo::tAmb;
    double m_plenumPressure = Thermo::pAmb;

    // Upstream of the throttle: ambient on a naturally aspirated engine, the
    // compressor discharge on a blown one.
    double m_boostPressure = Thermo::pAmb;
    double m_boostTemp     = Thermo::tAmb;
    double m_spool         = 0.0;    // 0..1
    double m_blowerTorque  = 0.0;    // N m taken off the crank by a belt blower
    double m_throttleFlow  = 0.0;    // kg/s, filtered

    // Exhaust collector: every runner dumps into it, it vents to atmosphere.
    double m_collectorMass     = 0.0;
    double m_collectorEnergy   = 0.0;
    double m_collectorBurned   = 0.0;
    double m_collectorTemp     = Thermo::tAmb;
    double m_collectorPressure = Thermo::pAmb;

    double m_pistonArea   = 0.0;
    double m_clearance    = 0.0;
    double m_displacement = 0.0;

    double m_netTorque    = 0.0;
    double m_torqueFilt   = 0.0;   // smoothed brake torque, for the gauges
    double m_fmep         = 0.0;   // Pa
    double m_peakPressure = 0.0;
    double m_peakAccum    = 0.0;
    double m_volEff       = 0.0;
    double m_residual     = 0.0;
    double m_rpmFilt      = 0.0;   // filtered crank speed, for the idle loop
    double m_rpmRate      = 0.0;   // rpm/s, damping term for the idle loop
    double m_pmaxHold     = Thermo::pAmb;  // decaying peak-pressure hold
    double m_idleCmd      = 0.5;   // idle valve position, 0..1
    double m_idleIntegral = 0.5;
    double m_sparkAdvance = 20.0;  // deg BTDC, current map output
    double m_afr          = 14.0;
    double m_lambda       = 1.0;
    double m_knockLevel   = 0.0;   // smoothed knock intensity, 0..1
    double m_knockRetard  = 0.0;   // deg the ECU is holding back
    double m_camAdvance   = 0.0;   // deg, current phaser position
    double m_floatLoss    = 0.0;   // 0 = on the cam, 1 = fully floating
    double m_oilTemp      = Thermo::tAmb;
    double m_oilPressure  = 0.0;   // Pa gauge
    double m_fuelRate     = 0.0;   // kg/s, filtered
    double m_fuelAccum    = 0.0;   // kg burnt since the last averaging window
    double m_fuelWindow   = 0.0;   // s

    Drivetrain m_drive;

    std::array<std::atomic<float>, kTraceBins> m_trace{};
};

} // namespace sim
