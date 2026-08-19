#include "sim/EngineDesign.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace sim {
namespace {

constexpr double kPi = 3.14159265358979323846;

const char* kLayoutN[]     = {"Inline", "V", "Flat / boxer", "W"};
const char* kCrankN[]      = {"Even fire", "Crossplane", "Odd fire (shared pin)"};
const char* kValvetrainN[] = {"OHV pushrod", "SOHC", "DOHC", "Desmodromic", "Pneumatic"};
const char* kMetalN[]      = {"Steel", "Sodium filled", "Titanium", "Ceramic"};
const char* kCamN[]        = {"Stock", "Street", "Sport", "Race"};
const char* kPortN[]       = {"As cast", "Hand ported", "CNC ported"};
const char* kFuelN[]       = {"Petrol 91 RON", "Petrol 95 RON", "Petrol 98 RON",
                              "Race fuel 110", "E85", "Ethanol E100", "Methanol",
                              "LPG", "CNG", "Diesel", "Nitromethane", "Hydrogen"};
const char* kOilN[]        = {"0W-20", "5W-30", "10W-40", "15W-50", "20W-60",
                              "10W-60 race"};
const char* kChargerN[]    = {"Naturally aspirated", "Turbocharger",
                              "Roots blower", "Centrifugal blower"};
const char* kHeaderN[]     = {"Log manifold", "4-into-1 header", "Tri-Y header",
                              "Per-bank collectors", "Open individual pipes"};
const char* kMufflerN[]    = {"Straight through", "Chambered", "Absorption", "Quiet OEM"};
const char* kThemeN[]      = {"Graphite", "Midnight", "Slate", "Blueprint"};

// Fuel properties. What separates one fuel from another in a cycle simulation
// is energy per kilogram, the air it needs, the heat it takes out of the charge
// as it evaporates, how fast its flame moves, and how hard it is to detonate.
const FuelSpec kFuelSpec[] = {
    // lhv       afr    vaporHeat  evap  octane speed  CI     autoIgn smoke
    {  44.0e6, 14.70,   350.0e3, 0.55,  91.0, 1.00, false,  850.0, 18.0},
    {  43.6e6, 14.70,   350.0e3, 0.55,  95.0, 1.00, false,  850.0, 18.0},
    {  43.4e6, 14.60,   350.0e3, 0.55,  98.0, 1.01, false,  850.0, 18.0},
    {  43.0e6, 14.50,   340.0e3, 0.55, 110.0, 1.03, false,  850.0, 18.0},
    {  29.2e6,  9.80,   760.0e3, 0.60, 105.0, 1.10, false,  850.0, 18.0},
    {  26.8e6,  9.00,   900.0e3, 0.62, 109.0, 1.12, false,  850.0, 18.0},
    {  19.9e6,  6.45,  1100.0e3, 0.65, 109.0, 1.25, false,  850.0, 18.0},
    {  46.4e6, 15.70,   425.0e3, 0.05, 105.0, 0.95, false,  850.0, 18.0},
    {  47.0e6, 17.20,     0.0e3, 0.00, 120.0, 0.85, false,  850.0, 18.0},
    {  42.5e6, 14.50,   250.0e3, 0.00,  25.0, 1.00,  true,  800.0, 18.5},
    {  11.3e6,  1.70,   560.0e3, 0.60, 100.0, 1.45, false,  850.0, 18.0},
    { 120.0e6, 34.30,     0.0e3, 0.00, 130.0, 2.40, false,  850.0, 18.0},
};

// Viscosity relative to a 5W-30 at 100 C, quoted at 40 C and at 100 C. Film
// strength is what keeps the boundary friction term down once the oil is hot.
const OilSpec kOilSpec[] = {
    { 3.6, 0.78, 0.90}, { 5.2, 1.00, 1.00}, { 7.4, 1.32, 1.08},
    {10.0, 1.70, 1.15}, {13.0, 2.10, 1.20}, { 9.0, 2.05, 1.35},
};

int idx(int v, int count) { return std::clamp(v, 0, count - 1); }

double wrap720(double a) {
    a = std::fmod(a, 720.0);
    return a < 0.0 ? a + 720.0 : a;
}

// Classic firing orders. Every one of these exists to keep consecutive events
// away from adjacent main bearings; taking them from a table beats deriving
// them, because the choice was never purely geometric in the first place.
const std::vector<int>& inlineOrder(int n) {
    static const std::vector<int> table[9] = {
        {}, {1}, {1, 2}, {1, 2, 3}, {1, 3, 4, 2}, {1, 2, 4, 5, 3},
        {1, 5, 3, 6, 2, 4}, {1, 2, 4, 6, 7, 5, 3}, {1, 6, 2, 5, 8, 3, 7, 4}};
    static const std::vector<int> fallback;
    if (n >= 1 && n <= 8) return table[n];
    return fallback;
}

int banksFor(int layout, int cylinders) {
    switch (static_cast<Layout>(layout)) {
        case Layout::Vee:
        case Layout::Flat: return cylinders >= 2 ? 2 : 1;
        case Layout::W:    return cylinders >= 8 ? 4 : 2;
        default:           return 1;
    }
}

double liftCapFor(int valvetrain) {
    static const double cap[] = {0.26, 0.30, 0.34, 0.40, 0.42};
    return cap[idx(valvetrain, 5)];
}

double profileFloorFor(int valvetrain) {
    static const double f[] = {0.84, 0.79, 0.72, 0.62, 0.55};
    return f[idx(valvetrain, 5)];
}

double floatRpmFor(const EngineDesign& d, double intakeValveDia) {
    // What limits engine speed is how fast the valvetrain can be thrown up and
    // down against its springs, and for a given level of technology that scales
    // with the whole reciprocating assembly - so it tracks mean piston speed.
    // This is why a short-stroke 600 four revs to 15000 and a long-stroke
    // pushrod twin of the same technology does not get near it.
    static const double speed[] = {21.0, 22.5, 25.0, 30.0, 36.0};   // m/s at float
    static const double metal[] = {1.00, 1.04, 1.12, 1.20};
    // A race grind comes with the springs to match; a stock one does not.
    static const double grind[] = {0.97, 1.00, 1.03, 1.06};
    const double frac = std::max(0.05, intakeValveDia / std::max(d.bore * 1e-3, 1e-4));
    const double size = std::clamp(std::pow(0.37 / frac, 0.35), 0.75, 1.25);
    const double limit = speed[idx(d.valvetrain, 5)] * metal[idx(d.valveMaterial, 4)] *
                         grind[idx(d.camProfile, 4)] * size;
    return limit * 60.0 / (2.0 * std::max(d.stroke * 1e-3, 0.01));
}

// ---------------------------------------------------------------------------
// Crank arrangement: which cylinder fires when, and which bank it sits on.
// ---------------------------------------------------------------------------
void computeFiring(const EngineDesign& d, std::vector<double>& phases,
                   std::vector<int>& bank, std::vector<int>& order) {
    const int n = std::clamp(d.cylinders, 1, static_cast<int>(kMaxCylinders));
    const int b = banksFor(d.layout, n);
    phases.assign(static_cast<std::size_t>(n), 0.0);
    bank.assign(static_cast<std::size_t>(n), 0);
    order.clear();
    // Cylinders alternate between banks: the 1-3-5-7 / 2-4-6-8 convention.
    for (int i = 0; i < n; ++i) bank[static_cast<std::size_t>(i)] = b > 1 ? i % b : 0;

    const bool vee = b > 1;
    const auto crank = static_cast<CrankType>(idx(d.crankType, 3));
    bool done = false;

    // A crossplane V8 is even-fire overall but not within either bank, and that
    // is the whole sound of it: each bank sees 90-180-270-180 rather than a
    // steady 180.
    if (crank == CrankType::Crossplane && vee && n == 8) {
        static const int cp[] = {1, 8, 4, 3, 6, 5, 7, 2};
        for (int k = 0; k < 8; ++k)
            phases[static_cast<std::size_t>(cp[k] - 1)] = k * 90.0;
        done = true;
    }

    // Shared crankpins: the second bank fires its included angle after the
    // first. A 90 degree V6's 90/150 limp and a big twin's gallop are the same
    // mechanism.
    if (!done && crank == CrankType::OddFire && vee && n >= 2) {
        if (n == 2) {
            // A big twin fires 360 plus the bank angle apart, not the bank
            // angle: the rear cylinder's compression stroke lands a revolution
            // later than the naive reading suggests.
            phases[0] = 0.0;
            phases[1] = wrap720(360.0 + d.bankAngle);
        } else {
            const int perBank = n / 2;
            const double interval = 720.0 / perBank;
            const std::vector<int>& sub = inlineOrder(perBank);
            for (int k = 0; k < perBank; ++k) {
                const int member = (k < static_cast<int>(sub.size())
                                    ? sub[static_cast<std::size_t>(k)] : k + 1) - 1;
                const std::size_t a = static_cast<std::size_t>(member) * 2;
                if (a + 1 >= phases.size()) continue;
                phases[a]     = k * interval;
                phases[a + 1] = wrap720(k * interval + d.bankAngle);
            }
        }
        done = true;
    }

    if (!done) {
        // Even fire. Within each bank the cylinders follow that bank's own
        // inline order; the banks are then interleaved so consecutive events
        // alternate sides.
        std::vector<std::vector<int>> seq(static_cast<std::size_t>(b));
        for (int bi = 0; bi < b; ++bi) {
            std::vector<int> members;
            for (int i = 0; i < n; ++i)
                if (bank[static_cast<std::size_t>(i)] == bi) members.push_back(i + 1);
            const std::vector<int>& sub = inlineOrder(static_cast<int>(members.size()));
            for (std::size_t k = 0; k < members.size(); ++k) {
                const std::size_t pick = k < sub.size()
                    ? static_cast<std::size_t>(sub[k] - 1) : k;
                seq[static_cast<std::size_t>(bi)].push_back(
                    members[std::min(pick, members.size() - 1)]);
            }
        }
        std::vector<std::size_t> cursor(static_cast<std::size_t>(b), 0);
        const double interval = 720.0 / n;
        for (int k = 0; k < n; ++k) {
            std::size_t bi = static_cast<std::size_t>(k % b);
            if (cursor[bi] >= seq[bi].size()) {
                // Uneven bank populations (an odd cylinder count on a V): take
                // from whichever bank still has a cylinder waiting.
                bool found = false;
                for (std::size_t t = 0; t < seq.size() && !found; ++t)
                    if (cursor[t] < seq[t].size()) { bi = t; found = true; }
                if (!found) continue;
            }
            phases[static_cast<std::size_t>(seq[bi][cursor[bi]] - 1)] = k * interval;
            ++cursor[bi];
        }
    }

    // Read the firing order back out of the phases so the two can never
    // disagree, whichever branch above produced them.
    std::vector<int> ids(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) ids[static_cast<std::size_t>(i)] = i;
    std::sort(ids.begin(), ids.end(), [&](int x, int y) {
        return phases[static_cast<std::size_t>(x)] < phases[static_cast<std::size_t>(y)];
    });
    for (int id : ids) order.push_back(id + 1);
}

} // namespace

const char* const* layoutNames()     { return kLayoutN; }
const char* const* crankNames()      { return kCrankN; }
const char* const* valvetrainNames() { return kValvetrainN; }
const char* const* valveMetalNames() { return kMetalN; }
const char* const* camNames()        { return kCamN; }
const char* const* portNames()       { return kPortN; }
const char* const* fuelNames()       { return kFuelN; }
const char* const* oilNames()        { return kOilN; }
const char* const* chargerNames()    { return kChargerN; }
const char* const* headerNames()     { return kHeaderN; }
const char* const* mufflerNames()    { return kMufflerN; }
const char* const* themeNames()      { return kThemeN; }

double maxValveFraction(int intakeValves, int exhaustValves) {
    const int total = std::max(2, intakeValves + exhaustValves);
    // A four-valve head really does get its intakes to about 0.40 of the bore,
    // and a two-valve head to about half.
    return std::min(0.52, 1.10 / (1.0 + std::sqrt(static_cast<double>(total - 1))));
}

void clampDesign(EngineDesign& d) {
    d.layout    = idx(d.layout, static_cast<int>(Layout::Count));
    d.crankType = idx(d.crankType, static_cast<int>(CrankType::Count));
    d.cylinders = std::clamp(d.cylinders, 1, static_cast<int>(kMaxCylinders));

    const auto layout = static_cast<Layout>(d.layout);
    if (layout == Layout::Vee || layout == Layout::Flat) {
        if (d.cylinders < 2) d.cylinders = 2;
        if (d.cylinders % 2) d.cylinders += 1;          // a V has two equal banks
    } else if (layout == Layout::W) {
        if (d.cylinders < 8) d.cylinders = 8;
        d.cylinders -= d.cylinders % 4;
    }
    if (layout == Layout::Inline)      d.bankAngle = 0.0;
    else if (layout == Layout::Flat)   d.bankAngle = 180.0;
    else if (layout == Layout::W)      d.bankAngle = std::clamp(d.bankAngle, 8.0, 30.0);
    else                               d.bankAngle = std::clamp(d.bankAngle, 10.0, 120.0);
    if (layout != Layout::Vee && d.crankType == static_cast<int>(CrankType::Crossplane))
        d.crankType = static_cast<int>(CrankType::EvenFire);

    d.bore      = std::clamp(d.bore, 40.0, 145.0);
    d.stroke    = std::clamp(d.stroke, 30.0, 145.0);
    d.rodRatio  = std::clamp(d.rodRatio, 1.20, 2.40);
    d.recipMass = std::clamp(d.recipMass, 0.08, 3.00);
    d.flywheel  = std::clamp(d.flywheel, 0.03, 1.20);

    d.valvetrain    = idx(d.valvetrain, static_cast<int>(Valvetrain::Count));
    d.valveMaterial = idx(d.valveMaterial, static_cast<int>(ValveMetal::Count));
    d.camProfile    = idx(d.camProfile, static_cast<int>(CamGrind::Count));
    d.portWork      = idx(d.portWork, static_cast<int>(PortWork::Count));
    d.intakeValves  = std::clamp(d.intakeValves, 1, 3);
    d.exhaustValves = std::clamp(d.exhaustValves, 1, 3);
    const double vmax = maxValveFraction(d.intakeValves, d.exhaustValves);
    d.intakeValveFrac   = std::clamp(d.intakeValveFrac, 0.15, vmax);
    d.exhaustValveFrac  = std::clamp(d.exhaustValveFrac, 0.13, vmax);
    d.liftRatio         = std::clamp(d.liftRatio, 0.10, liftCapFor(d.valvetrain));
    d.intakeDuration    = std::clamp(d.intakeDuration, 140.0, 330.0);
    d.exhaustDuration   = std::clamp(d.exhaustDuration, 140.0, 330.0);
    d.intakeCentre      = std::clamp(d.intakeCentre, 80.0, 135.0);
    d.exhaustCentre     = std::clamp(d.exhaustCentre, 80.0, 135.0);
    d.vvtRange          = std::clamp(d.vvtRange, 0.0, 45.0);

    d.fuel = idx(d.fuel, static_cast<int>(FuelKind::Count));
    const bool diesel = kFuelSpec[d.fuel].compressionIgnition;
    // A compression-ignition engine has to be able to light its own fuel: below
    // about 14:1 it simply never gets hot enough, so the floor moves with the
    // fuel rather than letting you build something that cannot run.
    d.compression  = std::clamp(d.compression, diesel ? 14.0 : 5.0, 26.0);
    d.lambdaCruise = std::clamp(d.lambdaCruise, 0.65, 1.60);
    d.lambdaPower  = std::clamp(d.lambdaPower, 0.60, 1.40);
    d.sparkIdle     = std::clamp(d.sparkIdle, -5.0, 50.0);
    d.sparkPeak     = std::clamp(d.sparkPeak, -5.0, 60.0);
    d.sparkPartLoad = std::clamp(d.sparkPartLoad, 0.0, 30.0);
    d.burnDuration  = std::clamp(d.burnDuration, 15.0, 120.0);
    d.ignitionDelay = std::clamp(d.ignitionDelay, 0.0, 30.0);
    d.combustionEff = std::clamp(d.combustionEff, 0.60, 1.00);
    d.redline = std::clamp(d.redline, 1500.0, diesel ? 6500.0 : 22000.0);
    // A long-overlap cam cannot idle slowly. So much of what it draws in goes
    // straight out of the exhaust that the charge left behind will barely burn,
    // which is why a race engine idles at two thousand and lopes even there.
    // The floor moves with the overlap the cam actually has.
    const double overlapDeg = 0.5 * (d.intakeDuration + d.exhaustDuration) -
                              d.intakeCentre - d.exhaustCentre;
    const double idleFloor = 350.0 + 13.0 * std::max(0.0, overlapDeg - 20.0);
    d.idleRpm = std::clamp(d.idleRpm, std::min(idleFloor, d.redline * 0.35),
                           std::min(3000.0, d.redline * 0.6));

    d.oilGrade      = idx(d.oilGrade, static_cast<int>(OilGrade::Count));
    d.oilTempTarget = std::clamp(d.oilTempTarget, 60.0, 150.0);
    d.oilStartTemp  = std::clamp(d.oilStartTemp, -30.0, d.oilTempTarget);
    d.frictionScale = std::clamp(d.frictionScale, 0.40, 2.50);
    d.accessoryLoad = std::clamp(d.accessoryLoad, 0.0, 60.0);

    d.throttleBore = std::clamp(d.throttleBore, 20.0, 140.0);
    d.plenumVolume = std::clamp(d.plenumVolume, 0.15, 20.0);
    d.runnerLength = std::clamp(d.runnerLength, 60.0, 900.0);
    d.runnerDia    = std::clamp(d.runnerDia, 15.0, 90.0);
    d.charger      = idx(d.charger, static_cast<int>(ChargerKind::Count));
    d.boost        = std::clamp(d.boost, 0.0, 4.0);
    if (d.charger == static_cast<int>(ChargerKind::None)) d.boost = 0.0;
    else if (d.boost < 0.05) d.boost = 0.05;
    d.spoolRpm    = std::clamp(d.spoolRpm, 800.0, 9000.0);
    d.turboLag    = std::clamp(d.turboLag, 0.05, 4.0);
    d.intercooler = std::clamp(d.intercooler, 0.0, 0.95);

    d.primaryLength = std::clamp(d.primaryLength, 80.0, 1200.0);
    d.primaryDia    = std::clamp(d.primaryDia, 15.0, 90.0);
    d.collectorVol  = std::clamp(d.collectorVol, 0.2, 25.0);
    d.header  = idx(d.header, static_cast<int>(HeaderStyle::Count));
    d.muffler = idx(d.muffler, static_cast<int>(MufflerKind::Count));

    d.gearCount = std::clamp(d.gearCount, 1, 8);
    for (double& g : d.gears) g = std::clamp(g, 0.30, 6.50);
    d.finalDrive     = std::clamp(d.finalDrive, 1.50, 7.00);
    d.wheelRadius    = std::clamp(d.wheelRadius, 0.15, 0.75);
    d.vehicleMass    = std::clamp(d.vehicleMass, 120.0, 8000.0);
    d.dragArea       = std::clamp(d.dragArea, 0.10, 3.00);
    d.clutchCapacity = std::clamp(d.clutchCapacity, 40.0, 3000.0);
    d.brakeTorque    = std::clamp(d.brakeTorque, 200.0, 12000.0);

    d.theme      = idx(d.theme, static_cast<int>(ThemeKind::Count));
    d.accentHue  = std::clamp(d.accentHue, 0.0, 360.0);
    d.coverHue   = std::clamp(d.coverHue, 0.0, 360.0);
    d.coverSat   = std::clamp(d.coverSat, 0.0, 100.0);
    d.blockShade = std::clamp(d.blockShade, 5.0, 90.0);
}

EngineParams paramsFromDesign(const EngineDesign& din) {
    EngineDesign d = din;
    clampDesign(d);

    EngineParams p;
    p.bore   = d.bore * 1e-3;
    p.stroke = d.stroke * 1e-3;
    p.rodLength = p.stroke * d.rodRatio;
    p.compressionRatio = d.compression;
    p.cylinders = d.cylinders;
    p.bankAngle = d.bankAngle;

    std::vector<int> order;
    computeFiring(d, p.phaseOffsets, p.cylinderBank, order);
    p.banks = banksFor(d.layout, d.cylinders);

    const double displacement = kPi * 0.25 * p.bore * p.bore * p.stroke;
    const double totalDisp = displacement * d.cylinders;
    const double dispRatio = totalDisp / 0.002;      // against the 2.0 litre reference

    // ---- Head --------------------------------------------------------------
    const double iDia = p.bore * d.intakeValveFrac;
    const double eDia = p.bore * d.exhaustValveFrac;
    const double lift = std::min(d.liftRatio, liftCapFor(d.valvetrain));
    static const double kCamExp[] = {0.95, 0.86, 0.78, 0.68};
    static const double kPortCd[] = {0.90, 1.00, 1.09};
    const double profExp = std::max(kCamExp[idx(d.camProfile, 4)],
                                    profileFloorFor(d.valvetrain));
    const double cd = kPortCd[idx(d.portWork, 3)];

    p.intake.openDeg    = wrap720(360.0 + d.intakeCentre - d.intakeDuration * 0.5);
    p.intake.closeDeg   = wrap720(360.0 + d.intakeCentre + d.intakeDuration * 0.5);
    p.intake.maxLift    = iDia * lift;
    p.intake.diameter   = iDia;
    p.intake.count      = d.intakeValves;
    p.intake.cdScale    = cd;
    p.intake.profileExp = profExp;

    p.exhaust.openDeg    = wrap720(360.0 - d.exhaustCentre - d.exhaustDuration * 0.5);
    p.exhaust.closeDeg   = wrap720(360.0 - d.exhaustCentre + d.exhaustDuration * 0.5);
    p.exhaust.maxLift    = eDia * lift;
    p.exhaust.diameter   = eDia;
    p.exhaust.count      = d.exhaustValves;
    p.exhaust.cdScale    = cd * 0.92;
    p.exhaust.profileExp = profExp;

    static const double kVtDrag[] = {1.25, 1.06, 1.00, 1.18, 1.10};
    p.valveFloatRpm  = floatRpmFor(d, iDia);
    p.valvetrainDrag = kVtDrag[idx(d.valvetrain, 5)];
    p.vvtRange   = d.vvt ? d.vvtRange : 0.0;
    p.vvtLowRpm  = std::max(800.0, d.redline * 0.29);
    p.vvtHighRpm = std::max(p.vvtLowRpm + 500.0, d.redline * 0.82);

    // ---- Combustion --------------------------------------------------------
    p.sparkIdle      = d.sparkIdle;
    p.sparkHighSpeed = d.sparkPeak;
    p.sparkPartLoad  = d.sparkPartLoad;
    p.burnDuration   = d.burnDuration;
    p.ignitionDelay  = d.ignitionDelay;
    p.lambdaCruise   = d.lambdaCruise;
    p.lambdaPower    = d.lambdaPower;
    p.fuel           = kFuelSpec[idx(d.fuel, static_cast<int>(FuelKind::Count))];
    p.combustionEff  = d.combustionEff;
    p.knockControl   = d.knockControl;
    p.knockScale     = 2.2;
    p.redline        = d.redline;
    p.idleTargetRpm  = d.idleRpm;

    // ---- Mechanics ---------------------------------------------------------
    p.inertia       = d.flywheel;
    p.recipMass     = d.recipMass;
    p.starterTorque = 50.0 + 60.0 * std::pow(std::max(dispRatio, 0.05), 0.7);
    p.cfA = 0.55 * d.frictionScale;
    p.cfB = 0.008;
    p.cfC = 0.020 * d.frictionScale;
    p.cfD = 0.0016 * d.frictionScale;
    p.accessoryTorque = d.accessoryLoad;
    p.oil = kOilSpec[idx(d.oilGrade, static_cast<int>(OilGrade::Count))];
    p.oilTempTarget = d.oilTempTarget + 273.15;
    p.oilStartTemp  = d.oilStartTemp + 273.15;

    // ---- Induction ---------------------------------------------------------
    p.throttleBore = d.throttleBore * 1e-3;
    p.plenumVolume = d.plenumVolume * 1e-3;
    p.intakeRunner.length = d.runnerLength * 1e-3;
    p.intakeRunner.area   = kPi * 0.25 * (d.runnerDia * 1e-3) * (d.runnerDia * 1e-3);
    p.intakeRunner.zeta   = 1.1;
    // Wall friction per unit mass flow rises as the pipe gets narrower.
    p.intakeRunner.viscous = 9.0e4 * (0.00110 / p.intakeRunner.area);

    // The idle bypass is a fraction of throttle area, so it has to be rescaled
    // whenever either the engine or the throttle body changes size - otherwise
    // a big engine behind a small throttle will not idle at all.
    const double throttleArea = kPi * 0.25 * p.throttleBore * p.throttleBore;
    // ... and with the idle speed itself: an engine asked to idle at 1300 rpm
    // needs half again the air of one idling at 850, or the valve saturates and
    // it settles wherever it likes instead of where it was asked to.
    const double bypassScale = std::clamp(dispRatio * (0.002124 / throttleArea) *
                                          (d.idleRpm / 850.0), 0.15, 12.0);
    // The valve needs enough authority for the worst case in the range - a big
    // engine with a long-overlap cam asked to idle high needs a great deal of
    // air, and a race cam needs more again because so much of what it draws in
    // goes straight out of the exhaust. The loop simply runs it part open when
    // that authority is not needed.
    const double overlap = 0.5 * (d.intakeDuration + d.exhaustDuration) -
                           d.intakeCentre - d.exhaustCentre;
    const double camThirst = 1.0 + std::clamp(overlap, 0.0, 120.0) / 45.0;
    p.idleBypassMin = std::clamp(0.0012 * bypassScale, 0.0002, 0.020);
    p.idleBypassMax = std::clamp(0.0300 * bypassScale * camThirst, 0.0060, 0.300);

    p.charger        = d.charger;
    p.boostTarget    = d.boost;
    p.spoolRpm       = d.spoolRpm;
    p.turboLag       = d.turboLag;
    p.interCoolerEff = d.intercooler;
    static const double kCompEff[] = {0.70, 0.72, 0.55, 0.74};
    p.compressorEff  = kCompEff[idx(d.charger, 4)];
    p.blowerDriveEff = 0.85;
    // A turbine has to be pushed through, and the more boost it is asked for
    // the tighter the housing: that backpressure is what a turbo charges you.
    p.turbineRestrict = d.charger == static_cast<int>(ChargerKind::Turbo)
                      ? std::clamp(0.62 - 0.06 * d.boost, 0.32, 0.80) : 1.0;

    // ---- Exhaust -----------------------------------------------------------
    static const double kHeaderLen[]  = {0.45, 1.00, 1.15, 1.00, 1.00};
    static const double kHeaderZeta[] = {2.80, 1.50, 1.70, 1.40, 0.90};
    static const double kHeaderVol[]  = {0.60, 1.00, 1.40, 0.80, 0.25};
    static const double kMufflerA[]   = {0.0030, 0.0019, 0.0024, 0.0013};
    const int hd = idx(d.header, static_cast<int>(HeaderStyle::Count));
    p.exhaustRunner.length = d.primaryLength * 1e-3 * kHeaderLen[hd];
    p.exhaustRunner.area   = kPi * 0.25 * (d.primaryDia * 1e-3) * (d.primaryDia * 1e-3);
    p.exhaustRunner.zeta   = kHeaderZeta[hd];
    p.exhaustRunner.viscous = 7.0e4 * (0.00090 / p.exhaustRunner.area);
    p.collectorVolume = d.collectorVol * 1e-3 * kHeaderVol[hd];
    p.outletArea = kMufflerA[idx(d.muffler, static_cast<int>(MufflerKind::Count))] *
                   std::sqrt(std::max(dispRatio, 0.05)) *
                   (hd == static_cast<int>(HeaderStyle::Open) ? 1.6 : 1.0);

    // How the system sounds, as opposed to how it flows. Unequal primaries are
    // not a flow property at all - they are the whole reason one boxer sounds
    // like a boxer and another does not.
    static const double kMismatch[] = {0.16, 0.03, 0.06, 0.04, 0.02};
    static const double kTail[]     = {0.55, 0.90, 0.72, 1.10};
    static const double kDamp[]     = {0.55, 0.26, 0.34, 0.16};
    static const double kLoud[]     = {1.15, 0.85, 1.00, 0.62};
    const int mf = idx(d.muffler, static_cast<int>(MufflerKind::Count));
    p.primaryMismatch = kMismatch[hd];
    p.tailLength      = kTail[mf];
    p.mufflerDamping  = kDamp[mf];
    p.exhaustLoudness = kLoud[mf];
    if (hd == static_cast<int>(HeaderStyle::Open)) {
        p.tailLength      = 0.30;
        p.mufflerDamping  = 0.78;
        p.exhaustLoudness = 1.35;
    }

    return p;
}

DrivetrainParams drivetrainFromDesign(const EngineDesign& din) {
    EngineDesign d = din;
    clampDesign(d);
    DrivetrainParams t;
    t.gearRatios.assign(d.gears.begin(), d.gears.begin() + d.gearCount);
    t.finalDrive     = d.finalDrive;
    t.wheelRadius    = d.wheelRadius;
    t.mass           = d.vehicleMass;
    t.dragArea       = d.dragArea;
    t.clutchCapacity = d.clutchCapacity;
    t.brakeTorque    = d.brakeTorque;
    t.lockRpm        = std::clamp(d.idleRpm * 1.55, 700.0, 4000.0);
    t.slipRpm        = std::clamp(d.idleRpm * 0.65, 250.0, 2000.0);
    return t;
}

DesignSummary summarise(const EngineDesign& din) {
    EngineDesign d = din;
    clampDesign(d);
    DesignSummary s;

    const double boreM = d.bore * 1e-3, strokeM = d.stroke * 1e-3;
    const double swept = kPi * 0.25 * boreM * boreM * strokeM;
    s.displacementL  = swept * d.cylinders * 1000.0;
    s.boreStroke     = d.bore / d.stroke;
    s.rodLengthMm    = d.stroke * d.rodRatio;
    s.intakeValveMm  = d.bore * d.intakeValveFrac;
    s.exhaustValveMm = d.bore * d.exhaustValveFrac;
    const double lift = std::min(d.liftRatio, liftCapFor(d.valvetrain));
    s.intakeLiftMm  = s.intakeValveMm * lift;
    s.exhaustLiftMm = s.exhaustValveMm * lift;

    s.lsa     = 0.5 * (d.intakeCentre + d.exhaustCentre);
    s.overlap = 0.5 * (d.intakeDuration + d.exhaustDuration) - d.intakeCentre - d.exhaustCentre;
    s.ivo = 0.5 * d.intakeDuration - d.intakeCentre;             // deg BTDC
    s.ivc = 0.5 * d.intakeDuration + d.intakeCentre - 180.0;     // deg ABDC
    s.evo = 0.5 * d.exhaustDuration + d.exhaustCentre - 180.0;   // deg BBDC
    s.evc = 0.5 * d.exhaustDuration - d.exhaustCentre;           // deg ATDC

    // Intake: runner and cylinder form a Helmholtz resonator, and the engine
    // speed where that helps most is a fixed multiple of its frequency. The
    // multiplier is a calibration against known-good manifolds, not a
    // derivation.
    const double area = kPi * 0.25 * (d.runnerDia * 1e-3) * (d.runnerDia * 1e-3);
    const double clearance = swept / (d.compression - 1.0);
    const double vEff = 0.5 * swept + clearance;
    const double fHelm = (343.0 / (2.0 * kPi)) *
                         std::sqrt(area / std::max(d.runnerLength * 1e-3 * vEff, 1e-9));
    s.tunedRpmIntake = 26.0 * fHelm;
    // Exhaust: the returning suction wave should arrive during overlap. This is
    // the header-length rule of thumb, rearranged for speed.
    const double ed = 180.0 + s.evo;
    s.tunedRpmExhaust = ed * 850.0 / std::max(d.primaryLength / 25.4 + 3.0, 1.0);

    s.firingInterval = 720.0 / d.cylinders;
    s.valveFloatRpm  = floatRpmFor(d, boreM * d.intakeValveFrac);
    s.stoichAfr = kFuelSpec[idx(d.fuel, static_cast<int>(FuelKind::Count))].stoichAfr;
    s.fuelLhvMJ = kFuelSpec[idx(d.fuel, static_cast<int>(FuelKind::Count))].lhv * 1e-6;
    s.banks     = banksFor(d.layout, d.cylinders);

    std::vector<int> bank;
    computeFiring(d, s.phases, bank, s.firingOrder);
    return s;
}

// ---------------------------------------------------------------------------
// Presets. Each one is a complete, running engine, and between them they cover
// most of the shapes the model can take.
// ---------------------------------------------------------------------------
namespace {

EngineDesign makeTurboFour() {
    EngineDesign d;
    d.name = "Turbo 2.0 Inline-4";
    d.sparkPeak = 28.0;
    d.compression = 9.6;
    d.intakeDuration = 224.0; d.exhaustDuration = 232.0;
    d.intakeCentre = 108.0;   d.exhaustCentre = 112.0;
    d.fuel = static_cast<int>(FuelKind::Petrol98);
    d.charger = static_cast<int>(ChargerKind::Turbo);
    d.boost = 1.10; d.spoolRpm = 2600.0; d.turboLag = 0.75; d.intercooler = 0.78;
    d.lambdaPower = 0.82;
    d.throttleBore = 60.0; d.plenumVolume = 3.0;
    d.primaryLength = 300.0; d.primaryDia = 32.0;
    d.muffler = static_cast<int>(MufflerKind::Straight);
    d.redline = 7000.0;
    d.vvt = true; d.vvtRange = 30.0;
    d.clutchCapacity = 480.0;
    d.accentHue = 200.0; d.coverHue = 8.0; d.coverSat = 60.0;
    return d;
}

EngineDesign makeCrossplaneV8() {
    EngineDesign d;
    d.name = "Crossplane 5.0 V8";
    d.layout = static_cast<int>(Layout::Vee);
    d.cylinders = 8; d.bankAngle = 90.0;
    d.crankType = static_cast<int>(CrankType::Crossplane);
    d.bore = 94.0; d.stroke = 90.0; d.rodRatio = 1.60;
    d.compression = 11.0; d.recipMass = 0.62; d.flywheel = 0.30;
    d.valvetrain = static_cast<int>(Valvetrain::DOHC);
    d.intakeDuration = 236.0; d.exhaustDuration = 244.0;
    d.fuel = static_cast<int>(FuelKind::Petrol95);
    d.redline = 7200.0; d.idleRpm = 750.0;
    d.throttleBore = 72.0; d.plenumVolume = 5.5;
    d.runnerLength = 260.0; d.runnerDia = 42.0;
    d.primaryLength = 480.0; d.primaryDia = 38.0; d.collectorVol = 2.6;
    d.header = static_cast<int>(HeaderStyle::PerBank);
    d.muffler = static_cast<int>(MufflerKind::Chambered);
    d.oilGrade = static_cast<int>(OilGrade::W5_30);
    d.vehicleMass = 1720.0; d.clutchCapacity = 620.0; d.finalDrive = 3.55;
    d.accentHue = 12.0; d.coverHue = 350.0; d.coverSat = 55.0; d.blockShade = 28.0;
    return d;
}

EngineDesign makeFlatplaneV8() {
    EngineDesign d = makeCrossplaneV8();
    d.name = "Flat-plane 4.5 V8";
    d.crankType = static_cast<int>(CrankType::EvenFire);
    d.bore = 94.0; d.stroke = 81.0; d.rodRatio = 1.85;
    d.compression = 12.5; d.recipMass = 0.42; d.flywheel = 0.16;
    d.valveMaterial = static_cast<int>(ValveMetal::Titanium);
    d.camProfile = static_cast<int>(CamGrind::Race);
    d.portWork = static_cast<int>(PortWork::CNC);
    d.intakeDuration = 268.0; d.exhaustDuration = 272.0;
    d.intakeCentre = 100.0; d.exhaustCentre = 104.0;
    d.liftRatio = 0.34;
    d.fuel = static_cast<int>(FuelKind::Petrol98);
    d.redline = 9000.0; d.idleRpm = 1250.0;
    d.runnerLength = 180.0; d.runnerDia = 46.0;
    d.primaryLength = 560.0;
    d.header = static_cast<int>(HeaderStyle::FourIntoOne);
    d.muffler = static_cast<int>(MufflerKind::Straight);
    d.oilGrade = static_cast<int>(OilGrade::RaceSynthetic);
    // Geared to reach the limiter near its actual top speed rather than a
    // theoretical 410 km/h it could never pull.
    d.gears = {3.20, 2.15, 1.62, 1.30, 1.09, 0.94, 0.84, 0.76};
    d.vehicleMass = 1450.0;
    d.accentHue = 50.0; d.coverHue = 40.0; d.coverSat = 70.0;
    return d;
}

EngineDesign makeInlineSix() {
    EngineDesign d;
    d.name = "Naturally aspirated 3.0 Inline-6";
    d.cylinders = 6;
    d.bore = 84.0; d.stroke = 90.0; d.rodRatio = 1.75;
    d.compression = 11.5; d.flywheel = 0.26;
    d.intakeDuration = 244.0; d.exhaustDuration = 248.0;
    d.fuel = static_cast<int>(FuelKind::Petrol98);
    d.redline = 8000.0;
    d.vvt = true; d.vvtRange = 32.0;
    d.throttleBore = 66.0; d.plenumVolume = 3.6;
    d.runnerLength = 340.0; d.runnerDia = 40.0;
    d.primaryLength = 520.0; d.primaryDia = 36.0;
    d.vehicleMass = 1480.0; d.clutchCapacity = 430.0;
    d.accentHue = 195.0; d.coverHue = 220.0; d.coverSat = 30.0;
    return d;
}

EngineDesign makeBoxer() {
    EngineDesign d;
    d.name = "Turbo 2.5 Flat-4";
    d.sparkPeak = 28.0;
    d.layout = static_cast<int>(Layout::Flat);
    d.cylinders = 4; d.bankAngle = 180.0;
    d.bore = 99.5; d.stroke = 79.0; d.rodRatio = 1.72;
    d.compression = 9.0;
    d.intakeDuration = 226.0; d.exhaustDuration = 234.0;
    d.fuel = static_cast<int>(FuelKind::Petrol98);
    d.charger = static_cast<int>(ChargerKind::Turbo);
    d.boost = 1.05; d.spoolRpm = 3000.0; d.turboLag = 1.0; d.intercooler = 0.72;
    d.lambdaPower = 0.80;
    d.redline = 6800.0;
    // Unequal-length primaries are exactly why this layout sounds the way it
    // does; the header style is what carries that into the exhaust model.
    d.header = static_cast<int>(HeaderStyle::LogManifold);
    d.primaryLength = 520.0; d.primaryDia = 34.0;
    d.muffler = static_cast<int>(MufflerKind::Chambered);
    d.vehicleMass = 1550.0; d.clutchCapacity = 460.0;
    d.accentHue = 210.0; d.coverHue = 30.0; d.coverSat = 45.0;
    return d;
}

EngineDesign makeV12() {
    EngineDesign d;
    d.name = "Race 6.0 V12";
    d.layout = static_cast<int>(Layout::Vee);
    d.cylinders = 12; d.bankAngle = 60.0;
    d.bore = 88.0; d.stroke = 82.0; d.rodRatio = 1.90;
    d.compression = 13.0; d.recipMass = 0.36; d.flywheel = 0.22;
    d.valvetrain = static_cast<int>(Valvetrain::Pneumatic);
    d.valveMaterial = static_cast<int>(ValveMetal::Titanium);
    d.camProfile = static_cast<int>(CamGrind::Race);
    d.portWork = static_cast<int>(PortWork::CNC);
    d.intakeDuration = 280.0; d.exhaustDuration = 282.0;
    d.intakeCentre = 98.0; d.exhaustCentre = 102.0;
    d.liftRatio = 0.34;
    d.fuel = static_cast<int>(FuelKind::RaceFuel);
    d.lambdaPower = 0.85;
    d.redline = 10500.0; d.idleRpm = 1900.0;
    d.throttleBore = 88.0; d.plenumVolume = 6.0;
    d.runnerLength = 150.0; d.runnerDia = 48.0;
    d.primaryLength = 620.0; d.primaryDia = 40.0; d.collectorVol = 3.4;
    d.header = static_cast<int>(HeaderStyle::PerBank);
    d.muffler = static_cast<int>(MufflerKind::Straight);
    d.oilGrade = static_cast<int>(OilGrade::RaceSynthetic);
    d.accessoryLoad = 14.0;
    d.gears = {3.10, 2.12, 1.63, 1.34, 1.14, 0.99, 0.88, 0.80};
    d.vehicleMass = 1180.0; d.clutchCapacity = 800.0; d.finalDrive = 4.10;
    d.accentHue = 350.0; d.coverHue = 355.0; d.coverSat = 65.0;
    return d;
}

EngineDesign makeBlownMethanol() {
    EngineDesign d;
    d.name = "Blown methanol 7.0 V8";
    d.sparkPeak = 26.0;
    d.layout = static_cast<int>(Layout::Vee);
    d.cylinders = 8; d.bankAngle = 90.0;
    d.crankType = static_cast<int>(CrankType::Crossplane);
    d.bore = 108.0; d.stroke = 95.0; d.rodRatio = 1.65;
    d.compression = 10.0; d.recipMass = 0.85; d.flywheel = 0.36;
    d.valvetrain = static_cast<int>(Valvetrain::OHV);
    d.intakeValves = 1; d.exhaustValves = 1;
    d.valveMaterial = static_cast<int>(ValveMetal::Titanium);
    d.intakeValveFrac = 0.47; d.exhaustValveFrac = 0.39;
    d.liftRatio = 0.26;
    d.camProfile = static_cast<int>(CamGrind::Race);
    d.intakeDuration = 268.0; d.exhaustDuration = 276.0;
    d.intakeCentre = 106.0; d.exhaustCentre = 110.0;
    d.fuel = static_cast<int>(FuelKind::Methanol);
    d.lambdaCruise = 0.85; d.lambdaPower = 0.70;
    d.charger = static_cast<int>(ChargerKind::Roots);
    d.boost = 1.6; d.spoolRpm = 2000.0; d.intercooler = 0.0;
    d.redline = 7000.0; d.idleRpm = 1000.0;
    d.throttleBore = 100.0; d.plenumVolume = 7.0;
    d.runnerLength = 140.0; d.runnerDia = 52.0;
    d.primaryLength = 460.0; d.primaryDia = 44.0;
    d.header = static_cast<int>(HeaderStyle::Open);
    d.muffler = static_cast<int>(MufflerKind::Straight);
    d.oilGrade = static_cast<int>(OilGrade::W20_60);
    d.vehicleMass = 1300.0; d.clutchCapacity = 1400.0;
    d.accentHue = 285.0; d.coverHue = 280.0; d.coverSat = 60.0;
    return d;
}

EngineDesign makeDiesel() {
    EngineDesign d;
    d.name = "3.0 Inline-6 turbo diesel";
    d.cylinders = 6;
    d.bore = 84.0; d.stroke = 90.0; d.rodRatio = 1.80;
    d.compression = 16.5; d.recipMass = 0.95; d.flywheel = 0.55;
    d.valvetrain = static_cast<int>(Valvetrain::DOHC);
    d.intakeDuration = 200.0; d.exhaustDuration = 210.0;
    d.intakeCentre = 112.0; d.exhaustCentre = 118.0;
    d.fuel = static_cast<int>(FuelKind::Diesel);
    d.sparkIdle = 6.0; d.sparkPeak = 14.0; d.sparkPartLoad = 4.0;
    d.burnDuration = 60.0; d.ignitionDelay = 6.0;
    d.charger = static_cast<int>(ChargerKind::Turbo);
    d.boost = 1.5; d.spoolRpm = 1800.0; d.turboLag = 0.8; d.intercooler = 0.80;
    d.redline = 4600.0; d.idleRpm = 700.0;
    d.throttleBore = 70.0; d.plenumVolume = 4.0;
    d.runnerLength = 280.0; d.runnerDia = 40.0;
    d.primaryLength = 300.0; d.primaryDia = 36.0;
    d.header = static_cast<int>(HeaderStyle::LogManifold);
    d.muffler = static_cast<int>(MufflerKind::Quiet);
    d.oilGrade = static_cast<int>(OilGrade::W10_40);
    d.accessoryLoad = 16.0;
    d.gearCount = 6;
    d.gears = {4.10, 2.32, 1.54, 1.18, 0.94, 0.76, 0.64, 0.56};
    d.finalDrive = 3.20; d.vehicleMass = 2100.0; d.clutchCapacity = 700.0;
    d.accentHue = 40.0; d.coverHue = 210.0; d.coverSat = 10.0; d.blockShade = 26.0;
    return d;
}

EngineDesign makeSuperbike() {
    EngineDesign d;
    d.name = "600cc Inline-4 superbike";
    d.cylinders = 4;
    d.bore = 67.0; d.stroke = 42.5; d.rodRatio = 2.10;
    d.compression = 13.0; d.recipMass = 0.16; d.flywheel = 0.045;
    d.valvetrain = static_cast<int>(Valvetrain::DOHC);
    d.valveMaterial = static_cast<int>(ValveMetal::Titanium);
    d.camProfile = static_cast<int>(CamGrind::Race);
    d.portWork = static_cast<int>(PortWork::Ported);
    d.intakeDuration = 262.0; d.exhaustDuration = 264.0;
    d.intakeCentre = 100.0; d.exhaustCentre = 104.0;
    d.liftRatio = 0.33;
    d.fuel = static_cast<int>(FuelKind::Petrol98);
    d.redline = 15500.0; d.idleRpm = 1300.0;
    d.throttleBore = 38.0; d.plenumVolume = 0.9;
    d.runnerLength = 110.0; d.runnerDia = 33.0;
    d.primaryLength = 380.0; d.primaryDia = 30.0; d.collectorVol = 0.9;
    d.muffler = static_cast<int>(MufflerKind::Absorption);
    d.oilGrade = static_cast<int>(OilGrade::W10_40);
    d.accessoryLoad = 2.5;
    d.gearCount = 6;
    d.gears = {2.75, 2.00, 1.60, 1.36, 1.19, 1.07, 1.00, 0.95};
    // A bike has a primary reduction between the crank and the gearbox that a
    // car does not, so the overall ratio folds that in with the chain: about
    // 2.07 by 2.9. Leaving it out gears a 600 for 550 km/h.
    d.finalDrive = 6.05; d.wheelRadius = 0.30; d.vehicleMass = 250.0;
    d.dragArea = 0.35; d.clutchCapacity = 140.0; d.brakeTorque = 900.0;
    d.accentHue = 105.0; d.coverHue = 100.0; d.coverSat = 50.0;
    return d;
}

EngineDesign makeVTwin() {
    EngineDesign d;
    d.name = "45 degree V-twin cruiser";
    d.layout = static_cast<int>(Layout::Vee);
    d.cylinders = 2; d.bankAngle = 45.0;
    d.crankType = static_cast<int>(CrankType::OddFire);
    d.bore = 101.0; d.stroke = 111.0; d.rodRatio = 1.55;
    d.compression = 9.5; d.recipMass = 0.75; d.flywheel = 0.42;
    d.valvetrain = static_cast<int>(Valvetrain::OHV);
    d.intakeValves = 1; d.exhaustValves = 1;
    d.intakeValveFrac = 0.45; d.exhaustValveFrac = 0.38;
    d.liftRatio = 0.24;
    d.camProfile = static_cast<int>(CamGrind::Street);
    d.intakeDuration = 218.0; d.exhaustDuration = 226.0;
    d.fuel = static_cast<int>(FuelKind::Petrol91);
    d.redline = 5800.0; d.idleRpm = 900.0;
    d.throttleBore = 46.0; d.plenumVolume = 0.8;
    d.runnerLength = 200.0; d.runnerDia = 40.0;
    d.primaryLength = 700.0; d.primaryDia = 42.0; d.collectorVol = 0.8;
    d.header = static_cast<int>(HeaderStyle::Open);
    d.muffler = static_cast<int>(MufflerKind::Chambered);
    d.oilGrade = static_cast<int>(OilGrade::W20_60);
    d.accessoryLoad = 3.0;
    d.gearCount = 5;
    d.gears = {3.20, 2.10, 1.55, 1.24, 1.00, 0.90, 0.82, 0.76};
    // Primary reduction and belt drive together, as above.
    d.finalDrive = 3.90; d.wheelRadius = 0.33; d.vehicleMass = 340.0;
    d.dragArea = 0.60; d.clutchCapacity = 260.0; d.brakeTorque = 1100.0;
    d.accentHue = 32.0; d.coverHue = 30.0; d.coverSat = 25.0; d.blockShade = 22.0;
    return d;
}

EngineDesign makeHydrogen() {
    EngineDesign d = makeInlineSix();
    d.name = "Hydrogen 3.0 Inline-6";
    d.fuel = static_cast<int>(FuelKind::Hydrogen);
    d.compression = 12.5;
    d.lambdaCruise = 1.60; d.lambdaPower = 1.30;   // hydrogen runs very lean
    d.charger = static_cast<int>(ChargerKind::Turbo);
    d.boost = 0.9; d.spoolRpm = 2400.0; d.intercooler = 0.85;
    d.burnDuration = 30.0;
    d.accentHue = 165.0; d.coverHue = 170.0; d.coverSat = 40.0;
    return d;
}

using Maker = EngineDesign (*)();
const Maker kMakers[] = {
    nullptr,            // index 0 is the default-constructed design
    makeTurboFour, makeCrossplaneV8, makeFlatplaneV8, makeInlineSix,
    makeBoxer, makeV12, makeBlownMethanol, makeDiesel, makeSuperbike,
    makeVTwin, makeHydrogen,
};

} // namespace

int presetCount() { return static_cast<int>(sizeof(kMakers) / sizeof(kMakers[0])); }

EngineDesign preset(int i) {
    EngineDesign d;
    if (i > 0 && i < presetCount() && kMakers[i]) d = kMakers[i]();
    clampDesign(d);
    return d;
}

const char* presetName(int i) {
    static std::string cache[16];
    const int n = presetCount();
    if (i < 0 || i >= n) return "";
    if (cache[i].empty()) cache[i] = preset(i).name;
    return cache[i].c_str();
}

// ---------------------------------------------------------------------------
// Save and load. One field per line, so a saved engine can be read, diffed and
// hand-edited - which is most of the value of a text format.
// ---------------------------------------------------------------------------
namespace {

// Every scalar field, listed once. Both directions of the file format and
// nothing else uses it, so there is exactly one place to add a new setting.
#define DESIGN_FIELDS(X)                                                       \
    X(layout) X(cylinders) X(bankAngle) X(crankType)                           \
    X(bore) X(stroke) X(rodRatio) X(compression) X(recipMass) X(flywheel)      \
    X(valvetrain) X(intakeValves) X(exhaustValves) X(valveMaterial)            \
    X(intakeValveFrac) X(exhaustValveFrac) X(liftRatio)                        \
    X(intakeDuration) X(intakeCentre) X(exhaustDuration) X(exhaustCentre)      \
    X(camProfile) X(portWork) X(vvt) X(vvtRange)                               \
    X(fuel) X(lambdaCruise) X(lambdaPower) X(sparkIdle) X(sparkPeak)           \
    X(sparkPartLoad) X(burnDuration) X(ignitionDelay) X(knockControl)          \
    X(combustionEff) X(redline) X(idleRpm)                                     \
    X(oilGrade) X(oilTempTarget) X(oilStartTemp) X(frictionScale)              \
    X(accessoryLoad)                                                           \
    X(throttleBore) X(plenumVolume) X(runnerLength) X(runnerDia)               \
    X(charger) X(boost) X(spoolRpm) X(turboLag) X(intercooler)                 \
    X(primaryLength) X(primaryDia) X(collectorVol) X(header) X(muffler)        \
    X(gearCount) X(finalDrive) X(wheelRadius) X(vehicleMass) X(dragArea)       \
    X(clutchCapacity) X(brakeTorque)                                           \
    X(theme) X(accentHue) X(coverHue) X(coverSat) X(blockShade)                \
    X(showCutaway) X(showTopView)

void assign(int& dst, const std::string& v)    { dst = std::atoi(v.c_str()); }
void assign(double& dst, const std::string& v) { dst = std::atof(v.c_str()); }
void assign(bool& dst, const std::string& v)   { dst = v == "1" || v == "true"; }

} // namespace

bool saveDesign(const EngineDesign& d, const std::string& path) {
    std::ofstream out(path);
    if (!out) return false;
    out << "# Enginio2D engine design\n";
    out << "name = " << d.name << "\n";
    out << std::setprecision(10);
#define X(f) out << #f << " = " << d.f << "\n";
    DESIGN_FIELDS(X)
#undef X
    for (int i = 0; i < 8; ++i)
        out << "gear" << (i + 1) << " = " << d.gears[static_cast<std::size_t>(i)] << "\n";
    return static_cast<bool>(out);
}

bool loadDesign(EngineDesign& d, const std::string& path) {
    std::ifstream in(path);
    if (!in) return false;
    EngineDesign fresh;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        const auto trim = [](std::string& s) {
            const std::size_t a = s.find_first_not_of(" \t\r\n");
            const std::size_t b = s.find_last_not_of(" \t\r\n");
            s = (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
        };
        trim(key); trim(val);
        if (key == "name") { fresh.name = val; continue; }
        if (key.rfind("gear", 0) == 0 && key.size() == 5 && key[4] >= '1' && key[4] <= '8') {
            fresh.gears[static_cast<std::size_t>(key[4] - '1')] = std::atof(val.c_str());
            continue;
        }
#define X(f) if (key == #f) { assign(fresh.f, val); continue; }
        DESIGN_FIELDS(X)
#undef X
    }
    clampDesign(fresh);
    d = fresh;
    return true;
}

} // namespace sim
