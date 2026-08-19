#pragma once
#include "sim/Drivetrain.h"
#include "sim/Engine.h"

#include <array>
#include <string>
#include <vector>

namespace sim {

// ---------------------------------------------------------------------------
// The specification a person edits, as opposed to the numbers the solver runs
// on. Everything here is in the units an engine is actually described in -
// millimetres, litres, bar, degrees, degrees Celsius - and paramsFromDesign()
// turns it into the SI quantities EngineParams wants, filling in the
// consequences: firing order from the layout, valve size from the bore, float
// speed from the valvetrain, and so on.
// ---------------------------------------------------------------------------

enum class Layout      { Inline, Vee, Flat, W, Count };
enum class CrankType   { EvenFire, Crossplane, OddFire, Count };
enum class Valvetrain  { OHV, SOHC, DOHC, Desmo, Pneumatic, Count };
enum class ValveMetal  { Steel, Sodium, Titanium, Ceramic, Count };
enum class CamGrind    { Stock, Street, Sport, Race, Count };
enum class PortWork    { AsCast, Ported, CNC, Count };
enum class FuelKind    { Petrol91, Petrol95, Petrol98, RaceFuel, E85, Ethanol,
                         Methanol, LPG, CNG, Diesel, Nitro, Hydrogen, Count };
enum class OilGrade    { W0_20, W5_30, W10_40, W15_50, W20_60, RaceSynthetic, Count };
enum class ChargerKind { None, Turbo, Roots, Centrifugal, Count };
enum class HeaderStyle { LogManifold, FourIntoOne, TriY, PerBank, Open, Count };
enum class MufflerKind { Straight, Chambered, Absorption, Quiet, Count };
enum class ThemeKind   { Graphite, Midnight, Slate, Blueprint, Count };

const char* const* layoutNames();
const char* const* crankNames();
const char* const* valvetrainNames();
const char* const* valveMetalNames();
const char* const* camNames();
const char* const* portNames();
const char* const* fuelNames();
const char* const* oilNames();
const char* const* chargerNames();
const char* const* headerNames();
const char* const* mufflerNames();
const char* const* themeNames();

struct EngineDesign {
    std::string name = "Stock 2.0 Inline-4";

    // ---- Layout ------------------------------------------------------------
    int    layout      = static_cast<int>(Layout::Inline);
    int    cylinders   = 4;
    double bankAngle   = 90.0;    // deg, included angle between banks
    int    crankType   = static_cast<int>(CrankType::EvenFire);

    // ---- Bottom end --------------------------------------------------------
    double bore        = 86.0;    // mm
    double stroke      = 86.0;    // mm
    double rodRatio    = 1.69;    // rod length / stroke
    double compression = 10.5;
    double recipMass   = 0.55;    // kg per cylinder, piston + rings + small end
    double flywheel    = 0.18;    // kg m^2, everything turning with the crank

    // ---- Head and valvetrain ----------------------------------------------
    int    valvetrain      = static_cast<int>(Valvetrain::DOHC);
    int    intakeValves    = 2;
    int    exhaustValves   = 2;
    int    valveMaterial   = static_cast<int>(ValveMetal::Steel);
    double intakeValveFrac = 0.37;   // valve head diameter as a fraction of bore
    double exhaustValveFrac= 0.33;
    double liftRatio       = 0.31;   // maximum lift / valve diameter
    double intakeDuration  = 230.0;  // deg at the seat
    double intakeCentre    = 105.0;  // intake centreline, deg ATDC
    double exhaustDuration = 245.0;
    double exhaustCentre   = 107.5;  // exhaust centreline, deg BTDC
    int    camProfile      = static_cast<int>(CamGrind::Street);
    int    portWork        = static_cast<int>(PortWork::AsCast);
    bool   vvt             = false;
    double vvtRange        = 25.0;   // deg of phaser authority

    // ---- Fuel and combustion ----------------------------------------------
    int    fuel          = static_cast<int>(FuelKind::Petrol95);
    double lambdaCruise  = 1.00;
    double lambdaPower   = 0.87;
    double sparkIdle     = 14.0;   // deg BTDC
    double sparkPeak     = 34.0;
    double sparkPartLoad = 12.0;
    double burnDuration  = 46.0;   // deg
    double ignitionDelay = 8.0;    // deg
    bool   knockControl  = true;
    double combustionEff = 0.97;
    double redline       = 7600.0; // rpm
    double idleRpm       = 850.0;

    // ---- Oil and friction --------------------------------------------------
    int    oilGrade      = static_cast<int>(OilGrade::W5_30);
    double oilTempTarget = 95.0;   // deg C once warm
    double oilStartTemp  = 20.0;   // deg C at key-on
    double frictionScale = 1.00;   // overall bearing/ring friction multiplier
    double accessoryLoad = 8.0;    // N m of alternator, pumps, aircon

    // ---- Induction ---------------------------------------------------------
    double throttleBore  = 52.0;   // mm
    double plenumVolume  = 2.2;    // litres
    double runnerLength  = 300.0;  // mm
    double runnerDia     = 37.4;   // mm
    int    charger       = static_cast<int>(ChargerKind::None);
    double boost         = 0.0;    // bar gauge
    double spoolRpm      = 3000.0;
    double turboLag      = 0.9;    // s
    double intercooler   = 0.70;   // effectiveness, 0..0.95

    // ---- Exhaust -----------------------------------------------------------
    double primaryLength = 420.0;  // mm
    double primaryDia    = 33.8;   // mm
    double collectorVol  = 1.6;    // litres
    int    header        = static_cast<int>(HeaderStyle::FourIntoOne);
    int    muffler       = static_cast<int>(MufflerKind::Absorption);

    // ---- Drivetrain --------------------------------------------------------
    int    gearCount      = 6;
    std::array<double, 8> gears{3.55, 2.10, 1.45, 1.08, 0.88, 0.72, 0.62, 0.56};
    double finalDrive     = 3.90;
    double wheelRadius    = 0.31;   // m
    double vehicleMass    = 1250.0; // kg
    double dragArea       = 0.62;   // Cd * A
    double clutchCapacity = 340.0;  // N m
    double brakeTorque    = 2600.0; // N m at the wheels

    // ---- Appearance --------------------------------------------------------
    int    theme       = static_cast<int>(ThemeKind::Graphite);
    double accentHue   = 26.0;    // deg
    double coverHue    = 210.0;
    double coverSat    = 18.0;    // %
    double blockShade  = 34.0;    // %
    bool   showCutaway = true;
    bool   showTopView = true;
};

// Derived numbers the editor shows back to you as you change things.
struct DesignSummary {
    double displacementL = 0.0;
    double boreStroke    = 0.0;   // ratio, >1 is oversquare
    double rodLengthMm   = 0.0;
    double intakeValveMm = 0.0;
    double exhaustValveMm= 0.0;
    double intakeLiftMm  = 0.0;
    double exhaustLiftMm = 0.0;
    double lsa           = 0.0;   // lobe separation angle
    double overlap       = 0.0;   // deg both valves open
    double ivo = 0.0, ivc = 0.0, evo = 0.0, evc = 0.0;
    double tunedRpmIntake  = 0.0; // where the intake runner's quarter wave helps
    double tunedRpmExhaust = 0.0;
    double firingInterval  = 0.0; // deg between events, mean
    double valveFloatRpm   = 0.0;
    double stoichAfr       = 0.0;
    double fuelLhvMJ       = 0.0;
    int    banks           = 1;
    std::vector<int>    firingOrder;   // 1-based cylinder numbers
    std::vector<double> phases;        // cycle angle of each cylinder
};

// Hard limits, applied in place. The editor calls this after every change so a
// design can never be left describing something that cannot exist - three
// intake valves that do not fit the bore, a pushrod head at 14000 rpm.
void clampDesign(EngineDesign& d);

EngineParams     paramsFromDesign(const EngineDesign& d);
DrivetrainParams drivetrainFromDesign(const EngineDesign& d);
DesignSummary    summarise(const EngineDesign& d);

// Largest valve head that will fit, as a fraction of the bore, for the number
// of valves asked for.
double maxValveFraction(int intakeValves, int exhaustValves);

// Presets. Index 0 is the design the program starts with.
int                 presetCount();
const char*         presetName(int i);
EngineDesign        preset(int i);

bool saveDesign(const EngineDesign& d, const std::string& path);
bool loadDesign(EngineDesign& d, const std::string& path);

} // namespace sim
