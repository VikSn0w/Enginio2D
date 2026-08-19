#include "ui/Editor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <filesystem>

namespace ui {
namespace {

constexpr float kPanelX = 20.0f, kPanelY = 14.0f, kPanelW = 1400.0f, kPanelH = 784.0f;
constexpr float kBodyY  = 112.0f, kBodyH = 654.0f;
constexpr float kTabX   = 38.0f, kTabW = 176.0f;
constexpr float kColA   = 244.0f, kColB = 584.0f, kColW = 312.0f;
const sf::FloatRect kBody({228.0f, kBodyY}, {700.0f, kBodyH});
const sf::FloatRect kSide({944.0f, kBodyY}, {456.0f, kBodyH});

const char* kTabNames[] = {"LAYOUT", "BOTTOM END", "HEAD & VALVES", "FUEL & SPARK",
                           "OIL & FRICTION", "INDUCTION", "EXHAUST", "DRIVETRAIN",
                           "APPEARANCE"};
constexpr int kTabCount = 9;

std::string fmt(const char* spec, double v) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), spec, v);
    return buf;
}

const char* const* presetNameList() {
    static std::vector<const char*> names;
    if (names.empty())
        for (int i = 0; i < sim::presetCount(); ++i) names.push_back(sim::presetName(i));
    return names.data();
}

std::string sanitise(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (std::isalnum(static_cast<unsigned char>(c))) out.push_back(c);
        else if (c == ' ' || c == '-' || c == '.') out.push_back('-');
    }
    if (out.empty()) out = "engine";
    return out;
}

} // namespace

void Editor::status(const std::string& s, float seconds) {
    m_status = s;
    m_statusTimer = seconds;
}

void Editor::refreshSaved() {
    m_saved.clear();
    std::error_code ec;
    std::filesystem::create_directory("designs", ec);
    for (const auto& e : std::filesystem::directory_iterator("designs", ec)) {
        if (!e.is_regular_file(ec)) continue;
        if (e.path().extension() != ".eng") continue;
        m_saved.push_back(e.path().stem().string());
    }
    std::sort(m_saved.begin(), m_saved.end());
    m_savedPtrs.clear();
    for (const std::string& s : m_saved) m_savedPtrs.push_back(s.c_str());
    if (m_savedPtrs.empty()) m_savedPtrs.push_back("(none saved yet)");
    m_savedPick = std::clamp(m_savedPick, 0, static_cast<int>(m_savedPtrs.size()) - 1);
    m_savedListed = true;
}

// ---------------------------------------------------------------------------
void Editor::tab(Ui& ui, int index, const char* name, float x, float& y) {
    const bool active = m_tab == index;
    if (ui.button(name, x, y, kTabW, 34.0f, active)) m_tab = index;
    y += 38.0f;
}

EditorResult Editor::draw(Ui& ui, sim::EngineDesign& d, sim::Dyno& dyno,
                          bool dirty, float dt) {
    EditorResult r;
    if (m_statusTimer > 0.0f) m_statusTimer -= dt;
    if (!m_savedListed) refreshSaved();

    const Palette& pal = ui.pal();
    const sim::DesignSummary sum = sim::summarise(d);

    // Scrim, so the running engine behind stays visible but does not compete.
    ui.rect(0.0f, 0.0f, 1440.0f, 812.0f, sf::Color(0, 0, 0, 175));
    ui.rect(kPanelX, kPanelY, kPanelW, kPanelH, pal.panel, pal.line, 1.0f);

    // ---- Title bar ---------------------------------------------------------
    ui.text("ENGINE EDITOR", 38.0f, 26.0f, 15, pal.accent);
    ui.column(200.0f, 22.0f, 330.0f);
    if (ui.textField("NAME", d.name, 40)) r.changed = true;
    ui.column(556.0f, 26.0f, 150.0f);
    ui.toggle("LIVE APPLY", liveApply);

    // Starting a sweep joins any run already in flight, which would stall the
    // interface for as long as that sweep had left to go.
    if (ui.button("DYNO PULL", 958.0f, 22.0f, 84.0f, 28.0f, dyno.running()) &&
        !dyno.running()) {
        dyno.start(sim::paramsFromDesign(d));
        dyno.setLabel(d.name);
        status("Dyno sweep started");
    }
    if (ui.button("SAVE", 1050.0f, 22.0f, 74.0f, 28.0f)) {
        const std::string path = "designs/" + sanitise(d.name) + ".eng";
        std::error_code ec;
        std::filesystem::create_directory("designs", ec);
        if (sim::saveDesign(d, path)) { status("Saved " + path); refreshSaved(); }
        else status("Could not write " + path);
    }
    if (ui.button("APPLY", 1132.0f, 22.0f, 96.0f, 28.0f, dirty)) r.apply = true;
    if (ui.button("REVERT", 1236.0f, 22.0f, 86.0f, 28.0f)) r.revert = true;
    if (ui.button("CLOSE", 1330.0f, 22.0f, 74.0f, 28.0f)) visible = false;

    // ---- Preset and file row ----------------------------------------------
    ui.column(38.0f, 66.0f, 400.0f);
    ui.choice("PRESET", m_preset, presetNameList(), sim::presetCount());
    if (ui.button("LOAD PRESET", 446.0f, 66.0f, 110.0f, 26.0f)) {
        d = sim::preset(m_preset);
        r.changed = true;
        r.apply = true;
        status(std::string("Loaded preset: ") + d.name);
    }
    ui.column(600.0f, 66.0f, 400.0f);
    ui.choice("SAVED", m_savedPick, m_savedPtrs.data(),
              static_cast<int>(m_savedPtrs.size()));
    if (ui.button("OPEN", 1008.0f, 66.0f, 90.0f, 26.0f)) {
        if (!m_saved.empty()) {
            const std::string path = "designs/" +
                m_saved[static_cast<std::size_t>(std::clamp(m_savedPick, 0,
                    static_cast<int>(m_saved.size()) - 1))] + ".eng";
            if (sim::loadDesign(d, path)) {
                r.changed = true;
                r.apply = true;
                status("Opened " + path);
            } else {
                status("Could not read " + path);
            }
        }
    }

    // ---- Tab rail ----------------------------------------------------------
    float ty = kBodyY;
    for (int i = 0; i < kTabCount; ++i) tab(ui, i, kTabNames[i], kTabX, ty);

    ui.rect(kTabX, ty + 12.0f, kTabW, 1.0f, pal.line);
    ui.text("SPECIFICATION", kTabX, ty + 26.0f, 11, pal.dim);
    ui.text(fmt("%.2f litre", sum.displacementL), kTabX, ty + 44.0f, 18, pal.text);
    ui.text(std::string(sim::layoutNames()[d.layout]) + "-" +
            std::to_string(d.cylinders), kTabX, ty + 70.0f, 13, pal.dim);
    ui.text(std::string(sim::fuelNames()[d.fuel]), kTabX, ty + 90.0f, 13, pal.dim);
    if (dirty)
        ui.text("EDITS NOT APPLIED", kTabX, ty + 116.0f, 12, pal.alert);

    // ---- Body --------------------------------------------------------------
    ui.rect(kBody.position.x, kBody.position.y, kBody.size.x, kBody.size.y,
            pal.bg, pal.line, 1.0f);
    const int t = std::clamp(m_tab, 0, kTabCount - 1);
    ui.beginScroll(kBody, m_scroll[t], m_contentH[t]);
    body(ui, d, r, sum, dyno);
    ui.endScroll();

    // ---- Sidebar -----------------------------------------------------------
    ui.rect(kSide.position.x, kSide.position.y, kSide.size.x, kSide.size.y,
            pal.bg, pal.line, 1.0f);
    sidebar(ui, d, sum, dyno);

    // ---- Status ------------------------------------------------------------
    if (m_statusTimer > 0.0f) ui.text(m_status, 38.0f, 772.0f, 12, pal.good);
    else ui.text("Drag a slider, or roll the wheel over any control. Hold Shift for fine steps."
                 "   TAB closes the editor.", 38.0f, 772.0f, 12, pal.dim);

    if (r.changed) sim::clampDesign(d);
    return r;
}

// ---------------------------------------------------------------------------
void Editor::body(Ui& ui, sim::EngineDesign& d, EditorResult& r,
                  const sim::DesignSummary& sum, const sim::Dyno& dyno) {
    const Palette& pal = ui.pal();
    const float top = kBodyY + 6.0f;
    auto touch = [&](bool c) { if (c) r.changed = true; };
    float endA = top, endB = top;

    switch (m_tab) {
    case 0: {   // ---------------------------------------------- LAYOUT
        ui.column(kColA, top, kColW);
        ui.heading("ARRANGEMENT");
        touch(ui.choice("CYLINDER LAYOUT", d.layout, sim::layoutNames(),
                        static_cast<int>(sim::Layout::Count)));
        touch(ui.sliderInt("CYLINDERS", d.cylinders, 1, 16, ""));
        if (d.layout == static_cast<int>(sim::Layout::Vee) ||
            d.layout == static_cast<int>(sim::Layout::W))
            touch(ui.slider("BANK ANGLE", d.bankAngle, 10.0, 120.0, "%.0f deg", 5.0));
        else
            ui.readout("BANK ANGLE", fmt("%.0f deg", d.bankAngle), pal.dim);
        touch(ui.choice("CRANK", d.crankType, sim::crankNames(),
                        static_cast<int>(sim::CrankType::Count)));
        ui.note("Crossplane suits a V8: it is what burbles.");
        ui.note("Odd fire shares crankpins, as a big twin does.");

        ui.heading("WHAT THAT GIVES");
        ui.readout("BANKS", std::to_string(sum.banks));
        ui.readout("MEAN FIRING INTERVAL", fmt("%.0f deg", sum.firingInterval));
        std::string fo;
        for (std::size_t i = 0; i < sum.firingOrder.size(); ++i)
            fo += (i ? "-" : "") + std::to_string(sum.firingOrder[i]);
        ui.readout("FIRING ORDER", fo, pal.accent);
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("WHEN EACH CYLINDER FIRES");
        firingChart(ui, sum, kColB, ui.cursorY(), kColW, 190.0f);
        ui.skip(200.0f);
        ui.note("Bars are coloured by bank. Uneven spacing");
        ui.note("within one bank is what you hear as a burble.");
        endB = ui.cursorY();
        break;
    }
    case 1: {   // ---------------------------------------------- BOTTOM END
        ui.column(kColA, top, kColW);
        ui.heading("CYLINDER");
        touch(ui.slider("BORE", d.bore, 40.0, 145.0, "%.1f mm", 0.5));
        touch(ui.slider("STROKE", d.stroke, 30.0, 145.0, "%.1f mm", 0.5));
        touch(ui.slider("ROD RATIO", d.rodRatio, 1.20, 2.40, "%.2f", 0.01));
        touch(ui.slider("COMPRESSION RATIO", d.compression, 5.0, 26.0, "%.2f : 1", 0.1));
        ui.note("Short stroke revs. Long stroke pulls low down.");
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("MOVING MASS");
        touch(ui.slider("RECIPROCATING MASS", d.recipMass, 0.08, 3.00, "%.2f kg", 0.01));
        touch(ui.slider("FLYWHEEL INERTIA", d.flywheel, 0.03, 1.20, "%.3f kg m2", 0.005));
        ui.heading("RESULT");
        ui.readout("DISPLACEMENT", fmt("%.3f litre", sum.displacementL), pal.accent);
        ui.readout("PER CYLINDER", fmt("%.0f cc", sum.displacementL * 1000.0 / d.cylinders));
        ui.readout("BORE / STROKE", fmt("%.2f", sum.boreStroke),
                   sum.boreStroke >= 1.0 ? pal.good : pal.text);
        ui.readout("ROD LENGTH", fmt("%.1f mm", sum.rodLengthMm));
        const double sp = 2.0 * d.stroke * 1e-3 * d.redline / 60.0;
        ui.readout("PISTON SPEED AT REDLINE", fmt("%.1f m/s", sp),
                   sp > 25.0 ? pal.alert : (sp > 21.0 ? pal.accent : pal.good));
        ui.note(sp > 25.0 ? "Above about 25 m/s nothing lasts very long."
                          : "Production engines sit near 20 m/s at peak.");
        endB = ui.cursorY();
        break;
    }
    case 2: {   // ---------------------------------------------- HEAD & VALVES
        ui.column(kColA, top, kColW);
        ui.heading("VALVETRAIN");
        touch(ui.choice("TYPE", d.valvetrain, sim::valvetrainNames(),
                        static_cast<int>(sim::Valvetrain::Count)));
        touch(ui.choice("VALVE MATERIAL", d.valveMaterial, sim::valveMetalNames(),
                        static_cast<int>(sim::ValveMetal::Count)));
        touch(ui.sliderInt("INTAKE VALVES", d.intakeValves, 1, 3, " per cyl"));
        touch(ui.sliderInt("EXHAUST VALVES", d.exhaustValves, 1, 3, " per cyl"));
        const double vmax = sim::maxValveFraction(d.intakeValves, d.exhaustValves);
        touch(ui.slider("INTAKE VALVE SIZE", d.intakeValveFrac, 0.15, vmax,
                        "%.3f x bore", 0.005));
        touch(ui.slider("EXHAUST VALVE SIZE", d.exhaustValveFrac, 0.13, vmax,
                        "%.3f x bore", 0.005));
        touch(ui.slider("LIFT RATIO", d.liftRatio, 0.10, 0.42, "%.3f x dia", 0.005));
        ui.note(fmt("More valves, smaller ones: cap %.3f of bore.", vmax));
        touch(ui.choice("PORT WORK", d.portWork, sim::portNames(),
                        static_cast<int>(sim::PortWork::Count)));
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("CAMSHAFT");
        touch(ui.choice("PROFILE", d.camProfile, sim::camNames(),
                        static_cast<int>(sim::CamGrind::Count)));
        touch(ui.slider("INTAKE DURATION", d.intakeDuration, 140.0, 330.0, "%.0f deg", 2.0));
        touch(ui.slider("INTAKE CENTRELINE", d.intakeCentre, 80.0, 135.0,
                        "%.1f deg ATDC", 0.5));
        touch(ui.slider("EXHAUST DURATION", d.exhaustDuration, 140.0, 330.0, "%.0f deg", 2.0));
        touch(ui.slider("EXHAUST CENTRELINE", d.exhaustCentre, 80.0, 135.0,
                        "%.1f deg BTDC", 0.5));
        touch(ui.toggle("VARIABLE CAM TIMING", d.vvt));
        if (d.vvt) touch(ui.slider("PHASER RANGE", d.vvtRange, 0.0, 45.0, "%.0f deg", 1.0));

        ui.heading("RESULT");
        ui.readout("VALVE HEADS", fmt("%.1f", sum.intakeValveMm) +
                                  fmt(" / %.1f mm", sum.exhaustValveMm));
        ui.readout("MAX LIFT", fmt("%.2f", sum.intakeLiftMm) +
                               fmt(" / %.2f mm", sum.exhaustLiftMm));
        ui.readout("LOBE SEPARATION", fmt("%.1f deg", sum.lsa));
        ui.readout("OVERLAP", fmt("%.1f deg", sum.overlap),
                   sum.overlap > 60.0 ? pal.accent : pal.text);
        ui.readout("IVO / IVC", fmt("%.0f BTDC", sum.ivo) + fmt(" / %.0f ABDC", sum.ivc));
        ui.readout("EVO / EVC", fmt("%.0f BBDC", sum.evo) + fmt(" / %.0f ATDC", sum.evc));
        ui.readout("VALVE FLOAT", fmt("%.0f rpm", sum.valveFloatRpm),
                   sum.valveFloatRpm < d.redline ? pal.alert : pal.good);
        if (sum.valveFloatRpm < d.redline)
            ui.note("Valves stop following the cam below the limiter.");
        endB = ui.cursorY();
        break;
    }
    case 3: {   // ---------------------------------------------- FUEL & SPARK
        ui.column(kColA, top, kColW);
        ui.heading("FUEL");
        touch(ui.choice("TYPE", d.fuel, sim::fuelNames(),
                        static_cast<int>(sim::FuelKind::Count)));
        ui.readout("STOICHIOMETRIC AFR", fmt("%.2f : 1", sum.stoichAfr));
        ui.readout("ENERGY", fmt("%.1f MJ/kg", sum.fuelLhvMJ));
        const bool ci = d.fuel == static_cast<int>(sim::FuelKind::Diesel);
        if (ci) {
            ui.note("Compression ignition: no throttle, no spark.");
            ui.note("The pedal meters fuel; the charge self-ignites.");
        }
        ui.heading("MIXTURE");
        touch(ui.slider("LAMBDA, LIGHT LOAD", d.lambdaCruise, 0.65, 1.60, "%.2f", 0.01));
        touch(ui.slider("LAMBDA, FULL LOAD", d.lambdaPower, 0.60, 1.40, "%.2f", 0.01));
        ui.readout("AFR LIGHT / FULL", fmt("%.1f", d.lambdaCruise * sum.stoichAfr) +
                                       fmt(" / %.1f : 1", d.lambdaPower * sum.stoichAfr));
        ui.note("Below 1.00 is rich. Enrichment at full load");
        ui.note("holds peak temperature and detonation down.");
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading(ci ? "INJECTION" : "IGNITION");
        touch(ui.slider(ci ? "INJECTION AT IDLE" : "ADVANCE AT IDLE",
                        d.sparkIdle, -5.0, 50.0, "%.1f deg BTDC", 0.5));
        touch(ui.slider(ci ? "INJECTION AT SPEED" : "ADVANCE AT SPEED",
                        d.sparkPeak, -5.0, 60.0, "%.1f deg BTDC", 0.5));
        touch(ui.slider("EXTRA ON LIGHT LOAD", d.sparkPartLoad, 0.0, 30.0, "%.1f deg", 0.5));
        touch(ui.slider("BURN DURATION", d.burnDuration, 15.0, 120.0, "%.0f deg", 1.0));
        touch(ui.slider("IGNITION DELAY", d.ignitionDelay, 0.0, 30.0, "%.1f deg", 0.5));
        touch(ui.slider("COMBUSTION EFFICIENCY", d.combustionEff, 0.60, 1.00, "%.2f", 0.01));
        if (!ci) {
            touch(ui.toggle("KNOCK CONTROL", d.knockControl));
            ui.note(d.knockControl ? "The ECU pulls timing when it hears detonation."
                                   : "No knock sensor: raise compression carefully.");
        }
        ui.heading("LIMITS");
        touch(ui.slider("REDLINE", d.redline, 1500.0, 22000.0, "%.0f rpm", 100.0));
        touch(ui.slider("IDLE SPEED", d.idleRpm, 350.0, 3000.0, "%.0f rpm", 25.0));
        endB = ui.cursorY();
        break;
    }
    case 4: {   // ---------------------------------------------- OIL & FRICTION
        ui.column(kColA, top, kColW);
        ui.heading("OIL");
        touch(ui.choice("GRADE", d.oilGrade, sim::oilNames(),
                        static_cast<int>(sim::OilGrade::Count)));
        touch(ui.slider("RUNNING TEMPERATURE", d.oilTempTarget, 60.0, 150.0, "%.0f C", 1.0));
        touch(ui.slider("TEMPERATURE AT START", d.oilStartTemp, -30.0, 120.0, "%.0f C", 1.0));
        ui.note("Thick oil costs power and builds pressure;");
        ui.note("thin oil gives it back, and runs out when hot.");
        ui.note("Start cold: friction falls as the oil warms.");
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("MECHANICAL LOSSES");
        touch(ui.slider("FRICTION SCALE", d.frictionScale, 0.40, 2.50, "%.2f x", 0.05));
        touch(ui.slider("ACCESSORY DRAG", d.accessoryLoad, 0.0, 60.0, "%.1f N m", 0.5));
        ui.note("Alternator, pumps, steering, air conditioning.");
        ui.heading("VALVETRAIN DRAG");
        ui.readout("FROM", sim::valvetrainNames()[d.valvetrain], pal.text);
        ui.note("A pushrod head drags more than a bucket;");
        ui.note("a desmo drags more again, and never floats.");
        endB = ui.cursorY();
        break;
    }
    case 5: {   // ---------------------------------------------- INDUCTION
        ui.column(kColA, top, kColW);
        ui.heading("MANIFOLD");
        touch(ui.slider("THROTTLE BORE", d.throttleBore, 20.0, 140.0, "%.0f mm", 1.0));
        touch(ui.slider("PLENUM VOLUME", d.plenumVolume, 0.15, 20.0, "%.2f litre", 0.05));
        touch(ui.slider("RUNNER LENGTH", d.runnerLength, 60.0, 900.0, "%.0f mm", 5.0));
        touch(ui.slider("RUNNER DIAMETER", d.runnerDia, 15.0, 90.0, "%.1f mm", 0.5));
        ui.readout("RAM TUNED NEAR", fmt("%.0f rpm", sum.tunedRpmIntake), pal.accent);
        ui.note("Long runners fill low down, short ones high up.");
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("FORCED INDUCTION");
        touch(ui.choice("TYPE", d.charger, sim::chargerNames(),
                        static_cast<int>(sim::ChargerKind::Count)));
        if (d.charger != static_cast<int>(sim::ChargerKind::None)) {
            touch(ui.slider("BOOST", d.boost, 0.05, 4.0, "%.2f bar", 0.05));
            touch(ui.slider("ON SONG FROM", d.spoolRpm, 800.0, 9000.0, "%.0f rpm", 100.0));
            if (d.charger == static_cast<int>(sim::ChargerKind::Turbo))
                touch(ui.slider("SPOOL TIME", d.turboLag, 0.05, 4.0, "%.2f s", 0.05));
            touch(ui.slider("INTERCOOLER", d.intercooler, 0.0, 0.95, "%.2f", 0.05));
            ui.note(d.charger == static_cast<int>(sim::ChargerKind::Turbo)
                    ? "A turbine is free to spin, but costs backpressure."
                    : "A blower is instant; the crank pays for it.");
            if (d.boost > 1.0 && d.compression > 11.0)
                ui.note("That boost on that compression will detonate.");
        } else {
            ui.note("Naturally aspirated: it all comes from the cam,");
            ui.note("the ports and the pipes.");
        }
        endB = ui.cursorY();
        break;
    }
    case 6: {   // ---------------------------------------------- EXHAUST
        ui.column(kColA, top, kColW);
        ui.heading("HEADER");
        touch(ui.choice("STYLE", d.header, sim::headerNames(),
                        static_cast<int>(sim::HeaderStyle::Count)));
        touch(ui.slider("PRIMARY LENGTH", d.primaryLength, 80.0, 1200.0, "%.0f mm", 10.0));
        touch(ui.slider("PRIMARY DIAMETER", d.primaryDia, 15.0, 90.0, "%.1f mm", 0.5));
        touch(ui.slider("COLLECTOR VOLUME", d.collectorVol, 0.2, 25.0, "%.2f litre", 0.1));
        ui.readout("SCAVENGE TUNED NEAR", fmt("%.0f rpm", sum.tunedRpmExhaust), pal.accent);
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("SILENCING");
        touch(ui.choice("MUFFLER", d.muffler, sim::mufflerNames(),
                        static_cast<int>(sim::MufflerKind::Count)));
        ui.note("A muffler restricts as well as silences, so a");
        ui.note("quiet one really does cost power at the top.");
        ui.heading("HOW THIS IS HEARD");
        ui.note("Each cylinder gets its own primary pipe, and");
        ui.note("they are summed per bank - so header style");
        ui.note("and firing order are audible, not just numbers.");
        endB = ui.cursorY();
        break;
    }
    case 7: {   // ---------------------------------------------- DRIVETRAIN
        ui.column(kColA, top, kColW);
        ui.heading("GEARBOX");
        touch(ui.sliderInt("GEARS", d.gearCount, 1, 8, ""));
        for (int i = 0; i < d.gearCount; ++i) {
            char label[24];
            std::snprintf(label, sizeof(label), "GEAR %d", i + 1);
            touch(ui.slider(label, d.gears[static_cast<std::size_t>(i)], 0.30, 6.50,
                            "%.3f : 1", 0.01));
        }
        touch(ui.slider("FINAL DRIVE", d.finalDrive, 1.50, 7.00, "%.2f : 1", 0.05));
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("VEHICLE");
        touch(ui.slider("MASS", d.vehicleMass, 120.0, 8000.0, "%.0f kg", 10.0));
        touch(ui.slider("DRAG AREA Cd x A", d.dragArea, 0.10, 3.00, "%.2f m2", 0.01));
        touch(ui.slider("WHEEL RADIUS", d.wheelRadius, 0.15, 0.75, "%.3f m", 0.005));
        touch(ui.slider("CLUTCH CAPACITY", d.clutchCapacity, 40.0, 3000.0, "%.0f N m", 10.0));
        touch(ui.slider("BRAKE TORQUE", d.brakeTorque, 200.0, 12000.0, "%.0f N m", 50.0));
        ui.heading("AT THE LIMITER");
        for (int i = 0; i < d.gearCount; ++i) {
            const double ratio = d.gears[static_cast<std::size_t>(i)] * d.finalDrive;
            const double kph = d.redline * 2.0 * 3.14159265 / 60.0 * d.wheelRadius /
                               std::max(ratio, 1e-6) * 3.6;
            ui.readout("GEAR " + std::to_string(i + 1), fmt("%.0f km/h", kph));
        }
        // A gear can be as tall as you like; what the car will actually do is
        // set by power against drag. Without this the top-gear figure reads as
        // a top speed, which it is not.
        {
            const double kw = dyno.peakPowerKw();
            double v = 0.0;
            if (kw > 1.0) {
                // Solve P*eta = 0.5 rho Cd A v^3 + Crr m g v by bisection.
                double lo = 1.0, hi = 200.0;
                for (int it = 0; it < 60; ++it) {
                    v = 0.5 * (lo + hi);
                    const double need = 0.5 * 1.2 * d.dragArea * v * v * v +
                                        0.013 * d.vehicleMass * 9.81 * v;
                    (need > kw * 1000.0 * 0.85 ? hi : lo) = v;
                }
            }
            ui.readout("TOP SPEED, DRAG LIMITED",
                       kw > 1.0 ? fmt("%.0f km/h", v * 3.6)
                                : std::string("run the dyno"), pal.accent);
            ui.note("A taller top gear than this is an overdrive, not");
            ui.note("more speed.");
        }
        endB = ui.cursorY();
        break;
    }
    default: {  // ---------------------------------------------- APPEARANCE
        ui.column(kColA, top, kColW);
        ui.heading("INTERFACE");
        ui.choice("THEME", d.theme, sim::themeNames(),
                  static_cast<int>(sim::ThemeKind::Count));
        ui.slider("ACCENT HUE", d.accentHue, 0.0, 360.0, "%.0f deg", 1.0);
        ui.rect(kColA, ui.cursorY(), kColW, 14.0f, hsl(d.accentHue, 0.90, 0.590));
        ui.skip(24.0f);
        ui.slider("BLOCK SHADE", d.blockShade, 5.0, 90.0, "%.0f %%", 1.0);
        ui.rect(kColA, ui.cursorY(), kColW, 14.0f,
                hsl(220.0, 0.10, std::clamp(d.blockShade * 0.005, 0.03, 0.45)));
        ui.skip(24.0f);
        endA = ui.cursorY();

        ui.column(kColB, top, kColW);
        ui.heading("ENGINE FINISH");
        ui.slider("CAM COVER HUE", d.coverHue, 0.0, 360.0, "%.0f deg", 1.0);
        ui.slider("CAM COVER SATURATION", d.coverSat, 0.0, 100.0, "%.0f %%", 1.0);
        ui.rect(kColB, ui.cursorY(), kColW, 20.0f, hsl(d.coverHue, d.coverSat * 0.01, 0.42));
        ui.skip(30.0f);
        ui.heading("VIEWS");
        ui.toggle("CYLINDER CUTAWAY", d.showCutaway);
        ui.toggle("ENGINE FROM ABOVE", d.showTopView);
        ui.note("Appearance applies at once; it is not physics.");
        endB = ui.cursorY();
        break;
    }
    }

    m_contentH[std::clamp(m_tab, 0, kTabCount - 1)] =
        std::max(endA, endB) - kBodyY + 24.0f;
}

// ---------------------------------------------------------------------------
void Editor::firingChart(Ui& ui, const sim::DesignSummary& sum, float x, float y,
                         float w, float h) {
    const Palette& pal = ui.pal();
    const float trackY = y + 26.0f;
    const float trackH = h - 54.0f;
    ui.rect(x, trackY, w, trackH, pal.panel, pal.line, 1.0f);
    for (int deg = 0; deg <= 720; deg += 180) {
        const float px = x + w * deg / 720.0f;
        ui.line({px, trackY}, {px, trackY + trackH}, pal.grid);
        ui.centred(std::to_string(deg), px, trackY + trackH + 4.0f, 11, pal.dim);
    }

    static const sf::Color kBankTint[4] = {
        sf::Color(120, 180, 255), sf::Color(255, 150, 110),
        sf::Color(150, 230, 160), sf::Color(230, 170, 240)};

    const int n = static_cast<int>(sum.phases.size());
    for (int i = 0; i < n; ++i) {
        const float px = x + w * static_cast<float>(sum.phases[static_cast<std::size_t>(i)]) / 720.0f;
        // Cylinders alternate banks, so the tint can be read straight off the
        // index the same way the model assigns it.
        const int bank = sum.banks > 1 ? i % sum.banks : 0;
        const float lane = trackY + 6.0f + (trackH - 20.0f) *
                           (sum.banks > 1 ? static_cast<float>(bank) / (sum.banks - 1) : 0.5f) *
                           0.75f;
        ui.rect(px - 1.5f, trackY + 2.0f, 3.0f, trackH - 4.0f,
                sf::Color(kBankTint[bank % 4].r, kBankTint[bank % 4].g,
                          kBankTint[bank % 4].b, 90));
        ui.rect(px - 8.0f, lane, 16.0f, 15.0f, kBankTint[bank % 4]);
        ui.centred(std::to_string(i + 1), px, lane - 1.0f, 11, sf::Color(18, 20, 26));
    }
    ui.text("BANK A", x, y + 4.0f, 11, kBankTint[0]);
    if (sum.banks > 1) ui.text("BANK B", x + 56.0f, y + 4.0f, 11, kBankTint[1]);
    if (sum.banks > 2) ui.text("C / D", x + 112.0f, y + 4.0f, 11, kBankTint[2]);
}

// ---------------------------------------------------------------------------
void Editor::dynoChart(Ui& ui, sim::Dyno& dyno, float x, float y, float w, float h) {
    const Palette& pal = ui.pal();
    ui.rect(x, y, w, h, pal.panel, pal.line, 1.0f);

    const int n = dyno.count();
    if (n < 2) {
        ui.centred(dyno.running() ? "measuring..." : "no sweep yet - press DYNO PULL",
                   x + w * 0.5f, y + h * 0.5f - 8.0f, 13, pal.dim);
        return;
    }

    float rpmLo = dyno.point(0).rpm, rpmHi = dyno.point(n - 1).rpm;
    float maxT = 1.0f, maxP = 1.0f;
    for (int i = 0; i < n; ++i) {
        maxT = std::max(maxT, dyno.point(i).torque);
        maxP = std::max(maxP, dyno.point(i).power);
    }
    maxT *= 1.18f; maxP *= 1.18f;
    if (rpmHi <= rpmLo) rpmHi = rpmLo + 1.0f;

    const float x0 = x + 40.0f, x1 = x + w - 40.0f;
    const float yb = y + h - 22.0f, yt = y + 16.0f;
    auto px = [&](float rpm) { return x0 + (x1 - x0) * (rpm - rpmLo) / (rpmHi - rpmLo); };

    for (int g = 0; g <= 4; ++g) {
        const float gy = yb - (yb - yt) * g / 4.0f;
        ui.line({x0, gy}, {x1, gy}, pal.grid);
        ui.right(fmt("%.0f", maxT * g / 4.0f), x0 - 4.0f, gy - 8.0f, 10, pal.dim);
        ui.text(fmt("%.0f", maxP * g / 4.0f), x1 + 4.0f, gy - 8.0f, 10, pal.dim);
    }
    for (int g = 0; g <= 4; ++g) {
        const float rpm = rpmLo + (rpmHi - rpmLo) * g / 4.0f;
        ui.line({px(rpm), yt}, {px(rpm), yb}, pal.grid);
        ui.centred(fmt("%.0f", rpm), px(rpm), yb + 3.0f, 10, pal.dim);
    }

    // Torque in the accent colour, power in the intake blue, each against its
    // own scale - so the axis labels on either side are the ones that matter.
    auto curve = [&](bool power, sf::Color col) {
        sf::VertexArray v(sf::PrimitiveType::LineStrip);
        for (int i = 0; i < n; ++i) {
            const sim::DynoPoint p = dyno.point(i);
            const float val = power ? p.power / maxP : p.torque / maxT;
            sf::Vertex vert;
            vert.position = {px(p.rpm), yb - (yb - yt) * std::clamp(val, 0.0f, 1.0f)};
            vert.color = col;
            v.append(vert);
        }
        ui.vertices(v);
    };
    ui.line({x0, yb}, {x1, yb}, pal.line);
    curve(false, pal.accent);
    curve(true, pal.intake);

    ui.text("N m", x + 6.0f, y + 2.0f, 10, pal.accent);
    ui.right("kW", x + w - 6.0f, y + 2.0f, 10, pal.intake);
}

// ---------------------------------------------------------------------------
void Editor::sidebar(Ui& ui, const sim::EngineDesign& d, const sim::DesignSummary& sum,
                     sim::Dyno& dyno) {
    const Palette& pal = ui.pal();
    const float x = kSide.position.x + 16.0f;
    const float w = kSide.size.x - 32.0f;

    ui.column(x, kSide.position.y + 10.0f, w);
    ui.text("DYNAMOMETER", x, kSide.position.y + 12.0f, 13, pal.accent);
    ui.right(dyno.running() ? fmt("%.0f %%", dyno.progress() * 100.0f)
                            : (dyno.label().empty() ? std::string() : dyno.label()),
             x + w, kSide.position.y + 12.0f, 12, pal.dim);

    dynoChart(ui, dyno, x, kSide.position.y + 34.0f, w, 208.0f);

    ui.column(x, kSide.position.y + 250.0f, w);
    ui.readout("PEAK POWER", fmt("%.1f kW", dyno.peakPowerKw()) +
                             fmt(" (%.0f hp)", dyno.peakPowerKw() * 1.34102) +
                             fmt(" at %.0f", dyno.peakPowerRpm()), pal.intake);
    ui.readout("PEAK TORQUE", fmt("%.1f N m", dyno.peakTorqueNm()) +
                              fmt(" at %.0f rpm", dyno.peakTorqueRpm()), pal.accent);
    ui.readout("SPECIFIC OUTPUT",
               sum.displacementL > 0.01
                   ? fmt("%.1f kW / litre", dyno.peakPowerKw() / sum.displacementL)
                   : std::string("-"));

    ui.heading("SPECIFICATION");
    ui.readout("DISPLACEMENT", fmt("%.3f litre", sum.displacementL));
    ui.readout("BORE x STROKE", fmt("%.1f", d.bore) + fmt(" x %.1f mm", d.stroke));
    ui.readout("COMPRESSION", fmt("%.2f : 1", d.compression));
    ui.readout("VALVES", std::to_string(d.intakeValves + d.exhaustValves) +
                         " per cylinder, " + sim::valvetrainNames()[d.valvetrain]);
    ui.readout("CAM", fmt("%.0f", d.intakeDuration) + fmt(" / %.0f deg", d.exhaustDuration) +
                      fmt(", %.0f LSA", sum.lsa));
    ui.readout("OVERLAP", fmt("%.1f deg", sum.overlap));
    ui.readout("INDUCTION", d.charger == 0
                   ? std::string("naturally aspirated")
                   : std::string(sim::chargerNames()[d.charger]) + fmt(", %.2f bar", d.boost));
    ui.readout("FUEL", sim::fuelNames()[d.fuel]);
    ui.readout("OIL", std::string(sim::oilNames()[d.oilGrade]) +
                      fmt(", %.0f C", d.oilTempTarget));
    ui.readout("EXHAUST", std::string(sim::headerNames()[d.header]) + ", " +
                          sim::mufflerNames()[d.muffler]);
    ui.readout("RAM / SCAVENGE TUNED", fmt("%.0f", sum.tunedRpmIntake) +
                                       fmt(" / %.0f rpm", sum.tunedRpmExhaust));
    ui.readout("VALVE FLOAT", fmt("%.0f rpm", sum.valveFloatRpm),
               sum.valveFloatRpm < d.redline ? pal.alert : pal.good);
    ui.readout("REDLINE", fmt("%.0f rpm", d.redline));
}

} // namespace ui
