#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "audio/EngineSound.h"
#include "sim/Dyno.h"
#include "sim/Engine.h"
#include "sim/EngineDesign.h"
#include "ui/Editor.h"
#include "ui/Widgets.h"

namespace {

// The whole interface is laid out in this fixed space and letterboxed into
// whatever the window happens to be, so resizing never breaks the layout.
constexpr float kW = 1440.0f;
constexpr float kH = 812.0f;
constexpr float kPi = 3.14159265358979323846f;

std::string fmt(const char* spec, float v) {
    char buf[80];
    std::snprintf(buf, sizeof(buf), spec, v);
    return buf;
}

sf::Font loadFont() {
    sf::Font font;
    for (const char* path : {"C:/Windows/Fonts/consola.ttf",
                             "C:/Windows/Fonts/segoeui.ttf",
                             "C:/Windows/Fonts/arial.ttf"}) {
        if (font.openFromFile(path)) break;
    }
    return font;
}

// Keep the design space centred and un-stretched inside the window.
sf::View fittedView(sf::Vector2u windowSize) {
    sf::View view(sf::FloatRect({0.0f, 0.0f}, {kW, kH}));
    const float wanted = kW / kH;
    const float actual = static_cast<float>(windowSize.x) /
                         std::max(1.0f, static_cast<float>(windowSize.y));
    sf::Vector2f size{1.0f, 1.0f}, pos{0.0f, 0.0f};
    if (actual > wanted) { size.x = wanted / actual; pos.x = (1.0f - size.x) * 0.5f; }
    else                 { size.y = actual / wanted; pos.y = (1.0f - size.y) * 0.5f; }
    view.setViewport(sf::FloatRect(pos, size));
    return view;
}

// Glow colour for the charge: blue when cold, orange-white at flame temperature.
sf::Color chargeColour(float tempK) {
    const float t = std::clamp((tempK - 320.0f) / 2200.0f, 0.0f, 1.0f);
    return sf::Color(static_cast<std::uint8_t>(60 + 195 * t),
                     static_cast<std::uint8_t>(90 + 110 * t * t),
                     static_cast<std::uint8_t>(180 - 130 * t),
                     255);
}

const char* strokeName(float phase) {
    return phase < 180.0f ? "POWER"
         : phase < 360.0f ? "EXHAUST"
         : phase < 540.0f ? "INTAKE"
                          : "COMPRESSION";
}

void panel(ui::Ui& u, float x, float y, float w, float h) {
    u.rect(x, y, w, h, u.pal().panel, u.pal().line, 1.0f);
}

// ---------------------------------------------------------------------------
// Side cutaway of one cylinder. Scaled so any bore and stroke fills the panel.
// ---------------------------------------------------------------------------
void drawCylinderSide(ui::Ui& u, const sim::EngineParams& p, const sim::CylinderView& c,
                      float ox, float topY, float panelH) {
    const ui::Palette& pal = u.pal();
    const float travel = static_cast<float>(p.rodLength + 0.5 * p.stroke);
    const float scale  = std::clamp((panelH - 120.0f) / (travel * 1.6f), 300.0f, 1600.0f);
    const float a      = 0.5f * static_cast<float>(p.stroke) * scale;
    const float rod    = static_cast<float>(p.rodLength) * scale;
    const float bore   = static_cast<float>(p.bore) * scale;
    const float crankY = topY + (rod + a) + 24.0f;
    const float theta  = c.phase * kPi / 180.0f;

    const float pin  = a * std::cos(theta) +
                       std::sqrt(std::max(1.0f, rod * rod - a * a * std::sin(theta) * std::sin(theta)));
    const float pinY = crankY - pin;
    const float pistonH = std::max(14.0f, bore * 0.34f);
    // The gudgeon pin sits below the crown, so the deck has to clear the crown,
    // not the pin - otherwise the piston comes through the head at top dead
    // centre.
    const float deckY = crankY - (rod + a) - pistonH * 0.62f - 4.0f;

    const float wallH = crankY - deckY;
    u.rect(ox - bore * 0.5f - 6.0f, deckY, 6.0f, wallH, pal.block);
    u.rect(ox + bore * 0.5f, deckY, 6.0f, wallH, pal.block);
    u.rect(ox - bore * 0.5f - 6.0f, deckY - 18.0f, bore + 12.0f, 18.0f, pal.cover);

    const float chamberH = std::max(2.0f, pinY - pistonH * 0.62f - deckY);
    sf::Color gc = chargeColour(c.temperature);
    gc.a = static_cast<std::uint8_t>(std::clamp(70.0f + c.pressure * 3.2f, 70.0f, 245.0f));
    u.rect(ox - bore * 0.5f, deckY, bore, chamberH, gc);
    if (c.knock > 0.02f)
        u.rect(ox - bore * 0.5f, deckY, bore, chamberH,
               sf::Color(255, 255, 255, static_cast<std::uint8_t>(160 * c.knock)));

    auto valve = [&](float vx, float lift01, sf::Color col) {
        const float t = 24.0f * std::clamp(lift01, 0.0f, 1.0f);
        u.rect(vx - 2.5f, deckY - 44.0f + t, 5.0f, 44.0f, col);
        u.rect(vx - 11.0f, deckY - 4.0f + t, 22.0f, 7.0f, col);
    };
    valve(ox - bore * 0.26f, c.intakeLift, pal.intake);
    valve(ox + bore * 0.26f, c.exhaustLift, pal.exhaust);

    const float crankX    = ox + a * std::sin(theta);
    const float crankPinY = crankY - a * std::cos(theta);
    {
        // The connecting rod is the one part here that is not axis-aligned.
        const sf::Vector2f d{crankX - ox, crankPinY - pinY};
        const float len = std::sqrt(d.x * d.x + d.y * d.y);
        sf::RectangleShape r({8.0f, len});
        r.setOrigin({4.0f, 0.0f});
        r.setPosition({ox, pinY});
        r.setRotation(sf::degrees(-std::atan2(d.x, d.y) * 180.0f / kPi));
        r.setFillColor(pal.metal);
        u.rectShape(r);
    }

    u.rect(ox - (bore - 2.0f) * 0.5f, pinY - pistonH * 0.62f, bore - 2.0f, pistonH,
           sf::Color(196, 202, 214));
    u.rect(ox - 13.0f, crankY - 13.0f, 26.0f, 26.0f, sf::Color(92, 99, 116));
    u.rect(crankX - 7.0f, crankPinY - 7.0f, 14.0f, 14.0f, pal.accent);

    u.centred(strokeName(c.phase), ox, crankY + 34.0f, 18, pal.accent);
    u.centred(fmt("%.0f deg", c.phase), ox, crankY + 58.0f, 13, pal.dim);
    u.centred(fmt("%.1f bar", c.pressure) + fmt("   %.0f K", c.temperature),
              ox, crankY + 78.0f, 13, pal.dim);
}

// ---------------------------------------------------------------------------
// One bore seen from above: valve heads opening and closing, and the ring
// around each is the curtain gap the gas actually flows through, to scale.
// ---------------------------------------------------------------------------
void drawBoreTop(ui::Ui& u, const sim::EngineParams& p, const sim::CylinderView& c,
                 float cx, float cy, float radius) {
    const ui::Palette& pal = u.pal();
    const float scale = radius / (0.5f * static_cast<float>(p.bore));
    const bool burning = c.burnFraction > 0.001f && c.burnFraction < 0.999f;

    auto circle = [&](float x, float y, float r, sf::Color fill, sf::Color outline,
                      float thick) {
        sf::CircleShape s(r, 40);
        s.setOrigin({r, r});
        s.setPosition({x, y});
        s.setFillColor(fill);
        s.setOutlineColor(outline);
        s.setOutlineThickness(thick);
        u.circleShape(s);
    };

    if (burning) {
        const float glow = radius * (1.25f + 0.35f * std::sin(c.burnFraction * kPi));
        circle(cx, cy, glow,
               sf::Color(255, 170, 60,
                         static_cast<std::uint8_t>(90 * std::sin(c.burnFraction * kPi))),
               sf::Color::Transparent, 0.0f);
    }
    if (c.knock > 0.02f)
        circle(cx, cy, radius * 1.5f, sf::Color(255, 255, 255,
               static_cast<std::uint8_t>(120 * c.knock)), sf::Color::Transparent, 0.0f);

    sf::Color gc = chargeColour(c.temperature);
    gc.a = static_cast<std::uint8_t>(std::clamp(55.0f + c.pressure * 2.4f, 55.0f, 225.0f));
    circle(cx, cy, radius, gc, burning ? sf::Color(255, 200, 120) : pal.line,
           burning ? 3.0f : 2.0f);

    auto valves = [&](const sim::ValveTiming& v, float lift01, sf::Color col, float side) {
        const float r = std::max(2.0f, 0.5f * static_cast<float>(v.diameter) * scale);
        const int   n = std::max(1, v.count);
        for (int i = 0; i < n; ++i) {
            const float spread = (n == 1) ? 0.0f
                                          : (static_cast<float>(i) / (n - 1) - 0.5f) * 2.0f;
            const float vx = cx + spread * radius * 0.46f;
            const float vy = cy + side * radius * 0.40f;

            const float curtain = static_cast<float>(v.maxLift) * lift01 * scale;
            if (curtain > 0.3f)
                circle(vx, vy, r + curtain * 0.5f, sf::Color::Transparent,
                       sf::Color(col.r, col.g, col.b, 150), std::max(1.0f, curtain));

            const float t = std::clamp(lift01, 0.0f, 1.0f);
            circle(vx, vy, r,
                   sf::Color(static_cast<std::uint8_t>(col.r * (0.28f + 0.72f * t)),
                             static_cast<std::uint8_t>(col.g * (0.28f + 0.72f * t)),
                             static_cast<std::uint8_t>(col.b * (0.28f + 0.72f * t))),
                   col, 1.5f);
        }
    };
    valves(p.intake,  c.intakeLift,  pal.intake,  -1.0f);
    valves(p.exhaust, c.exhaustLift, pal.exhaust,  1.0f);

    const float pr = burning ? 7.0f : 4.0f;
    circle(cx, cy, pr, burning ? sf::Color(255, 246, 200) : sf::Color(120, 126, 140),
           sf::Color::Transparent, 0.0f);
}

// The whole engine from above, one column per bank - so an inline, a V, a flat
// and a W all read as what they are.
void drawEngineTop(ui::Ui& u, const sim::EngineParams& p, const sim::Snapshot& s,
                   float x, float y, float w, float h) {
    const ui::Palette& pal = u.pal();
    u.text("ENGINE FROM ABOVE", x + 14.0f, y + 10.0f, 13, pal.dim);

    const int n = std::max(1, s.cylinderCount);
    const int banks = std::clamp(p.banks, 1, 4);
    const int rows = (n + banks - 1) / banks;
    const float top = y + 34.0f;
    const float rowH = (h - 52.0f) / rows;
    const float colW = (w - 40.0f) / banks;
    const float radius = std::min(rowH * 0.40f, colW * 0.40f);

    for (int b = 0; b < banks; ++b) {
        const float cx = x + 20.0f + colW * (b + 0.5f);
        u.rect(cx - radius * 1.45f, top - 6.0f, radius * 2.9f, h - 46.0f,
               pal.block, pal.line, 1.0f);
        u.line({cx - radius * 0.62f, top - 6.0f}, {cx - radius * 0.62f, top + h - 52.0f},
               sf::Color(60, 70, 90));
        u.line({cx + radius * 0.62f, top - 6.0f}, {cx + radius * 0.62f, top + h - 52.0f},
               sf::Color(80, 60, 60));
        if (banks > 1)
            u.centred(std::string("BANK ") + static_cast<char>('A' + b), cx, y + 10.0f,
                      11, pal.dim);
    }

    for (int i = 0; i < n; ++i) {
        const sim::CylinderView& c = s.cyl[static_cast<std::size_t>(i)];
        const int b = std::clamp(c.bank, 0, banks - 1);
        const int row = banks > 1 ? i / banks : i;
        const float cx = x + 20.0f + colW * (b + 0.5f);
        const float cy = top + rowH * (row + 0.5f);
        drawBoreTop(u, p, c, cx, cy, radius);

        const bool burning = c.burnFraction > 0.001f && c.burnFraction < 0.999f;
        u.centred("#" + std::to_string(i + 1), cx, cy + radius + 2.0f, 12,
                  burning ? pal.accent : pal.dim);
    }
}

// ---------------------------------------------------------------------------
void drawValveDiagram(ui::Ui& u, const sim::EngineParams& p, float camAdvance,
                      const sim::CylinderView& c, float x, float y, float w, float h) {
    const ui::Palette& pal = u.pal();
    panel(u, x, y, w, h);
    u.text("VALVE LIFT / CRANK ANGLE", x + 12.0f, y + 8.0f, 13, pal.dim);

    const float x0 = x + 44.0f, x1 = x + w - 12.0f;
    const float yb = y + h - 24.0f, yt = y + 34.0f;
    const float maxLift = std::max(0.5f,
        static_cast<float>(std::max(p.intake.maxLift, p.exhaust.maxLift)) * 1000.0f);

    auto px = [&](float deg) { return x0 + (x1 - x0) * deg / 720.0f; };
    auto py = [&](float mm)  { return yb - (yb - yt) * (mm / maxLift); };
    // The engine phases the intake cam, so the diagram has to phase with it or
    // it stops describing what is actually happening.
    auto liftI = [&](float deg) {
        return static_cast<float>(sim::valveLiftAt(p.intake, deg + camAdvance)) * 1000.0f;
    };
    auto liftE = [&](float deg) {
        return static_cast<float>(sim::valveLiftAt(p.exhaust, deg)) * 1000.0f;
    };

    for (int deg = 0; deg <= 720; deg += 90) {
        u.line({px(static_cast<float>(deg)), yt}, {px(static_cast<float>(deg)), yb}, pal.grid);
        if (deg % 180 == 0)
            u.centred(std::to_string(deg), px(static_cast<float>(deg)), yb + 4.0f, 12, pal.dim);
    }
    const int mmStep = maxLift > 12.0f ? 4 : 2;
    for (int mm = 0; mm <= static_cast<int>(maxLift); mm += mmStep) {
        u.line({x0, py(static_cast<float>(mm))}, {x1, py(static_cast<float>(mm))}, pal.grid);
        u.right(std::to_string(mm), x0 - 6.0f, py(static_cast<float>(mm)) - 8.0f, 12, pal.dim);
    }

    // Overlap: both valves off their seats at once, which is when the exhaust
    // pulse can pull fresh charge through the cylinder.
    {
        sf::VertexArray band(sf::PrimitiveType::TriangleStrip);
        for (int deg = 0; deg <= 720; ++deg) {
            const float d = static_cast<float>(deg);
            const float both = std::min(liftI(d), liftE(d));
            if (both <= 0.001f) continue;
            sf::Vertex v0, v1;
            v0.position = {px(d), yb};
            v1.position = {px(d), py(both)};
            v0.color = v1.color = sf::Color(255, 214, 120, 70);
            band.append(v0); band.append(v1);
        }
        u.vertices(band);
    }

    auto curve = [&](bool intake, sf::Color col) {
        sf::VertexArray line(sf::PrimitiveType::LineStrip);
        for (int deg = 0; deg <= 720; deg += 2) {
            const float d = static_cast<float>(deg);
            sf::Vertex v;
            v.position = {px(d), py(intake ? liftI(d) : liftE(d))};
            v.color = col;
            line.append(v);
        }
        u.vertices(line);
    };
    curve(false, pal.exhaust);
    curve(true,  pal.intake);

    u.line({px(c.phase), yt}, {px(c.phase), yb}, sf::Color(255, 255, 255, 120));

    u.text(fmt("IVO %.0f", static_cast<float>(p.intake.openDeg - camAdvance)) +
           fmt("  IVC %.0f", static_cast<float>(p.intake.closeDeg - camAdvance)),
           x + 200.0f, y + 8.0f, 12, pal.intake);
    u.text(fmt("EVO %.0f", static_cast<float>(p.exhaust.openDeg)) +
           fmt("  EVC %.0f", static_cast<float>(p.exhaust.closeDeg)),
           x + 330.0f, y + 8.0f, 12, pal.exhaust);
}

void drawTacho(ui::Ui& u, float cx, float cy, float radius, const sim::Snapshot& s,
               float redline) {
    const ui::Palette& pal = u.pal();
    const float start = 140.0f, sweep = 260.0f;
    const float maxRpm = std::max(2000.0f, std::ceil(redline * 1.12f / 1000.0f) * 1000.0f);
    const int   step   = maxRpm > 12000.0f ? 1000 : 500;
    const int   major  = maxRpm > 12000.0f ? 2000 : 1000;
    auto angleFor = [&](float rpm) {
        return (start + sweep * std::clamp(rpm / maxRpm, 0.0f, 1.0f)) * kPi / 180.0f;
    };

    for (int r = 0; r <= static_cast<int>(maxRpm); r += step) {
        const bool isMajor = (r % major) == 0;
        const float ang = angleFor(static_cast<float>(r));
        const float r0 = radius - (isMajor ? 18.0f : 10.0f);
        const sf::Color col = (r >= redline) ? pal.alert : (isMajor ? pal.text : pal.dim);
        u.line({cx + r0 * std::cos(ang), cy + r0 * std::sin(ang)},
               {cx + radius * std::cos(ang), cy + radius * std::sin(ang)}, col);
        if (isMajor) {
            const float lr = radius - 34.0f;
            u.centred(std::to_string(r / 1000), cx + lr * std::cos(ang),
                      cy + lr * std::sin(ang) - 10.0f, 15, col);
        }
    }

    sf::VertexArray arc(sf::PrimitiveType::TriangleStrip);
    const int steps = 96;
    for (int i = 0; i <= steps; ++i) {
        const float f = static_cast<float>(i) / steps;
        const float rpm = f * std::clamp(s.rpm, 0.0f, maxRpm);
        const float ang = angleFor(rpm);
        const sf::Color col = rpm >= redline ? pal.alert : pal.accent;
        sf::Vertex v0, v1;
        v0.position = {cx + (radius - 6.0f) * std::cos(ang), cy + (radius - 6.0f) * std::sin(ang)};
        v1.position = {cx + (radius + 2.0f) * std::cos(ang), cy + (radius + 2.0f) * std::sin(ang)};
        v0.color = v1.color = col;
        arc.append(v0); arc.append(v1);
    }
    u.vertices(arc);

    const float ang = angleFor(s.rpm);
    sf::RectangleShape needle({radius - 26.0f, 4.0f});
    needle.setOrigin({6.0f, 2.0f});
    needle.setPosition({cx, cy});
    needle.setRotation(sf::degrees(ang * 180.0f / kPi));
    needle.setFillColor(sf::Color(240, 240, 245));
    u.rectShape(needle);

    u.rect(cx - 9.0f, cy - 9.0f, 18.0f, 18.0f, pal.line);
    u.centred(fmt("%.0f", s.rpm), cx, cy + 20.0f, 32, pal.text);
    u.centred("rpm", cx, cy + 62.0f, 13, pal.dim);
}

void drawGearbox(ui::Ui& u, const sim::Snapshot& s, float x, float y, float w) {
    const ui::Palette& pal = u.pal();
    u.text("TRANSMISSION", x, y, 13, pal.dim);

    const int slots = std::max(2, s.gearCount + 1);
    const float bw = (w - (slots - 1) * 6.0f) / slots;
    for (int i = 0; i < slots; ++i) {
        const bool active = (i == s.gear);
        u.rect(x + i * (bw + 6.0f), y + 20.0f, bw, 34.0f,
               active ? pal.accent : pal.panelAlt, active ? pal.accent : pal.line, 1.0f);
        u.centred(i == 0 ? "N" : std::to_string(i), x + i * (bw + 6.0f) + bw * 0.5f,
                  y + 26.0f, 18, active ? sf::Color(20, 22, 28) : pal.dim);
    }

    u.text("ROAD SPEED", x, y + 68.0f, 13, pal.dim);
    u.right(fmt("%.0f km/h", s.speedKph), x + w, y + 64.0f, 22, pal.text);

    u.text("CLUTCH", x, y + 100.0f, 13, pal.dim);
    u.rect(x + w * 0.45f, y + 100.0f, w * 0.55f, 10.0f, pal.grid);
    u.rect(x + w * 0.45f, y + 100.0f, w * 0.55f * std::clamp(s.clutchLock, 0.0f, 1.0f),
           10.0f, std::abs(s.clutchSlip) > 120.0f ? pal.alert : pal.good);
    u.text(s.gear == 0 ? "neutral"
                       : (std::abs(s.clutchSlip) > 120.0f ? "slipping" : "locked"),
           x, y + 118.0f, 12, pal.dim);
    u.right(fmt("%.0f N m at the wheels", s.wheelTorque), x + w, y + 118.0f, 12, pal.dim);
}

void drawBar(ui::Ui& u, const std::string& label, float value01, float x, float y,
             float w, sf::Color col) {
    u.text(label, x, y - 17.0f, 13, u.pal().dim);
    u.rect(x, y, w, 10.0f, u.pal().grid);
    u.rect(x, y, w * std::clamp(value01, 0.0f, 1.0f), 10.0f, col);
}

void drawPressureTrace(ui::Ui& u, const sim::Engine& engine, const sim::CylinderView& c,
                       float x, float y, float w, float h, float peakHint) {
    const ui::Palette& pal = u.pal();
    panel(u, x, y, w, h);
    u.text("CYLINDER PRESSURE / CRANK ANGLE", x + 12.0f, y + 8.0f, 13, pal.dim);

    // The scale follows the engine: 90 bar suits a stock four, and hides most
    // of the trace on something with two bar of boost.
    const float maxBar = std::max(60.0f, std::ceil(peakHint * 1.25f / 30.0f) * 30.0f);
    const float x0 = x + 44.0f, x1 = x + w - 12.0f;
    const float yb = y + h - 24.0f, yt = y + 34.0f;
    for (int b = 0; b <= 3; ++b) {
        const float gy = yb - (yb - yt) * b / 3.0f;
        u.line({x0, gy}, {x1, gy}, pal.grid);
        u.right(std::to_string(static_cast<int>(maxBar * b / 3)), x0 - 6.0f, gy - 8.0f,
                12, pal.dim);
    }
    for (int deg = 0; deg <= 720; deg += 180)
        u.line({x0 + (x1 - x0) * deg / 720.0f, yt}, {x0 + (x1 - x0) * deg / 720.0f, yb},
               pal.grid);

    sf::VertexArray trace(sf::PrimitiveType::LineStrip);
    for (std::size_t i = 0; i < sim::Engine::kTraceBins; ++i) {
        const float bar = std::clamp(engine.traceAt(i), 0.0f, maxBar);
        sf::Vertex v;
        v.position = {x0 + (x1 - x0) * i / (sim::Engine::kTraceBins - 1),
                      yb - (yb - yt) * (bar / maxBar)};
        v.color = pal.accent;
        trace.append(v);
    }
    u.vertices(trace);
    u.line({x0 + (x1 - x0) * c.phase / 720.0f, yt},
           {x0 + (x1 - x0) * c.phase / 720.0f, yb}, sf::Color(255, 255, 255, 120));
}

void drawScope(ui::Ui& u, const audio::EngineSound& sound, float x, float y,
               float w, float h) {
    const ui::Palette& pal = u.pal();
    panel(u, x, y, w, h);
    u.text("EXHAUST WAVEFORM", x + 12.0f, y + 8.0f, 13, pal.dim);

    const std::size_t n = audio::EngineSound::kScope;
    const std::size_t head = sound.scopeHead();
    sf::VertexArray wave(sf::PrimitiveType::LineStrip);
    for (std::size_t i = 0; i < n; ++i) {
        const float v = sound.scopeAt((head + 1 + i) % n);
        sf::Vertex vert;
        vert.position = {x + 12.0f + (w - 24.0f) * i / (n - 1),
                         y + h * 0.6f - std::clamp(v, -1.0f, 1.0f) * (h * 0.3f)};
        vert.color = pal.intake;
        wave.append(vert);
    }
    u.vertices(wave);
}

// A row of warning lamps. Each one is a thing the simulation is actually doing,
// not decoration.
void drawLamps(ui::Ui& u, const sim::Snapshot& s, const sim::EngineParams& p,
               float x, float y, float w) {
    const ui::Palette& pal = u.pal();
    struct Lamp { const char* name; bool on; sf::Color col; };
    const Lamp lamps[] = {
        {"KNOCK", s.knock > 0.05f, pal.alert},
        {"FLOAT", s.valveFloat > 0.02f, pal.alert},
        {"OIL",   s.oilPressure < 0.9f && s.rpm > 300.0f, pal.alert},
        {"COLD",  s.oilTemp < static_cast<float>(p.oilTempTarget) - 25.0f, pal.intake},
        {"BOOST", s.boost > 5.0f, pal.good},
        {"LIMIT", s.rpm > static_cast<float>(p.redline) - 100.0, pal.alert},
    };
    const int n = static_cast<int>(sizeof(lamps) / sizeof(lamps[0]));
    const float bw = (w - (n - 1) * 6.0f) / n;
    for (int i = 0; i < n; ++i) {
        const sf::Color col = lamps[i].on ? lamps[i].col : pal.grid;
        u.rect(x + i * (bw + 6.0f), y, bw, 22.0f,
               lamps[i].on ? sf::Color(col.r, col.g, col.b, 60) : pal.panelAlt,
               col, 1.0f);
        u.centred(lamps[i].name, x + i * (bw + 6.0f) + bw * 0.5f, y + 3.0f, 11,
                  lamps[i].on ? col : pal.dim);
    }
}

} // namespace

int main() {
    sim::EngineDesign design  = sim::preset(0);
    sim::EngineDesign applied = design;
    sim::EngineParams params  = sim::paramsFromDesign(design);

    sim::Engine engine(params);
    engine.drivetrain().setParams(sim::drivetrainFromDesign(design));
    audio::EngineSound sound(engine);
    sim::Dyno dyno;

    sf::RenderWindow window(sf::VideoMode({static_cast<unsigned>(kW),
                                           static_cast<unsigned>(kH)}),
                            "Enginio2D - engine simulator");
    window.setFramerateLimit(60);
    window.setView(fittedView(window.getSize()));

    const sf::Font font = loadFont();
    ui::Palette palette = ui::makePalette(design.theme, design.accentHue,
                                          design.blockShade, design.coverHue,
                                          design.coverSat);
    ui::Ui gui(font, palette);
    ui::Editor editor;

    sound.setVolume(60.0f);
    sound.play();                             // the physics runs on the audio thread

    float throttleCmd = 0.0f;
    float brakeCmd = 0.0f;
    bool  ignition = true;
    int   gearShown = 0;
    bool  pendingChange = false;
    bool  forceApply = false;
    float applyTimer = 0.0f;

    sf::Clock clock;
    while (window.isOpen()) {
        const float dt = std::min(0.1f, clock.restart().asSeconds());

        ui::InputState in;
        in.mouse = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        in.down = sf::Mouse::isButtonPressed(sf::Mouse::Button::Left);
        in.fine = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) ||
                  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift);

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
            if (const auto* rs = event->getIf<sf::Event::Resized>()) {
                (void)rs;
                window.setView(fittedView(window.getSize()));
            }
            if (const auto* mb = event->getIf<sf::Event::MouseButtonPressed>())
                if (mb->button == sf::Mouse::Button::Left) in.pressed = true;
            if (const auto* mb = event->getIf<sf::Event::MouseButtonReleased>())
                if (mb->button == sf::Mouse::Button::Left) in.released = true;
            if (const auto* w = event->getIf<sf::Event::MouseWheelScrolled>())
                in.wheel += w->delta;
            if (const auto* txt = event->getIf<sf::Event::TextEntered>())
                in.typed.push_back(txt->unicode);
            if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
                using K = sf::Keyboard::Key;
                if (key->code == K::Backspace) in.backspace = true;
                if (key->code == K::Enter)     in.enter = true;
                if (key->code == K::Tab) {
                    editor.visible = !editor.visible;
                    if (editor.visible) editor.refreshSaved();
                }
                if (!gui.textFocused()) {
                    switch (key->code) {
                        case K::Escape: window.close(); break;
                        case K::I: ignition = !ignition; sound.setIgnition(ignition); break;
                        case K::E: sound.requestGear(std::min(gearShown + 1, 8)); break;
                        case K::Q: sound.requestGear(std::max(gearShown - 1, 0)); break;
                        case K::N: case K::Num0: sound.requestGear(0); break;
                        case K::Num1: sound.requestGear(1); break;
                        case K::Num2: sound.requestGear(2); break;
                        case K::Num3: sound.requestGear(3); break;
                        case K::Num4: sound.requestGear(4); break;
                        case K::Num5: sound.requestGear(5); break;
                        case K::Num6: sound.requestGear(6); break;
                        case K::Num7: sound.requestGear(7); break;
                        case K::Num8: sound.requestGear(8); break;
                        default: break;
                    }
                }
            }
        }

        const bool driving = !gui.textFocused();
        const bool wantThrottle = driving &&
            (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W) ||
             sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up));
        const bool wantBrake = driving &&
            (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down) ||
             sf::Keyboard::isKeyPressed(sf::Keyboard::Key::B));
        // A pedal is not a switch: ramp so the manifold has time to fill.
        throttleCmd += ((wantThrottle ? 1.0f : 0.0f) - throttleCmd) * std::min(1.0f, dt * 9.0f);
        brakeCmd    += ((wantBrake ? 1.0f : 0.0f) - brakeCmd) * std::min(1.0f, dt * 8.0f);
        sound.setThrottle(throttleCmd);
        sound.setBrake(brakeCmd);
        sound.setStarter(driving && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S));

        const sim::Snapshot s = sound.snapshot();
        const sim::CylinderView& c1 = s.cyl[0];
        gearShown = s.gear;

        // Appearance is not physics: it takes effect the moment it is edited.
        palette = ui::makePalette(design.theme, design.accentHue, design.blockShade,
                                  design.coverHue, design.coverSat);
        gui.setPalette(palette);

        window.clear(palette.bg);
        gui.begin(window, in);

        // ---- Engine views ---------------------------------------------------
        const float viewsRight = 700.0f;
        const int   viewCount = (design.showCutaway ? 1 : 0) + (design.showTopView ? 1 : 0);
        float vx = 12.0f;
        const float vw = viewCount > 0 ? (viewsRight - 12.0f - (viewCount - 1) * 8.0f) / viewCount
                                       : 0.0f;
        if (design.showCutaway) {
            panel(gui, vx, 12.0f, vw, 560.0f);
            gui.text("CYLINDER 1  SECTION", vx + 14.0f, 22.0f, 13, palette.dim);
            drawCylinderSide(gui, params, c1, vx + vw * 0.5f, 60.0f, 560.0f);
            vx += vw + 8.0f;
        }
        if (design.showTopView) {
            panel(gui, vx, 12.0f, vw, 560.0f);
            drawEngineTop(gui, params, s, vx, 12.0f, vw, 560.0f);
            vx += vw + 8.0f;
        }

        // ---- Instruments ----------------------------------------------------
        const float ix = viewCount > 0 ? 708.0f : 12.0f;
        const float iw = 1428.0f - ix;
        panel(gui, ix, 12.0f, iw, 560.0f);
        drawTacho(gui, ix + 136.0f, 176.0f, 132.0f, s, static_cast<float>(params.redline));
        drawGearbox(gui, s, ix + 16.0f, 336.0f, 268.0f);

        const float rx = ix + 300.0f;
        const float rw = 1428.0f - rx;
        const float colW = (rw - 20.0f) * 0.5f;
        gui.text("OUTPUT", rx, 34.0f, 13, palette.dim);
        gui.right(fmt("%.1f", s.torque), rx + 150.0f, 50.0f, 28, palette.text);
        gui.text("N m", rx + 160.0f, 64.0f, 14, palette.dim);
        gui.right(fmt("%.1f", s.power), rx + 150.0f, 86.0f, 28, palette.text);
        gui.text("kW", rx + 160.0f, 100.0f, 14, palette.dim);
        gui.right(fmt("%.0f hp", s.power * 1.34102f), rx + rw, 92.0f, 16, palette.dim);

        gui.column(rx, 142.0f, colW);
        gui.readout("MANIFOLD", fmt("%.1f kPa", s.manifoldPressure));
        gui.readout("BOOST", s.boost > 0.5f ? fmt("%.2f bar", s.boost * 0.01f)
                                            : std::string("-"),
                    s.boost > 0.5f ? palette.good : palette.text);
        gui.readout("CHARGE TEMP", fmt("%.0f K", s.chargeTemp));
        gui.readout("VOL. EFFICIENCY", fmt("%.0f %%", s.volumetricEff * 100.0f));
        gui.readout("RESIDUAL GAS", fmt("%.0f %%", s.residualFraction * 100.0f));
        gui.readout("PEAK PRESSURE", fmt("%.1f bar", s.peakPressure));
        gui.readout("EXHAUST TEMP", fmt("%.0f K", s.exhaustTemp),
                    s.exhaustTemp > 1250.0f ? palette.alert : palette.text);
        gui.readout("BACK PRESSURE", fmt("%.1f kPa", s.backPressure));
        gui.readout("SPARK", fmt("%.1f deg", s.sparkAdvance));
        gui.readout("KNOCK RETARD", fmt("%.1f deg", s.knockRetard),
                    s.knockRetard > 0.5f ? palette.alert : palette.text);
        gui.readout("CAM PHASE", fmt("%.1f deg", s.camAdvance));

        gui.column(rx + colW + 20.0f, 142.0f, colW);
        gui.readout("AIR / FUEL", fmt("%.1f : 1", s.afr));
        gui.readout("LAMBDA", fmt("%.2f", s.lambda),
                    s.lambda > 1.05f ? palette.alert : palette.text);
        gui.readout("FUEL FLOW", fmt("%.2f kg/h", s.fuelFlow));
        gui.readout("BSFC", s.bsfc > 1.0f ? fmt("%.0f g/kWh", s.bsfc) : std::string("-"));
        gui.readout("FRICTION", fmt("%.2f bar", s.fmep));
        gui.readout("OIL TEMP", fmt("%.0f C", s.oilTemp - 273.15f));
        gui.readout("OIL PRESSURE", fmt("%.2f bar", s.oilPressure),
                    s.oilPressure < 0.9f && s.rpm > 300.0f ? palette.alert : palette.text);
        gui.readout("VALVE FLOAT", fmt("%.0f %%", s.valveFloat * 100.0f),
                    s.valveFloat > 0.02f ? palette.alert : palette.text);
        gui.readout("IDLE VALVE", fmt("%.0f %%", s.idleValve * 100.0f));
        gui.readout("CLUTCH SLIP", fmt("%.0f rpm", s.clutchSlip));
        gui.readout("KNOCK", fmt("%.0f %%", s.knock * 100.0f),
                    s.knock > 0.05f ? palette.alert : palette.text);

        drawBar(gui, "THROTTLE", s.throttle, rx, 420.0f, rw, palette.accent);
        drawBar(gui, "BRAKE", s.brake, rx, 458.0f, rw, palette.exhaust);
        drawLamps(gui, s, params, rx, 486.0f, rw);

        gui.text(design.name, ix + 18.0f, 500.0f, 15, palette.accent);
        gui.text(ignition ? "IGNITION ON" : "IGNITION OFF", ix + 18.0f, 522.0f, 13,
                 ignition ? palette.good : palette.alert);
        gui.text("W throttle   DOWN brake   S starter   Q/E shift   0-8 gear   "
                 "I ignition   TAB editor   ESC quit",
                 ix + 18.0f, 544.0f, 12, palette.dim);

        // ---- Plots ----------------------------------------------------------
        drawValveDiagram(gui, params, s.camAdvance, c1, 12.0f, 584.0f, 480.0f, 216.0f);
        drawPressureTrace(gui, engine, c1, 500.0f, 584.0f, 480.0f, 216.0f, s.peakPressure);
        drawScope(gui, sound, 988.0f, 584.0f, 440.0f, 216.0f);

        // ---- Editor ---------------------------------------------------------
        if (editor.visible) {
            const ui::EditorResult res = editor.draw(gui, design, dyno, pendingChange, dt);
            if (res.changed) { pendingChange = true; applyTimer = 0.18f; }
            if (res.revert)  { design = applied; pendingChange = false; forceApply = false; }
            if (res.apply)   { pendingChange = true; forceApply = true; }
        }
        gui.end();

        // The rebuild is handed to the audio thread, which owns the engine. If
        // it has not picked up the last one yet we simply try again next frame.
        if (pendingChange) {
            applyTimer -= dt;
            const bool wantNow = forceApply || (editor.liveApply && applyTimer <= 0.0f);
            if (wantNow) {
                sim::EngineParams next = sim::paramsFromDesign(design);
                if (sound.requestConfig(next, sim::drivetrainFromDesign(design))) {
                    params = next;
                    applied = design;
                    pendingChange = false;
                    forceApply = false;
                }
            }
        } else {
            forceApply = false;
        }

        window.display();
    }

    dyno.cancel();
    sound.stop();
    return 0;
}
