#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include "audio/EngineSound.h"
#include "sim/Engine.h"

namespace {

constexpr unsigned kWinW = 1440;
constexpr unsigned kWinH = 812;
constexpr float    kPi   = 3.14159265358979323846f;

const sf::Color kBg      {14, 16, 20};
const sf::Color kPanel   {24, 27, 34};
const sf::Color kLine    {58, 64, 78};
const sf::Color kGrid    {40, 45, 56};
const sf::Color kText    {214, 220, 232};
const sf::Color kDim     {126, 134, 150};
const sf::Color kAccent  {255, 138, 46};
const sf::Color kIntake  {86, 168, 255};
const sf::Color kExhaust {255, 104, 76};
const sf::Color kGood    {120, 210, 140};
const sf::Color kAlert   {226, 68, 60};
const sf::Color kMetal   {150, 158, 175};

std::string fmt(const char* spec, float v) {
    char buf[64];
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

void drawPanel(sf::RenderTarget& rt, float x, float y, float w, float h) {
    sf::RectangleShape r({w, h});
    r.setPosition({x, y});
    r.setFillColor(kPanel);
    r.setOutlineColor(kLine);
    r.setOutlineThickness(1.0f);
    rt.draw(r);
}

void drawLine(sf::RenderTarget& rt, sf::Vector2f a, sf::Vector2f b, sf::Color c) {
    sf::VertexArray v(sf::PrimitiveType::Lines, 2);
    v[0].position = a; v[1].position = b;
    v[0].color = v[1].color = c;
    rt.draw(v);
}

class Hud {
public:
    explicit Hud(const sf::Font& font) : m_font(font) {}

    void text(sf::RenderTarget& rt, const std::string& s, float x, float y,
              unsigned size = 15, sf::Color col = kText) {
        sf::Text t(m_font, s, size);
        t.setFillColor(col);
        t.setPosition({std::round(x), std::round(y)});
        rt.draw(t);
    }

    void centred(sf::RenderTarget& rt, const std::string& s, float cx, float y,
                 unsigned size = 15, sf::Color col = kText) {
        sf::Text t(m_font, s, size);
        t.setFillColor(col);
        const auto b = t.getLocalBounds();
        t.setPosition({std::round(cx - b.size.x * 0.5f - b.position.x), std::round(y)});
        rt.draw(t);
    }

    void right(sf::RenderTarget& rt, const std::string& s, float rx, float y,
               unsigned size = 15, sf::Color col = kText) {
        sf::Text t(m_font, s, size);
        t.setFillColor(col);
        const auto b = t.getLocalBounds();
        t.setPosition({std::round(rx - b.size.x - b.position.x), std::round(y)});
        rt.draw(t);
    }

    void readout(sf::RenderTarget& rt, const std::string& label, const std::string& value,
                 float x, float y, float w, sf::Color col = kText) {
        text(rt, label, x, y, 13, kDim);
        right(rt, value, x + w, y - 2.0f, 16, col);
    }

private:
    const sf::Font& m_font;
};

// ---------------------------------------------------------------------------
// Side cutaway of one cylinder.
// ---------------------------------------------------------------------------
void drawCylinderSide(sf::RenderTarget& rt, Hud& hud, const sim::EngineParams& p,
                      const sim::CylinderView& c, float ox, float topY) {
    const float scale  = 1150.0f;                  // pixels per metre
    const float a      = 0.5f * static_cast<float>(p.stroke) * scale;
    const float rod    = static_cast<float>(p.rodLength) * scale;
    const float bore   = static_cast<float>(p.bore) * scale;
    const float crankY = topY + 300.0f;
    const float theta  = c.phase * kPi / 180.0f;

    const float pin  = a * std::cos(theta) +
                       std::sqrt(rod * rod - a * a * std::sin(theta) * std::sin(theta));
    const float pinY = crankY - pin;
    const float deckY   = crankY - (rod + a) - 6.0f;
    const float pistonH = 30.0f;

    sf::RectangleShape wallL({6.0f, 290.0f});
    wallL.setPosition({ox - bore * 0.5f - 6.0f, deckY});
    wallL.setFillColor({46, 51, 62});
    sf::RectangleShape wallR = wallL;
    wallR.setPosition({ox + bore * 0.5f, deckY});
    sf::RectangleShape head({bore + 12.0f, 16.0f});
    head.setPosition({ox - bore * 0.5f - 6.0f, deckY - 16.0f});
    head.setFillColor({46, 51, 62});
    rt.draw(wallL); rt.draw(wallR); rt.draw(head);

    const float chamberH = std::max(2.0f, pinY - pistonH * 0.5f - deckY);
    sf::RectangleShape gas({bore, chamberH});
    gas.setPosition({ox - bore * 0.5f, deckY});
    sf::Color gc = chargeColour(c.temperature);
    gc.a = static_cast<std::uint8_t>(std::clamp(70.0f + c.pressure * 3.2f, 70.0f, 245.0f));
    gas.setFillColor(gc);
    rt.draw(gas);

    auto valve = [&](float vx, float lift01, sf::Color col) {
        const float travel = 24.0f * lift01;
        sf::RectangleShape stem({5.0f, 44.0f});
        stem.setPosition({vx - 2.5f, deckY - 44.0f + travel});
        stem.setFillColor(col);
        sf::RectangleShape face({22.0f, 7.0f});
        face.setPosition({vx - 11.0f, deckY - 4.0f + travel});
        face.setFillColor(col);
        rt.draw(stem); rt.draw(face);
    };
    valve(ox - bore * 0.26f, c.intakeLift, kIntake);
    valve(ox + bore * 0.26f, c.exhaustLift, kExhaust);

    const float crankX    = ox + a * std::sin(theta);
    const float crankPinY = crankY - a * std::cos(theta);
    {
        const sf::Vector2f d{crankX - ox, crankPinY - pinY};
        const float len = std::sqrt(d.x * d.x + d.y * d.y);
        sf::RectangleShape r({8.0f, len});
        r.setOrigin({4.0f, 0.0f});
        r.setPosition({ox, pinY});
        r.setRotation(sf::degrees(-std::atan2(d.x, d.y) * 180.0f / kPi));
        r.setFillColor(kMetal);
        rt.draw(r);
    }

    sf::RectangleShape piston({bore - 2.0f, pistonH});
    piston.setOrigin({(bore - 2.0f) * 0.5f, pistonH * 0.5f});
    piston.setPosition({ox, pinY});
    piston.setFillColor({196, 202, 214});
    rt.draw(piston);

    sf::CircleShape mainBearing(13.0f);
    mainBearing.setOrigin({13.0f, 13.0f});
    mainBearing.setPosition({ox, crankY});
    mainBearing.setFillColor({92, 99, 116});
    rt.draw(mainBearing);
    sf::CircleShape journal(8.0f);
    journal.setOrigin({8.0f, 8.0f});
    journal.setPosition({crankX, crankPinY});
    journal.setFillColor(kAccent);
    rt.draw(journal);

    hud.centred(rt, strokeName(c.phase), ox, crankY + 34.0f, 18, kAccent);
    hud.centred(rt, fmt("%.0f deg", c.phase), ox, crankY + 58.0f, 13, kDim);
    hud.centred(rt, fmt("%.1f bar", c.pressure) + fmt("   %.0f K", c.temperature),
                ox, crankY + 78.0f, 13, kDim);
}

// ---------------------------------------------------------------------------
// One bore seen from above: valve heads opening and closing, and the ring
// around each is the curtain gap the gas actually flows through, to scale.
// ---------------------------------------------------------------------------
void drawBoreTop(sf::RenderTarget& rt, const sim::EngineParams& p,
                 const sim::CylinderView& c, float cx, float cy, float radius) {
    const float scale = radius / (0.5f * static_cast<float>(p.bore));
    const bool burning = c.burnFraction > 0.001f && c.burnFraction < 0.999f;

    // Combustion halo, so a firing cylinder is unmistakable at a glance.
    if (burning) {
        const float glow = radius * (1.25f + 0.35f * std::sin(c.burnFraction * kPi));
        sf::CircleShape halo(glow, 48);
        halo.setOrigin({glow, glow});
        halo.setPosition({cx, cy});
        halo.setFillColor(sf::Color(255, 170, 60,
                                    static_cast<std::uint8_t>(90 * std::sin(c.burnFraction * kPi))));
        rt.draw(halo);
    }

    sf::CircleShape bore(radius, 56);
    bore.setOrigin({radius, radius});
    bore.setPosition({cx, cy});
    sf::Color gc = chargeColour(c.temperature);
    gc.a = static_cast<std::uint8_t>(std::clamp(55.0f + c.pressure * 2.4f, 55.0f, 225.0f));
    bore.setFillColor(gc);
    bore.setOutlineColor(burning ? sf::Color(255, 200, 120) : kLine);
    bore.setOutlineThickness(burning ? 3.0f : 2.0f);
    rt.draw(bore);

    auto valves = [&](const sim::ValveTiming& v, float lift01, sf::Color col, float side) {
        const float r = 0.5f * static_cast<float>(v.diameter) * scale;
        const int   n = std::max(1, v.count);
        for (int i = 0; i < n; ++i) {
            const float spread = (n == 1) ? 0.0f
                                          : (static_cast<float>(i) / (n - 1) - 0.5f) * 2.0f;
            const float vx = cx + spread * radius * 0.46f;
            const float vy = cy + side * radius * 0.40f;

            const float curtain = static_cast<float>(v.maxLift) * lift01 * scale;
            if (curtain > 0.3f) {
                const float rr = r + curtain * 0.5f;
                sf::CircleShape ring(rr, 32);
                ring.setOrigin({rr, rr});
                ring.setPosition({vx, vy});
                ring.setFillColor(sf::Color::Transparent);
                ring.setOutlineColor(sf::Color(col.r, col.g, col.b, 150));
                ring.setOutlineThickness(std::max(1.0f, curtain));
                rt.draw(ring);
            }

            sf::CircleShape headC(r, 32);
            headC.setOrigin({r, r});
            headC.setPosition({vx, vy});
            const float t = std::clamp(lift01, 0.0f, 1.0f);
            headC.setFillColor(sf::Color(static_cast<std::uint8_t>(col.r * (0.28f + 0.72f * t)),
                                         static_cast<std::uint8_t>(col.g * (0.28f + 0.72f * t)),
                                         static_cast<std::uint8_t>(col.b * (0.28f + 0.72f * t))));
            headC.setOutlineColor(col);
            headC.setOutlineThickness(1.5f);
            rt.draw(headC);
        }
    };
    valves(p.intake,  c.intakeLift,  kIntake,  -1.0f);
    valves(p.exhaust, c.exhaustLift, kExhaust,  1.0f);

    const float pr = burning ? 7.0f : 4.0f;
    sf::CircleShape plug(pr, 20);
    plug.setOrigin({pr, pr});
    plug.setPosition({cx, cy});
    plug.setFillColor(burning ? sf::Color(255, 246, 200) : sf::Color(120, 126, 140));
    rt.draw(plug);
}

// The whole engine from above: every cylinder in the block, so the firing
// order is something you watch rather than something you read.
void drawEngineTop(sf::RenderTarget& rt, Hud& hud, const sim::EngineParams& p,
                   const sim::Snapshot& s, float x, float y, float w, float h) {
    hud.text(rt, "ENGINE FROM ABOVE", x + 14.0f, y + 10.0f, 13, kDim);

    const int n = std::max(1, s.cylinderCount);
    const float top = y + 34.0f;
    const float rowH = (h - 46.0f) / n;
    const float radius = std::min(rowH * 0.40f, w * 0.26f);
    const float cx = x + w * 0.52f;

    // Block outline and camshaft lines, for context.
    sf::RectangleShape block({radius * 2.9f, h - 40.0f});
    block.setPosition({cx - radius * 1.45f, top - 6.0f});
    block.setFillColor({30, 34, 42});
    block.setOutlineColor(kLine);
    block.setOutlineThickness(1.0f);
    rt.draw(block);
    drawLine(rt, {cx - radius * 0.62f, top - 6.0f}, {cx - radius * 0.62f, top + (h - 46.0f)},
             sf::Color(60, 70, 90));
    drawLine(rt, {cx + radius * 0.62f, top - 6.0f}, {cx + radius * 0.62f, top + (h - 46.0f)},
             sf::Color(80, 60, 60));

    for (int i = 0; i < n; ++i) {
        const sim::CylinderView& c = s.cyl[static_cast<std::size_t>(i)];
        const float cy = top + rowH * (i + 0.5f);
        drawBoreTop(rt, p, c, cx, cy, radius);

        const bool burning = c.burnFraction > 0.001f && c.burnFraction < 0.999f;
        hud.text(rt, "#" + std::to_string(i + 1), x + 12.0f, cy - 18.0f, 15,
                 burning ? kAccent : kDim);
        hud.text(rt, strokeName(c.phase), x + 12.0f, cy + 2.0f, 11, kDim);
        hud.right(rt, fmt("%.0f bar", c.pressure), x + w - 12.0f, cy - 16.0f, 13,
                  burning ? kAccent : kDim);
        hud.right(rt, fmt("%.0f K", c.temperature), x + w - 12.0f, cy + 2.0f, 11, kDim);
    }
}

// ---------------------------------------------------------------------------
void drawValveDiagram(sf::RenderTarget& rt, Hud& hud, const sim::Engine& engine,
                      const sim::CylinderView& c, float x, float y, float w, float h) {
    drawPanel(rt, x, y, w, h);
    hud.text(rt, "VALVE LIFT / CRANK ANGLE", x + 12.0f, y + 8.0f, 13, kDim);

    const sim::EngineParams& p = engine.params();
    const float x0 = x + 44.0f, x1 = x + w - 12.0f;
    const float yb = y + h - 24.0f, yt = y + 34.0f;
    const float maxLift = static_cast<float>(std::max(p.intake.maxLift, p.exhaust.maxLift)) * 1000.0f;

    auto px = [&](float deg) { return x0 + (x1 - x0) * deg / 720.0f; };
    auto py = [&](float mm)  { return yb - (yb - yt) * (mm / maxLift); };

    for (int deg = 0; deg <= 720; deg += 90) {
        drawLine(rt, {px(static_cast<float>(deg)), yt}, {px(static_cast<float>(deg)), yb}, kGrid);
        if (deg % 180 == 0)
            hud.centred(rt, std::to_string(deg), px(static_cast<float>(deg)), yb + 4.0f, 12, kDim);
    }
    for (int mm = 0; mm <= static_cast<int>(maxLift); mm += 2) {
        drawLine(rt, {x0, py(static_cast<float>(mm))}, {x1, py(static_cast<float>(mm))}, kGrid);
        hud.right(rt, std::to_string(mm), x0 - 6.0f, py(static_cast<float>(mm)) - 8.0f, 12, kDim);
    }

    // Overlap: both valves off their seats at once, which is when the exhaust
    // pulse can pull fresh charge through the cylinder.
    {
        sf::VertexArray band(sf::PrimitiveType::TriangleStrip);
        for (int deg = 0; deg <= 720; ++deg) {
            const float d = static_cast<float>(deg);
            const float both = std::min(static_cast<float>(engine.intakeLiftAt(d)),
                                        static_cast<float>(engine.exhaustLiftAt(d))) * 1000.0f;
            if (both <= 0.001f) continue;
            sf::Vertex v0, v1;
            v0.position = {px(d), yb};
            v1.position = {px(d), py(both)};
            v0.color = v1.color = sf::Color(255, 214, 120, 70);
            band.append(v0); band.append(v1);
        }
        rt.draw(band);
    }

    auto curve = [&](bool intake, sf::Color col) {
        sf::VertexArray line(sf::PrimitiveType::LineStrip);
        for (int deg = 0; deg <= 720; deg += 2) {
            const float d = static_cast<float>(deg);
            const float mm = static_cast<float>(intake ? engine.intakeLiftAt(d)
                                                       : engine.exhaustLiftAt(d)) * 1000.0f;
            sf::Vertex v;
            v.position = {px(d), py(mm)};
            v.color = col;
            line.append(v);
        }
        rt.draw(line);
    };
    curve(false, kExhaust);
    curve(true,  kIntake);

    drawLine(rt, {px(c.phase), yt}, {px(c.phase), yb}, sf::Color(255, 255, 255, 120));

    hud.text(rt, fmt("IVO %.0f", static_cast<float>(p.intake.openDeg)) +
                 fmt("  IVC %.0f", static_cast<float>(p.intake.closeDeg)),
             x + 200.0f, y + 8.0f, 12, kIntake);
    hud.text(rt, fmt("EVO %.0f", static_cast<float>(p.exhaust.openDeg)) +
                 fmt("  EVC %.0f", static_cast<float>(p.exhaust.closeDeg)),
             x + 330.0f, y + 8.0f, 12, kExhaust);
}

void drawTacho(sf::RenderTarget& rt, Hud& hud, float cx, float cy, float radius,
               const sim::Snapshot& s, float redline) {
    const float start = 140.0f, sweep = 260.0f;
    const float maxRpm = 9000.0f;
    auto angleFor = [&](float rpm) {
        return (start + sweep * std::clamp(rpm / maxRpm, 0.0f, 1.0f)) * kPi / 180.0f;
    };

    for (int r = 0; r <= 9000; r += 500) {
        const bool major = (r % 1000) == 0;
        const float ang = angleFor(static_cast<float>(r));
        const float r0 = radius - (major ? 18.0f : 10.0f);
        const sf::Color col = (r >= redline) ? kAlert : (major ? kText : kDim);
        drawLine(rt, {cx + r0 * std::cos(ang), cy + r0 * std::sin(ang)},
                     {cx + radius * std::cos(ang), cy + radius * std::sin(ang)}, col);
        if (major) {
            const float lr = radius - 34.0f;
            hud.centred(rt, std::to_string(r / 1000),
                        cx + lr * std::cos(ang), cy + lr * std::sin(ang) - 10.0f, 15, col);
        }
    }

    sf::VertexArray arc(sf::PrimitiveType::TriangleStrip);
    const int steps = 96;
    for (int i = 0; i <= steps; ++i) {
        const float f = static_cast<float>(i) / steps;
        const float rpm = f * std::clamp(s.rpm, 0.0f, maxRpm);
        const float ang = angleFor(rpm);
        const sf::Color col = rpm >= redline ? kAlert : kAccent;
        sf::Vertex v0, v1;
        v0.position = {cx + (radius - 6.0f) * std::cos(ang), cy + (radius - 6.0f) * std::sin(ang)};
        v1.position = {cx + (radius + 2.0f) * std::cos(ang), cy + (radius + 2.0f) * std::sin(ang)};
        v0.color = v1.color = col;
        arc.append(v0); arc.append(v1);
    }
    rt.draw(arc);

    const float ang = angleFor(s.rpm);
    sf::RectangleShape needle({radius - 26.0f, 4.0f});
    needle.setOrigin({6.0f, 2.0f});
    needle.setPosition({cx, cy});
    needle.setRotation(sf::degrees(ang * 180.0f / kPi));
    needle.setFillColor(sf::Color(240, 240, 245));
    rt.draw(needle);

    sf::CircleShape hub(9.0f);
    hub.setOrigin({9.0f, 9.0f});
    hub.setPosition({cx, cy});
    hub.setFillColor(kLine);
    rt.draw(hub);

    hud.centred(rt, fmt("%.0f", s.rpm), cx, cy + 20.0f, 32, kText);
    hud.centred(rt, "rpm", cx, cy + 62.0f, 13, kDim);
}

// Gear indicator: neutral plus six ratios, current one lit.
void drawGearbox(sf::RenderTarget& rt, Hud& hud, const sim::Snapshot& s,
                 float x, float y, float w) {
    hud.text(rt, "TRANSMISSION", x, y, 13, kDim);

    const int slots = s.gearCount + 1;                 // N, 1..6
    const float bw = (w - (slots - 1) * 6.0f) / slots;
    for (int i = 0; i < slots; ++i) {
        const bool active = (i == s.gear);
        sf::RectangleShape box({bw, 34.0f});
        box.setPosition({x + i * (bw + 6.0f), y + 20.0f});
        box.setFillColor(active ? kAccent : sf::Color(38, 42, 52));
        box.setOutlineColor(active ? kAccent : kLine);
        box.setOutlineThickness(1.0f);
        rt.draw(box);
        hud.centred(rt, i == 0 ? "N" : std::to_string(i),
                    x + i * (bw + 6.0f) + bw * 0.5f, y + 26.0f, 18,
                    active ? sf::Color(20, 22, 28) : kDim);
    }

    hud.text(rt, "ROAD SPEED", x, y + 68.0f, 13, kDim);
    hud.right(rt, fmt("%.0f km/h", s.speedKph), x + w, y + 64.0f, 22, kText);

    hud.text(rt, "CLUTCH", x, y + 100.0f, 13, kDim);
    sf::RectangleShape bg({w * 0.55f, 10.0f});
    bg.setPosition({x + w * 0.45f, y + 100.0f});
    bg.setFillColor({40, 44, 54});
    rt.draw(bg);
    sf::RectangleShape fg({w * 0.55f * std::clamp(s.clutchLock, 0.0f, 1.0f), 10.0f});
    fg.setPosition({x + w * 0.45f, y + 100.0f});
    fg.setFillColor(std::abs(s.clutchSlip) > 120.0f ? kAlert : kGood);
    rt.draw(fg);
    hud.text(rt, s.gear == 0 ? "neutral"
                             : (std::abs(s.clutchSlip) > 120.0f ? "slipping" : "locked"),
             x, y + 118.0f, 12, kDim);
    hud.right(rt, fmt("%.0f N m at the wheels", s.wheelTorque), x + w, y + 118.0f, 12, kDim);
}

void drawBar(sf::RenderTarget& rt, Hud& hud, const std::string& label, float value01,
             float x, float y, float w, sf::Color col) {
    hud.text(rt, label, x, y - 17.0f, 13, kDim);
    sf::RectangleShape bg({w, 10.0f});
    bg.setPosition({x, y});
    bg.setFillColor({40, 44, 54});
    rt.draw(bg);
    sf::RectangleShape fg({w * std::clamp(value01, 0.0f, 1.0f), 10.0f});
    fg.setPosition({x, y});
    fg.setFillColor(col);
    rt.draw(fg);
}

void drawPressureTrace(sf::RenderTarget& rt, Hud& hud, const sim::Engine& engine,
                       const sim::CylinderView& c, float x, float y, float w, float h) {
    drawPanel(rt, x, y, w, h);
    hud.text(rt, "CYLINDER PRESSURE / CRANK ANGLE", x + 12.0f, y + 8.0f, 13, kDim);

    const float maxBar = 90.0f;
    const float x0 = x + 44.0f, x1 = x + w - 12.0f;
    const float yb = y + h - 24.0f, yt = y + 34.0f;
    for (int b = 0; b <= 3; ++b) {
        const float gy = yb - (yb - yt) * b / 3.0f;
        drawLine(rt, {x0, gy}, {x1, gy}, kGrid);
        hud.right(rt, std::to_string(static_cast<int>(maxBar * b / 3)), x0 - 6.0f, gy - 8.0f, 12, kDim);
    }
    for (int deg = 0; deg <= 720; deg += 180)
        drawLine(rt, {x0 + (x1 - x0) * deg / 720.0f, yt},
                     {x0 + (x1 - x0) * deg / 720.0f, yb}, kGrid);

    sf::VertexArray curve(sf::PrimitiveType::LineStrip);
    for (std::size_t i = 0; i < sim::Engine::kTraceBins; ++i) {
        const float bar = std::clamp(engine.traceAt(i), 0.0f, maxBar);
        sf::Vertex v;
        v.position = {x0 + (x1 - x0) * i / (sim::Engine::kTraceBins - 1),
                      yb - (yb - yt) * (bar / maxBar)};
        v.color = kAccent;
        curve.append(v);
    }
    rt.draw(curve);
    drawLine(rt, {x0 + (x1 - x0) * c.phase / 720.0f, yt},
                 {x0 + (x1 - x0) * c.phase / 720.0f, yb}, sf::Color(255, 255, 255, 120));
}

void drawScope(sf::RenderTarget& rt, Hud& hud, const audio::EngineSound& sound,
               float x, float y, float w, float h) {
    drawPanel(rt, x, y, w, h);
    hud.text(rt, "EXHAUST WAVEFORM", x + 12.0f, y + 8.0f, 13, kDim);

    const std::size_t n = audio::EngineSound::kScope;
    const std::size_t head = sound.scopeHead();
    sf::VertexArray wave(sf::PrimitiveType::LineStrip);
    for (std::size_t i = 0; i < n; ++i) {
        const float v = sound.scopeAt((head + 1 + i) % n);
        sf::Vertex vert;
        vert.position = {x + 12.0f + (w - 24.0f) * i / (n - 1),
                         y + h * 0.6f - std::clamp(v, -1.0f, 1.0f) * (h * 0.3f)};
        vert.color = kIntake;
        wave.append(vert);
    }
    rt.draw(wave);
}

} // namespace

int main() {
    sim::EngineParams params;                 // ~2.0 L inline-four, see Engine.h
    sim::Engine engine(params);
    audio::EngineSound sound(engine);

    sf::RenderWindow window(sf::VideoMode({kWinW, kWinH}), "Enginio2D - engine simulator");
    window.setFramerateLimit(60);

    const sf::Font font = loadFont();
    Hud hud(font);

    sound.setVolume(60.0f);
    sound.play();                             // the physics runs on the audio thread

    float throttleCmd = 0.0f;
    float brakeCmd = 0.0f;
    bool  ignition = true;
    int   gearShown = 0;

    sf::Clock clock;
    while (window.isOpen()) {
        const float dt = clock.restart().asSeconds();

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
            if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
                using K = sf::Keyboard::Key;
                switch (key->code) {
                    case K::Escape: window.close(); break;
                    case K::I: ignition = !ignition; sound.setIgnition(ignition); break;
                    case K::E: sound.requestGear(std::min(gearShown + 1, 6)); break;
                    case K::Q: sound.requestGear(std::max(gearShown - 1, 0)); break;
                    case K::N: case K::Num0: sound.requestGear(0); break;
                    case K::Num1: sound.requestGear(1); break;
                    case K::Num2: sound.requestGear(2); break;
                    case K::Num3: sound.requestGear(3); break;
                    case K::Num4: sound.requestGear(4); break;
                    case K::Num5: sound.requestGear(5); break;
                    case K::Num6: sound.requestGear(6); break;
                    default: break;
                }
            }
        }

        const bool wantThrottle = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W) ||
                                  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);
        const bool wantBrake = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down) ||
                               sf::Keyboard::isKeyPressed(sf::Keyboard::Key::B);
        // A pedal is not a switch: ramp so the manifold has time to fill.
        throttleCmd += ((wantThrottle ? 1.0f : 0.0f) - throttleCmd) * std::min(1.0f, dt * 9.0f);
        brakeCmd    += ((wantBrake ? 1.0f : 0.0f) - brakeCmd) * std::min(1.0f, dt * 8.0f);
        sound.setThrottle(throttleCmd);
        sound.setBrake(brakeCmd);
        sound.setStarter(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S));

        const sim::Snapshot s = sound.snapshot();
        const sim::CylinderView& c1 = s.cyl[0];
        gearShown = s.gear;

        window.clear(kBg);

        // Left: section through cylinder 1.
        drawPanel(window, 12.0f, 12.0f, 380.0f, 560.0f);
        hud.text(window, "CYLINDER 1  SECTION", 26.0f, 22.0f, 13, kDim);
        drawCylinderSide(window, hud, params, c1, 202.0f, 70.0f);

        // Middle: the whole engine from above.
        drawPanel(window, 400.0f, 12.0f, 300.0f, 560.0f);
        drawEngineTop(window, hud, params, s, 400.0f, 12.0f, 300.0f, 560.0f);

        // Right: instruments.
        drawPanel(window, 708.0f, 12.0f, 720.0f, 560.0f);
        drawTacho(window, hud, 856.0f, 190.0f, 150.0f, s, static_cast<float>(params.redline));
        drawGearbox(window, hud, s, 726.0f, 360.0f, 300.0f);

        const float rx = 1064.0f, rw = 340.0f;
        hud.text(window, "OUTPUT", rx, 34.0f, 13, kDim);
        hud.right(window, fmt("%.1f", s.torque), rx + 190.0f, 50.0f, 28, kText);
        hud.text(window, "N m", rx + 200.0f, 64.0f, 14, kDim);
        hud.right(window, fmt("%.1f", s.power), rx + 190.0f, 86.0f, 28, kText);
        hud.text(window, "kW", rx + 200.0f, 100.0f, 14, kDim);

        float ry = 142.0f;
        auto row = [&](const char* label, const std::string& value, sf::Color col = kText) {
            hud.readout(window, label, value, rx, ry, rw, col);
            ry += 25.0f;
        };
        row("MANIFOLD PRESSURE", fmt("%.1f kPa", s.manifoldPressure));
        row("EXHAUST BACK PRESSURE", fmt("%.1f kPa", s.backPressure));
        row("VOLUMETRIC EFFICIENCY", fmt("%.0f %%", s.volumetricEff * 100.0f));
        row("RESIDUAL GAS", fmt("%.0f %%", s.residualFraction * 100.0f));
        row("PEAK CYL. PRESSURE", fmt("%.1f bar", s.peakPressure));
        row("EXHAUST GAS TEMP", fmt("%.0f K", s.exhaustTemp));
        row("SPARK ADVANCE", fmt("%.1f deg BTDC", s.sparkAdvance));
        row("AIR / FUEL RATIO", fmt("%.1f : 1", s.afr));
        row("FRICTION FMEP", fmt("%.2f bar", s.fmep));
        row("IDLE VALVE", fmt("%.0f %%", s.idleValve * 100.0f));
        row("CLUTCH SLIP", fmt("%.0f rpm", s.clutchSlip));

        drawBar(window, hud, "THROTTLE", s.throttle, rx, 440.0f, rw, kAccent);
        drawBar(window, hud, "BRAKE", s.brake, rx, 480.0f, rw, kExhaust);

        hud.text(window, ignition ? "IGNITION ON" : "IGNITION OFF", 726.0f, 500.0f, 15,
                 ignition ? kGood : kAlert);
        if (s.rpm > params.redline - 100.0)
            hud.text(window, "REV LIMITER", 726.0f, 522.0f, 15, kAlert);

        hud.text(window,
                 "W  throttle    DOWN  brake    S  starter    Q / E  shift    0-6  gear    I  ignition    ESC  quit",
                 726.0f, 546.0f, 12, kDim);

        // Bottom row of plots.
        drawValveDiagram(window, hud, engine, c1, 12.0f, 584.0f, 480.0f, 216.0f);
        drawPressureTrace(window, hud, engine, c1, 500.0f, 584.0f, 480.0f, 216.0f);
        drawScope(window, hud, sound, 988.0f, 584.0f, 440.0f, 216.0f);

        window.display();
    }

    sound.stop();
    return 0;
}
