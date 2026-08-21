#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "audio/EngineSound.h"
#include "input/Gamepad.h"
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

// ---------------------------------------------------------------------------
// Drawing helpers. The toolkit gives axis-aligned rectangles and raw vertex
// arrays; an engine is neither, so these fill the gap: filled polygons, rounded
// bodies, thick lines at any angle, and a coil spring.
// ---------------------------------------------------------------------------
// Blend two palette colours, so parts can be lifted away from the background
// without hard-coding shades that would ignore the theme.
sf::Color mix(sf::Color a, sf::Color b, float t) {
    const auto ch = [&](std::uint8_t p, std::uint8_t q) {
        return static_cast<std::uint8_t>(p + (q - p) * std::clamp(t, 0.0f, 1.0f));
    };
    return sf::Color(ch(a.r, b.r), ch(a.g, b.g), ch(a.b, b.b));
}

void poly(ui::Ui& u, const std::vector<sf::Vector2f>& pts, sf::Color fill) {
    if (pts.size() < 3) return;
    sf::VertexArray va(sf::PrimitiveType::TriangleFan, pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        va[i].position = pts[i];
        va[i].color = fill;
    }
    u.vertices(va);
}

void thick(ui::Ui& u, sf::Vector2f a, sf::Vector2f b, float width, sf::Color col) {
    const sf::Vector2f d{b.x - a.x, b.y - a.y};
    const float len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len < 1e-4f) return;
    const sf::Vector2f n{-d.y / len * width * 0.5f, d.x / len * width * 0.5f};
    poly(u, {{a.x + n.x, a.y + n.y}, {b.x + n.x, b.y + n.y},
             {b.x - n.x, b.y - n.y}, {a.x - n.x, a.y - n.y}}, col);
}

void outline(ui::Ui& u, const std::vector<sf::Vector2f>& pts, sf::Color col, float width) {
    for (std::size_t i = 0; i < pts.size(); ++i)
        thick(u, pts[i], pts[(i + 1) % pts.size()], width, col);
}

// A rectangle with rounded corners, as a point list so it can be filled,
// outlined or transformed like anything else.
std::vector<sf::Vector2f> roundRect(float x, float y, float w, float h, float r) {
    r = std::min(r, 0.5f * std::min(w, h));
    std::vector<sf::Vector2f> pts;
    const sf::Vector2f corner[4] = {{x + w - r, y + r}, {x + w - r, y + h - r},
                                    {x + r, y + h - r}, {x + r, y + r}};
    const float start[4] = {-90.0f, 0.0f, 90.0f, 180.0f};
    for (int k = 0; k < 4; ++k)
        for (int i = 0; i <= 4; ++i) {
            const float a = (start[k] + 90.0f * i / 4.0f) * kPi / 180.0f;
            pts.push_back({corner[k].x + r * std::cos(a), corner[k].y + r * std::sin(a)});
        }
    return pts;
}

void roundRectFill(ui::Ui& u, float x, float y, float w, float h, float r, sf::Color fill) {
    poly(u, roundRect(x, y, w, h, r), fill);
}

void disc(ui::Ui& u, sf::Vector2f c, float r, sf::Color fill,
          sf::Color line = sf::Color::Transparent, float width = 0.0f) {
    sf::CircleShape s(r, 28);
    s.setOrigin({r, r});
    s.setPosition(c);
    s.setFillColor(fill);
    s.setOutlineColor(line);
    s.setOutlineThickness(width);
    u.circleShape(s);
}

// A valve spring: a zigzag between two points that closes up as the valve
// lifts, because that is exactly what a spring does and it is the clearest
// possible indication of how hard the valvetrain is working.
void spring(ui::Ui& u, sf::Vector2f top, sf::Vector2f bottom, float halfWidth,
            int turns, float width, sf::Color col) {
    const sf::Vector2f d{bottom.x - top.x, bottom.y - top.y};
    const float len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len < 2.0f || turns < 1) return;
    const sf::Vector2f along{d.x / len, d.y / len};
    const sf::Vector2f across{-along.y, along.x};
    sf::Vector2f prev = top;
    const int steps = turns * 2;
    for (int i = 1; i <= steps; ++i) {
        const float t = static_cast<float>(i) / steps;
        const float side = (i % 2 == 0) ? -1.0f : 1.0f;
        const sf::Vector2f next{top.x + along.x * len * t + across.x * halfWidth * side,
                                top.y + along.y * len * t + across.y * halfWidth * side};
        thick(u, prev, next, width, col);
        prev = next;
    }
}

void panel(ui::Ui& u, float x, float y, float w, float h) {
    const ui::Palette& pal = u.pal();
    roundRectFill(u, x, y, w, h, 8.0f, pal.panel);
    outline(u, roundRect(x, y, w, h, 8.0f), pal.line, 1.0f);
}

// A panel with its title in the frame, so every section is labelled the same
// way instead of each caller placing its own text.
void panelTitled(ui::Ui& u, float x, float y, float w, float h, const std::string& title) {
    const ui::Palette& pal = u.pal();
    panel(u, x, y, w, h);
    u.rect(x + 14.0f, y + 15.0f, 3.0f, 11.0f, pal.accent);
    u.text(title, x + 24.0f, y + 13.0f, 13, pal.dim);
}

// ---------------------------------------------------------------------------
// The engine in section: one barrel per bank, at the bank angle the design
// actually specifies, around a shared crank.
//
// A cross-section is the honest picture for this. The cylinders of an inline
// engine sit one behind another and show as a single barrel; a V shows two, a
// W four. So what is drawn is one representative cylinder per bank, which is
// both what you would see if you cut the engine open and what makes the layout
// legible at a glance.
// ---------------------------------------------------------------------------
namespace {

// Everything about one cylinder is drawn in its own frame: y runs up the bore
// away from the crank, x across it, and the whole thing is rotated to the bank.
struct Frame {
    sf::Vector2f origin;   // crank centre, in screen space
    float sinA = 0.0f, cosA = 1.0f;

    sf::Vector2f pt(float across, float along) const {
        return {origin.x + across * cosA + along * sinA,
                origin.y + across * sinA - along * cosA};
    }
};

} // namespace

void drawEngineSection(ui::Ui& u, const sim::EngineParams& p, const sim::Snapshot& s,
                       float x, float y, float w, float h) {
    const ui::Palette& pal = u.pal();
    const int banks = std::clamp(p.banks, 1, 4);
    const double bankAngle = p.banks > 1 ? p.bankAngle : 0.0;

    // Parts have to separate from the panel and from each other, so the block
    // is lifted away from the background and everything structural is outlined.
    const sf::Color barrelCol = mix(pal.block, pal.cover, 0.45f);
    const sf::Color headCol   = mix(pal.cover, pal.accent, 0.18f);
    const sf::Color caseCol   = mix(pal.block, pal.bg, 0.35f);
    const sf::Color edge      = mix(pal.line, pal.text, 0.25f);

    int rep[4] = {0, 0, 0, 0};
    for (int b = 0; b < banks; ++b)
        for (int i = 0; i < s.cylinderCount && i < static_cast<int>(p.cylinderBank.size()); ++i)
            if (p.cylinderBank[static_cast<std::size_t>(i)] == b) { rep[b] = i; break; }

    // ---- Fit ---------------------------------------------------------------
    // Everything is sized from the real geometry, then scaled once so the whole
    // engine sits inside the panel with room left for the readout underneath.
    const float halfBank = static_cast<float>(bankAngle) * 0.5f * kPi / 180.0f;
    const float boreM = static_cast<float>(p.bore);
    const float upM   = static_cast<float>(p.rodLength + 0.5 * p.stroke) + boreM * 1.45f;
    const float downM = static_cast<float>(0.5 * p.stroke) + boreM * 0.62f;
    const float needY = upM * std::cos(halfBank) + downM;
    const float needX = 2.0f * (upM * std::sin(halfBank) + boreM * 0.85f);

    const float topPad = 46.0f, botPad = 78.0f;
    const float scale = std::min((h - topPad - botPad) / std::max(needY, 1e-3f),
                                 (w - 34.0f) / std::max(needX, 1e-3f));

    const float a    = 0.5f * static_cast<float>(p.stroke) * scale;
    const float rod  = static_cast<float>(p.rodLength) * scale;
    const float bore = boreM * scale;

    // Centre what is actually drawn in the space available, rather than hanging
    // it from the top: a wide V is limited by the panel width, so without this
    // it floats with all the slack underneath it.
    const float extentUp   = upM * std::cos(halfBank) * scale;
    const float extentDown = downM * scale;
    const float slack = std::max(0.0f, (h - topPad - botPad) - (extentUp + extentDown));
    const sf::Vector2f crank{x + w * 0.5f, y + topPad + slack * 0.5f + extentUp};

    // ---- Crankcase ---------------------------------------------------------
    const float caseW = std::max(bore * 1.5f, (a + bore * 0.5f) * 2.2f);
    const float caseH = (a + bore * 0.55f) * 1.9f;
    roundRectFill(u, crank.x - caseW * 0.5f, crank.y - caseH * 0.30f, caseW, caseH,
                  caseH * 0.30f, caseCol);
    outline(u, roundRect(crank.x - caseW * 0.5f, crank.y - caseH * 0.30f, caseW, caseH,
                         caseH * 0.30f), edge, 1.5f);

    for (int b = 0; b < banks; ++b) {
        const sim::CylinderView& c = s.cyl[static_cast<std::size_t>(rep[b])];
        const float bankDeg = banks == 1 ? 0.0f
                            : static_cast<float>(bankAngle) *
                              (static_cast<float>(b) / (banks - 1) - 0.5f);
        Frame f;
        f.origin = crank;
        f.sinA = std::sin(bankDeg * kPi / 180.0f);
        f.cosA = std::cos(bankDeg * kPi / 180.0f);

        const float theta = c.phase * kPi / 180.0f;
        const float sinT = std::sin(theta), cosT = std::cos(theta);
        const float pin = a * cosT + std::sqrt(std::max(1.0f, rod * rod - a * a * sinT * sinT));
        const float pistonH = std::max(16.0f, bore * 0.40f);
        const float deck = rod + a + pistonH * 0.55f;
        // The barrel runs down into the crankcase rather than stopping short of
        // it, which is what stops it looking like it is floating.
        const float skirt = -bore * 0.12f;
        const float halfB = bore * 0.5f;
        const float wall  = std::max(6.0f, bore * 0.13f);

        // ---- Barrel ------------------------------------------------------
        for (int side = -1; side <= 1; side += 2) {
            const std::vector<sf::Vector2f> quad = {
                f.pt(side * halfB, skirt), f.pt(side * (halfB + wall), skirt),
                f.pt(side * (halfB + wall), deck), f.pt(side * halfB, deck)};
            poly(u, quad, barrelCol);
            outline(u, quad, edge, 1.2f);
        }
        // Fins, on the outside of the bank only: on a V the inner ones would
        // reach across into the other barrel, which reads as a mess rather than
        // as cooling.
        const int fins = std::max(3, static_cast<int>((deck - skirt) / (bore * 0.22f)));
        const float outerSide = banks == 1 ? 0.0f : (bankDeg >= 0.0f ? 1.0f : -1.0f);
        for (int i = 1; i < fins; ++i) {
            const float fy = skirt + (deck - skirt) * i / fins;
            const float fw = halfB + wall * 1.9f;
            const float inner = halfB + wall * 0.2f;
            if (outerSide <= 0.0f)
                thick(u, f.pt(-fw, fy), f.pt(-inner, fy), 3.0f, pal.cover);
            if (outerSide >= 0.0f)
                thick(u, f.pt(fw, fy), f.pt(inner, fy), 3.0f, pal.cover);
        }

        // ---- Charge ------------------------------------------------------
        const float crown = pin + pistonH * 0.45f;
        if (deck > crown + 1.0f) {
            sf::Color gc = chargeColour(c.temperature);
            gc.a = static_cast<std::uint8_t>(std::clamp(90.0f + c.pressure * 3.0f, 90.0f, 250.0f));
            poly(u, {f.pt(-halfB, crown), f.pt(halfB, crown),
                     f.pt(halfB, deck), f.pt(-halfB, deck)}, gc);
            if (c.knock > 0.02f)
                poly(u, {f.pt(-halfB, crown), f.pt(halfB, crown),
                         f.pt(halfB, deck), f.pt(-halfB, deck)},
                     sf::Color(255, 255, 255, static_cast<std::uint8_t>(180 * c.knock)));
        }

        // ---- Head --------------------------------------------------------
        const float headTop = deck + bore * 0.78f;
        const std::vector<sf::Vector2f> head = {
            f.pt(-halfB - wall, deck), f.pt(halfB + wall, deck),
            f.pt(halfB + wall * 0.55f, headTop), f.pt(-halfB - wall * 0.55f, headTop)};
        poly(u, head, headCol);
        outline(u, head, edge, 1.4f);

        // Valves hang from the head into the bore, springs above them.
        auto valve = [&](float ax, float lift01, sf::Color col) {
            const float travel = bore * 0.20f * std::clamp(lift01, 0.0f, 1.0f);
            const float seat = deck - travel;
            const float retainer = headTop - bore * 0.16f;
            thick(u, f.pt(ax, seat), f.pt(ax, retainer + bore * 0.04f), bore * 0.06f, pal.metal);
            const float vr = bore * 0.20f;
            const std::vector<sf::Vector2f> vhead = {
                f.pt(ax - vr, seat + bore * 0.06f), f.pt(ax + vr, seat + bore * 0.06f),
                f.pt(ax + vr * 0.5f, seat - bore * 0.02f), f.pt(ax - vr * 0.5f, seat - bore * 0.02f)};
            poly(u, vhead, col);
            spring(u, f.pt(ax, retainer), f.pt(ax, seat + bore * 0.14f), bore * 0.12f, 5,
                   std::max(1.8f, bore * 0.03f), mix(col, pal.text, 0.15f));
            thick(u, f.pt(ax - vr * 0.75f, retainer), f.pt(ax + vr * 0.75f, retainer),
                  bore * 0.055f, pal.metal);
        };
        valve(-bore * 0.27f, c.intakeLift, pal.intake);
        valve(bore * 0.27f, c.exhaustLift, pal.exhaust);

        // Cam, half crank speed, sunk into the head rather than floating above.
        const float camY = headTop - bore * 0.02f;
        const float camR = bore * 0.17f;
        const float camA = theta * 0.5f;
        disc(u, f.pt(0.0f, camY), camR, pal.metal, edge, 1.2f);
        const float lobe = camR * 1.75f;
        poly(u, {f.pt(camR * 0.45f * std::cos(camA + 1.9f), camY + camR * 0.45f * std::sin(camA + 1.9f)),
                 f.pt(lobe * std::cos(camA), camY + lobe * std::sin(camA)),
                 f.pt(camR * 0.45f * std::cos(camA - 1.9f), camY + camR * 0.45f * std::sin(camA - 1.9f))},
             pal.metal);

        // ---- Piston ------------------------------------------------------
        const float pw = bore - 4.0f;
        const std::vector<sf::Vector2f> piston = {
            f.pt(-pw * 0.5f, pin - pistonH * 0.55f), f.pt(pw * 0.5f, pin - pistonH * 0.55f),
            f.pt(pw * 0.5f, pin + pistonH * 0.45f), f.pt(-pw * 0.5f, pin + pistonH * 0.45f)};
        poly(u, piston, sf::Color(214, 220, 232));
        outline(u, piston, sf::Color(120, 126, 142), 1.4f);
        for (int r = 0; r < 2; ++r) {
            const float ry = pin + pistonH * (0.30f - 0.15f * r);
            thick(u, f.pt(-pw * 0.5f, ry), f.pt(pw * 0.5f, ry), 2.5f, sf::Color(120, 126, 142));
        }

        // ---- Rod ----------------------------------------------------------
        const sf::Vector2f journal = f.pt(a * sinT, a * cosT);
        const sf::Vector2f pinPt = f.pt(0.0f, pin);
        thick(u, pinPt, journal, bore * 0.16f, pal.metal);
        disc(u, pinPt, bore * 0.105f, sf::Color(96, 102, 118), edge, 1.2f);
        disc(u, journal, bore * 0.15f, pal.metal, edge, 1.2f);
    }

    // ---- Crank -------------------------------------------------------------
    // Drawn after the rods so the webs read as being in front, and shaped like
    // a counterweight rather than a plain disc: the mass is opposite the throw,
    // which is the whole point of it.
    {
        const sim::CylinderView& c0 = s.cyl[static_cast<std::size_t>(rep[0])];
        const float bankDeg = banks == 1 ? 0.0f
                            : static_cast<float>(bankAngle) * (0.0f / std::max(1, banks - 1) - 0.5f);
        Frame f;
        f.origin = crank;
        f.sinA = std::sin(bankDeg * kPi / 180.0f);
        f.cosA = std::cos(bankDeg * kPi / 180.0f);
        const float theta = c0.phase * kPi / 180.0f;
        const float sinT = std::sin(theta), cosT = std::cos(theta);

        const float cwR = a * 0.80f + bore * 0.16f;
        const sf::Vector2f cwC = f.pt(-a * sinT * 0.55f, -a * cosT * 0.55f);
        std::vector<sf::Vector2f> web;
        // A fan opposite the throw, closed off with a narrow waist by the axis.
        const float base = std::atan2(cwC.y - crank.y, cwC.x - crank.x);
        for (int i = 0; i <= 16; ++i) {
            const float ang = base - 1.35f + 2.70f * i / 16.0f;
            web.push_back({crank.x + cwR * std::cos(ang), crank.y + cwR * std::sin(ang)});
        }
        web.push_back({crank.x + bore * 0.22f * std::cos(base + kPi * 0.5f),
                       crank.y + bore * 0.22f * std::sin(base + kPi * 0.5f)});
        web.push_back({crank.x + bore * 0.22f * std::cos(base - kPi * 0.5f),
                       crank.y + bore * 0.22f * std::sin(base - kPi * 0.5f)});
        poly(u, web, sf::Color(118, 125, 140));
        outline(u, web, edge, 1.4f);
        disc(u, crank, bore * 0.19f, sf::Color(158, 166, 182), edge, 1.4f);
        disc(u, f.pt(a * sinT, a * cosT), bore * 0.08f, pal.accent);
    }

    // ---- Readout ------------------------------------------------------------
    const sim::CylinderView& c0 = s.cyl[0];
    const float ly = y + h - botPad + 8.0f;
    u.centred(strokeName(c0.phase), x + w * 0.5f, ly, 17, pal.accent);
    u.centred(fmt("%.0f deg", c0.phase) + fmt("    %.1f bar", c0.pressure) +
              fmt("    %.0f K", c0.temperature),
              x + w * 0.5f, ly + 24.0f, 12, pal.dim);
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
    // The panel frame carries the title; the bank labels get their own line so
    // the two cannot collide on a V.
    u.rect(x + 14.0f, y + 15.0f, 3.0f, 11.0f, pal.accent);
    u.text("ENGINE FROM ABOVE", x + 24.0f, y + 13.0f, 13, pal.dim);

    const int n = std::max(1, s.cylinderCount);
    const int banks = std::clamp(p.banks, 1, 4);
    const int rows = (n + banks - 1) / banks;
    // Room for the panel title and the bank labels above the bores.
    const float top = y + 50.0f;
    const float rowH = (h - 68.0f) / rows;
    const float colW = (w - 40.0f) / banks;
    const float radius = std::min(rowH * 0.40f, colW * 0.40f);

    for (int b = 0; b < banks; ++b) {
        const float cx = x + 20.0f + colW * (b + 0.5f);
        u.rect(cx - radius * 1.45f, top - 6.0f, radius * 2.9f, h - 62.0f,
               pal.block, pal.line, 1.0f);
        u.line({cx - radius * 0.62f, top - 6.0f}, {cx - radius * 0.62f, top + h - 68.0f},
               sf::Color(60, 70, 90));
        u.line({cx + radius * 0.62f, top - 6.0f}, {cx + radius * 0.62f, top + h - 68.0f},
               sf::Color(80, 60, 60));
        if (banks > 1)
            u.centred(std::string("BANK ") + static_cast<char>('A' + b), cx, y + 32.0f,
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

    // The pedal and what the disc is doing about it are two different things,
    // and with a manual clutch both matter: the pedal is where your foot is,
    // the clamp is how much torque the disc can hold there.
    u.text("CLUTCH PEDAL", x, y + 100.0f, 13, pal.dim);
    u.rect(x + w * 0.45f, y + 100.0f, w * 0.55f, 10.0f, pal.grid);
    u.rect(x + w * 0.45f, y + 100.0f, w * 0.55f * std::clamp(s.clutchPedal, 0.0f, 1.0f),
           10.0f, pal.intake);

    u.text("CLAMP", x, y + 118.0f, 13, pal.dim);
    u.rect(x + w * 0.45f, y + 118.0f, w * 0.55f, 10.0f, pal.grid);
    u.rect(x + w * 0.45f, y + 118.0f, w * 0.55f * std::clamp(s.clutchLock, 0.0f, 1.0f),
           10.0f, std::abs(s.clutchSlip) > 120.0f ? pal.alert : pal.good);

    const char* state = s.gearGrind > 0.05f ? "clutch it to change gear"
                      : s.gear == 0         ? "neutral"
                      : s.clutchLock < 0.02f ? "disengaged"
                      : std::abs(s.clutchSlip) > 120.0f ? "slipping"
                                                        : "locked";
    u.text(state, x, y + 136.0f, 12,
           s.gearGrind > 0.05f ? pal.alert : pal.dim);
    u.right(fmt("%.0f N m at the wheels", s.wheelTorque), x + w, y + 136.0f, 12, pal.dim);
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
        // Three things only a driver-operated clutch can produce: an engine
        // dragged to a stop, tyres turning faster than the road, and a lever
        // pushed at a gearset that is still spinning.
        {"STALL", s.stalled, pal.alert},
        {"SPIN",  s.wheelSlip > 0.30f, pal.alert},
        {"GRIND", s.gearGrind > 0.05f, pal.alert},
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

    // Controllers. Whatever was bound last time wins over the guess made from
    // looking at the device, so a pad that needed correcting stays corrected.
    input::Gamepad pad;
    const std::string padPath = input::bindingsPath();
    if (input::loadBindings(pad.bindings(), padPath)) editor.padLoaded = true;
    pad.start();

    sound.setVolume(60.0f);
    sound.play();                             // the physics runs on the audio thread

    // Keyboard pedals. A key is a switch and a pedal is not, so each one is
    // slewed at the rate a foot moves; an analog control is read straight and
    // must not be slewed, which is why they are kept apart and combined below.
    float throttleKey = 0.0f;
    float brakeKey = 0.0f;
    float clutchKey = 0.0f;
    float throttleCmd = 0.0f;
    float brakeCmd = 0.0f;
    float clutchCmd = 0.0f;
    bool  ignition = true;
    bool  fullscreen = false;
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

        pad.update(dt);

        while (const std::optional event = window.pollEvent()) {
            pad.handleEvent(*event);
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
                if (key->code == K::Tab) editor.visible = !editor.visible;
                if (key->code == K::F11) {
                    // Recreating the window is the only way to change between
                    // windowed and fullscreen, and everything attached to it has
                    // to be set up again afterwards.
                    fullscreen = !fullscreen;
                    if (fullscreen) {
                        window.create(sf::VideoMode::getDesktopMode(),
                                      "Enginio2D - engine simulator", sf::State::Fullscreen);
                    } else {
                        window.create(sf::VideoMode({static_cast<unsigned>(kW),
                                                     static_cast<unsigned>(kH)}),
                                      "Enginio2D - engine simulator");
                    }
                    window.setFramerateLimit(60);
                    window.setView(fittedView(window.getSize()));
                }
                if (!gui.textFocused()) {
                    switch (key->code) {
                        case K::Escape: window.close(); break;
                        case K::I: ignition = !ignition; sound.setIgnition(ignition); break;
                        // Relative shifts are resolved against the gear the
                        // simulation is in, not the one on screen: the gearbox
                        // may have just refused a change, and the snapshot
                        // showing that is a chunk behind.
                        case K::E: sound.requestShift(1); break;
                        case K::Q: sound.requestShift(-1); break;
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
        // While a control is being rebound, the pad is answering a question
        // rather than driving the car.
        const bool padDrives = !pad.learning();
        const bool wantThrottle = driving &&
            (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W) ||
             sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up));
        const bool wantBrake = driving &&
            (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down) ||
             sf::Keyboard::isKeyPressed(sf::Keyboard::Key::B));
        const bool wantClutch = driving &&
            (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C) ||
             sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl));

        // A key is a switch, so it is slewed at the speed a foot can move a
        // pedal. Linear rather than exponential, because a clutch that only
        // ever approaches the floor never frees the gearset.
        auto slew = [&](float& v, bool want, float perSecond) {
            const float limit = perSecond * dt;
            v += std::clamp((want ? 1.0f : 0.0f) - v, -limit, limit);
        };
        slew(throttleKey, wantThrottle, 6.0f);
        slew(brakeKey,    wantBrake,    7.0f);
        slew(clutchKey,   wantClutch,   4.5f);

        // Whichever is asking for more wins, so a pad and the keyboard can be
        // used interchangeably without either having to be switched off.
        throttleCmd = throttleKey;
        brakeCmd    = brakeKey;
        clutchCmd   = clutchKey;
        if (padDrives) {
            throttleCmd = std::max(throttleCmd, pad.value(input::Control::Throttle));
            brakeCmd    = std::max(brakeCmd,    pad.value(input::Control::Brake));
            clutchCmd   = std::max(clutchCmd,   pad.value(input::Control::Clutch));
            if (pad.pressed(input::Control::ShiftUp))   sound.requestShift(1);
            if (pad.pressed(input::Control::ShiftDown)) sound.requestShift(-1);
            if (pad.pressed(input::Control::Neutral))   sound.requestGear(0);
            if (pad.pressed(input::Control::Ignition)) {
                ignition = !ignition;
                sound.setIgnition(ignition);
            }
        }
        sound.setThrottle(throttleCmd);
        sound.setBrake(brakeCmd);
        sound.setClutch(clutchCmd);
        sound.setStarter((driving && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) ||
                         (padDrives && pad.held(input::Control::Starter)));

        const sim::Snapshot s = sound.snapshot();
        const sim::CylinderView& c1 = s.cyl[0];

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
            panelTitled(gui, vx, 12.0f, vw, 560.0f, "ENGINE SECTION");
            drawEngineSection(gui, params, s, vx, 12.0f, vw, 560.0f);
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
        gui.readout("WHEEL SLIP", fmt("%.0f %%", s.wheelSlip * 100.0f),
                    s.wheelSlip > 0.30f ? palette.alert : palette.text);
        gui.readout("KNOCK", fmt("%.0f %%", s.knock * 100.0f),
                    s.knock > 0.05f ? palette.alert : palette.text);

        drawBar(gui, "THROTTLE", s.throttle, rx, 404.0f, rw, palette.accent);
        drawBar(gui, "BRAKE", s.brake, rx, 438.0f, rw, palette.exhaust);
        drawBar(gui, "CLUTCH", s.clutchPedal, rx, 472.0f, rw, palette.intake);
        drawLamps(gui, s, params, rx, 494.0f, rw);

        gui.text(design.name, ix + 18.0f, 500.0f, 15, palette.accent);
        gui.text(ignition ? "IGNITION ON" : "IGNITION OFF", ix + 18.0f, 522.0f, 13,
                 ignition ? palette.good : palette.alert);
        gui.text("W throttle   DOWN brake   C clutch   S starter   Q/E shift   "
                 "0-8 gear   I ignition   TAB editor   ESC quit",
                 ix + 18.0f, 544.0f, 12, palette.dim);

        // ---- Plots ----------------------------------------------------------
        drawValveDiagram(gui, params, s.camAdvance, c1, 12.0f, 584.0f, 480.0f, 216.0f);
        drawPressureTrace(gui, engine, c1, 500.0f, 584.0f, 480.0f, 216.0f, s.peakPressure);
        drawScope(gui, sound, 988.0f, 584.0f, 440.0f, 216.0f);

        // ---- Editor ---------------------------------------------------------
        if (editor.visible) {
            const ui::EditorResult res = editor.draw(gui, design, dyno, pad, pendingChange, dt);
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
