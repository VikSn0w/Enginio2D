#include "ui/Widgets.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ui {
namespace {

constexpr float kSliderRow = 36.0f;
constexpr float kChoiceRow = 32.0f;
constexpr float kToggleRow = 28.0f;

std::string fmtValue(const char* spec, double v) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), spec, v);
    return buf;
}

double quantise(double v, double step) {
    if (step <= 0.0) return v;
    return std::round(v / step) * step;
}

} // namespace

sf::Color hsl(double h, double s, double l, std::uint8_t a) {
    h = std::fmod(std::fmod(h, 360.0) + 360.0, 360.0) / 60.0;
    s = std::clamp(s, 0.0, 1.0);
    l = std::clamp(l, 0.0, 1.0);
    const double c = (1.0 - std::abs(2.0 * l - 1.0)) * s;
    const double x = c * (1.0 - std::abs(std::fmod(h, 2.0) - 1.0));
    const double m = l - 0.5 * c;
    double r = 0.0, g = 0.0, b = 0.0;
    if (h < 1.0)      { r = c; g = x; }
    else if (h < 2.0) { r = x; g = c; }
    else if (h < 3.0) { g = c; b = x; }
    else if (h < 4.0) { g = x; b = c; }
    else if (h < 5.0) { r = x; b = c; }
    else              { r = c; b = x; }
    auto ch = [&](double v) {
        return static_cast<std::uint8_t>(std::clamp((v + m) * 255.0, 0.0, 255.0));
    };
    return sf::Color(ch(r), ch(g), ch(b), a);
}

Palette makePalette(int theme, double accentHue, double blockShade,
                    double coverHue, double coverSat) {
    // Each theme is a background hue and a saturation; the greys are then all
    // the same colour at different lightnesses, which is what keeps a dark
    // interface from looking muddy.
    struct ThemeSpec { double hue, sat, base; };
    static const ThemeSpec kThemes[] = {
        {220.0, 0.10, 0.065},   // Graphite
        {235.0, 0.28, 0.050},   // Midnight
        {205.0, 0.06, 0.100},   // Slate
        {212.0, 0.42, 0.075},   // Blueprint
    };
    const ThemeSpec& t = kThemes[std::clamp(theme, 0, 3)];

    Palette p;
    p.bg       = hsl(t.hue, t.sat, t.base);
    p.panel    = hsl(t.hue, t.sat * 0.9, t.base + 0.035);
    p.panelAlt = hsl(t.hue, t.sat * 0.9, t.base + 0.060);
    p.grid     = hsl(t.hue, t.sat * 0.7, t.base + 0.095);
    p.line     = hsl(t.hue, t.sat * 0.6, t.base + 0.160);
    p.dim      = hsl(t.hue, t.sat * 0.3, 0.545);
    p.text     = hsl(t.hue, t.sat * 0.2, 0.875);
    p.accent   = hsl(accentHue, 0.90, 0.590);
    p.good     = hsl(140.0, 0.45, 0.645);
    p.alert    = hsl(3.0, 0.72, 0.560);
    p.intake   = hsl(210.0, 0.90, 0.665);
    p.exhaust  = hsl(12.0, 0.90, 0.645);
    p.metal    = hsl(t.hue, 0.06, 0.635);
    p.block    = hsl(t.hue, t.sat * 0.8, std::clamp(blockShade * 0.005, 0.03, 0.45));
    p.cover    = hsl(coverHue, coverSat * 0.01, 0.42);
    return p;
}

// ---------------------------------------------------------------------------
void Ui::begin(sf::RenderTarget& rt, const InputState& in) {
    m_rt = &rt;
    m_in = in;
    m_clipping = false;
    m_clipScroll = 0.0f;
    if (!m_in.down) m_active = nullptr;
}

void Ui::end() {
    if (m_in.released) m_active = nullptr;
}

sf::Vector2f Ui::mouse() const {
    return {m_in.mouse.x, m_in.mouse.y + m_clipScroll};
}

bool Ui::hot(const sf::FloatRect& r) const {
    if (m_clipping && !m_clipRect.contains(m_in.mouse)) return false;
    return r.contains(mouse());
}

// ---------------------------------------------------------------------------
void Ui::rect(float x, float y, float w, float h, sf::Color fill,
              sf::Color outline, float thickness) {
    sf::RectangleShape r({w, h});
    r.setPosition({std::round(x), std::round(y)});
    r.setFillColor(fill);
    if (thickness > 0.0f) {
        r.setOutlineColor(outline);
        r.setOutlineThickness(thickness);
    }
    m_rt->draw(r);
}

void Ui::line(sf::Vector2f a, sf::Vector2f b, sf::Color c) {
    sf::VertexArray v(sf::PrimitiveType::Lines, 2);
    v[0].position = a; v[1].position = b;
    v[0].color = v[1].color = c;
    m_rt->draw(v);
}

void Ui::vertices(const sf::VertexArray& v) { m_rt->draw(v); }
void Ui::circleShape(const sf::CircleShape& s) { m_rt->draw(s); }
void Ui::rectShape(const sf::RectangleShape& s) { m_rt->draw(s); }

void Ui::text(const std::string& s, float x, float y, unsigned size, sf::Color col) {
    sf::Text t(*m_font, s, size);
    t.setFillColor(col);
    t.setPosition({std::round(x), std::round(y)});
    m_rt->draw(t);
}

void Ui::centred(const std::string& s, float cx, float y, unsigned size, sf::Color col) {
    sf::Text t(*m_font, s, size);
    t.setFillColor(col);
    const auto b = t.getLocalBounds();
    t.setPosition({std::round(cx - b.size.x * 0.5f - b.position.x), std::round(y)});
    m_rt->draw(t);
}

void Ui::right(const std::string& s, float rx, float y, unsigned size, sf::Color col) {
    sf::Text t(*m_font, s, size);
    t.setFillColor(col);
    const auto b = t.getLocalBounds();
    t.setPosition({std::round(rx - b.size.x - b.position.x), std::round(y)});
    m_rt->draw(t);
}

float Ui::textWidth(const std::string& s, unsigned size) const {
    sf::Text t(*m_font, s, size);
    return t.getLocalBounds().size.x;
}

// ---------------------------------------------------------------------------
void Ui::beginScroll(const sf::FloatRect& area, float& scroll, float contentHeight) {
    const float maxScroll = std::max(0.0f, contentHeight - area.size.y);
    if (area.contains(m_in.mouse) && m_in.wheel != 0.0f && m_active == nullptr)
        scroll -= m_in.wheel * 42.0f;
    scroll = std::clamp(scroll, 0.0f, maxScroll);

    m_savedView = m_rt->getView();

    // The clip region is a viewport, which is a fraction of the *window*, but
    // `area` is in design coordinates. Those two only coincide when the window
    // happens to be exactly the size of the design space - at any other size,
    // and most obviously in fullscreen, dividing design coordinates by the pixel
    // size puts every scrolling region in the wrong place and at the wrong
    // scale. So the rectangle is mapped through the view that is already
    // active, which is what carries the letterboxing.
    const sf::FloatRect base = m_savedView.getViewport();
    const sf::Vector2f visible = m_savedView.getSize();
    const sf::Vector2f origin(m_savedView.getCenter().x - visible.x * 0.5f,
                              m_savedView.getCenter().y - visible.y * 0.5f);
    const sf::Vector2f scale(visible.x > 1e-3f ? base.size.x / visible.x : 0.0f,
                             visible.y > 1e-3f ? base.size.y / visible.y : 0.0f);

    sf::View v(sf::FloatRect({area.position.x, area.position.y + scroll}, area.size));
    v.setViewport(sf::FloatRect(
        {base.position.x + (area.position.x - origin.x) * scale.x,
         base.position.y + (area.position.y - origin.y) * scale.y},
        {area.size.x * scale.x, area.size.y * scale.y}));
    m_rt->setView(v);
    m_clipping = true;
    m_clipRect = area;
    m_clipScroll = scroll;

    // A scrollbar, but only when there is something to scroll to.
    if (maxScroll > 1.0f) {
        const float trackX = area.position.x + area.size.x - 5.0f;
        const float frac = area.size.y / contentHeight;
        const float barH = std::max(30.0f, area.size.y * frac);
        const float barY = area.position.y + scroll +
                           (area.size.y - barH) * (scroll / maxScroll);
        rect(trackX, barY, 4.0f, barH, m_pal.line);
    }
}

void Ui::endScroll() {
    m_rt->setView(m_savedView);
    m_clipping = false;
    m_clipScroll = 0.0f;
}

// ---------------------------------------------------------------------------
void Ui::heading(const std::string& s) {
    m_y += 12.0f;
    text(s, m_x, m_y, 13, m_pal.accent);
    line({m_x, m_y + 19.0f}, {m_x + m_w, m_y + 19.0f}, m_pal.line);
    m_y += 28.0f;
}

void Ui::note(const std::string& s) {
    text(s, m_x, m_y, 11, m_pal.dim);
    m_y += 16.0f;
}

void Ui::readout(const std::string& label, const std::string& value) {
    readout(label, value, m_pal.text);
}

void Ui::readout(const std::string& label, const std::string& value, sf::Color col) {
    // Label left, value right, on one line - so a column of these has to be
    // set small enough that the two never meet in the middle.
    text(label, m_x, m_y + 3.0f, 11, m_pal.dim);
    right(value, m_x + m_w, m_y, 13, col);
    m_y += 21.0f;
}

bool Ui::slider(const std::string& label, double& v, double lo, double hi,
                const char* fmt, double step, const void* id) {
    if (id == nullptr) id = &v;
    const float y = m_y;
    const float trackY = y + 21.0f;
    const sf::FloatRect row({m_x, y}, {m_w, kSliderRow - 4.0f});
    const sf::FloatRect grab({m_x - 4.0f, trackY - 9.0f}, {m_w + 8.0f, 20.0f});
    bool changed = false;

    const bool over = hot(row);
    if (hot(grab) && m_in.pressed) m_active = id;
    if (m_active == id && m_in.down) {
        const double f = std::clamp((mouse().x - m_x) / std::max(m_w, 1.0f), 0.0f, 1.0f);
        double nv = lo + (hi - lo) * f;
        // Shift drags without snapping, which is the only way to reach a value
        // between two steps.
        if (!m_in.fine) nv = quantise(nv, step);
        nv = std::clamp(nv, lo, hi);
        if (nv != v) { v = nv; changed = true; }
    }
    if (over && m_in.wheel != 0.0f && m_active == nullptr) {
        const double nudge = (step > 0.0 ? step : (hi - lo) * 0.01) *
                             (m_in.fine ? 0.2 : 1.0);
        const double nv = std::clamp(v + nudge * m_in.wheel, lo, hi);
        if (nv != v) { v = nv; changed = true; }
    }

    const float f = static_cast<float>(std::clamp((v - lo) / std::max(hi - lo, 1e-9), 0.0, 1.0));
    text(label, m_x, y, 12, over ? m_pal.text : m_pal.dim);
    right(fmtValue(fmt, v), m_x + m_w, y - 2.0f, 14,
          m_active == id ? m_pal.accent : m_pal.text);
    rect(m_x, trackY, m_w, 5.0f, m_pal.grid);
    rect(m_x, trackY, m_w * f, 5.0f, m_active == id ? m_pal.accent : m_pal.line);
    const float hx = m_x + m_w * f;
    rect(hx - 3.0f, trackY - 4.0f, 6.0f, 13.0f,
         (over || m_active == id) ? m_pal.accent : m_pal.metal);

    m_y += kSliderRow;
    return changed;
}

bool Ui::sliderInt(const std::string& label, int& v, int lo, int hi, const char* suffix) {
    double d = v;
    char fmt[48];
    std::snprintf(fmt, sizeof(fmt), "%%.0f%s", suffix ? suffix : "");
    const bool changed = slider(label, d, lo, hi, fmt, 1.0, &v);
    (void)changed;
    const int nv = std::clamp(static_cast<int>(std::lround(d)), lo, hi);
    if (nv != v) { v = nv; return true; }
    return false;
}

bool Ui::choice(const std::string& label, int& v, const char* const* names, int count) {
    const float y = m_y;
    const float boxX = m_x + m_w * 0.38f;
    const float boxW = m_w - m_w * 0.38f;
    const float boxH = 24.0f;
    bool changed = false;

    text(label, m_x, y + 5.0f, 12, m_pal.dim);
    rect(boxX, y, boxW, boxH, m_pal.panelAlt, m_pal.line, 1.0f);

    const sf::FloatRect left({boxX, y}, {26.0f, boxH});
    const sf::FloatRect rightR({boxX + boxW - 26.0f, y}, {26.0f, boxH});
    const bool overL = hot(left), overR = hot(rightR);
    if (overL && m_in.pressed) { v = (v - 1 + count) % count; changed = true; }
    if (overR && m_in.pressed) { v = (v + 1) % count; changed = true; }
    // The wheel steps through the list, which is much faster than clicking an
    // arrow twelve times to reach the far end of the fuel list.
    if (hot(sf::FloatRect({boxX, y}, {boxW, boxH})) && m_in.wheel != 0.0f) {
        v = (v + (m_in.wheel > 0.0f ? 1 : count - 1)) % count;
        changed = true;
    }

    centred("<", boxX + 13.0f, y + 3.0f, 15, overL ? m_pal.accent : m_pal.dim);
    centred(">", boxX + boxW - 13.0f, y + 3.0f, 15, overR ? m_pal.accent : m_pal.dim);
    const int i = std::clamp(v, 0, std::max(0, count - 1));
    centred(names && count > 0 ? names[i] : "-", boxX + boxW * 0.5f, y + 3.0f, 13, m_pal.text);

    m_y += kChoiceRow;
    return changed;
}

bool Ui::toggle(const std::string& label, bool& v) {
    const float y = m_y;
    const float sw = 40.0f, sh = 20.0f;
    const float sx = m_x + m_w - sw;
    const sf::FloatRect box({sx, y}, {sw, sh});
    bool changed = false;
    if (hot(box) && m_in.pressed) { v = !v; changed = true; }

    text(label, m_x, y + 3.0f, 12, m_pal.dim);
    rect(sx, y, sw, sh, v ? m_pal.accent : m_pal.grid, m_pal.line, 1.0f);
    rect(v ? sx + sw - 17.0f : sx + 1.0f, y + 1.0f, 16.0f, sh - 2.0f,
         v ? sf::Color(20, 22, 28) : m_pal.metal);

    m_y += kToggleRow;
    return changed;
}

bool Ui::button(const std::string& label, float x, float y, float w, float h,
                bool active, sf::Color tint) {
    const sf::FloatRect box({x, y}, {w, h});
    const bool over = hot(box);
    const bool clicked = over && m_in.pressed;
    const sf::Color base = tint.a == 0 ? m_pal.accent : tint;
    sf::Color fill = active ? base : (over ? m_pal.panelAlt : m_pal.panel);
    rect(x, y, w, h, fill, over || active ? base : m_pal.line, 1.0f);
    centred(label, x + w * 0.5f, y + (h - 15.0f) * 0.5f - 1.0f, 13,
            active ? sf::Color(18, 20, 25) : (over ? m_pal.text : m_pal.dim));
    return clicked;
}

bool Ui::textField(const std::string& label, std::string& v, std::size_t maxLen) {
    const float y = m_y;
    const float boxX = m_x + m_w * 0.28f;
    const float boxW = m_w - m_w * 0.28f;
    const float boxH = 24.0f;
    const sf::FloatRect box({boxX, y}, {boxW, boxH});
    bool changed = false;

    if (m_in.pressed) {
        if (hot(box)) m_focus = &v;
        else if (m_focus == &v) m_focus = nullptr;
    }
    if (m_focus == &v) {
        for (std::uint32_t c : m_in.typed)
            if (c >= 32 && c < 127 && v.size() < maxLen) {
                v.push_back(static_cast<char>(c));
                changed = true;
            }
        if (m_in.backspace && !v.empty()) { v.pop_back(); changed = true; }
        if (m_in.enter) m_focus = nullptr;
    }

    text(label, m_x, y + 5.0f, 12, m_pal.dim);
    rect(boxX, y, boxW, boxH, m_pal.panelAlt,
         m_focus == &v ? m_pal.accent : m_pal.line, 1.0f);
    text(v, boxX + 8.0f, y + 3.0f, 13, m_pal.text);
    if (m_focus == &v)
        rect(boxX + 9.0f + textWidth(v, 13), y + 4.0f, 1.5f, boxH - 8.0f, m_pal.accent);

    m_y += kChoiceRow;
    return changed;
}

} // namespace ui
