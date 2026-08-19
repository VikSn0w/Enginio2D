#pragma once
#include <SFML/Graphics.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace ui {

// ---------------------------------------------------------------------------
// Colours. Everything the program draws pulls from one palette, which is built
// from the design's theme and accent hue - so recolouring the engine recolours
// the instruments with it.
// ---------------------------------------------------------------------------
struct Palette {
    sf::Color bg{14, 16, 20};
    sf::Color panel{24, 27, 34};
    sf::Color panelAlt{31, 35, 44};
    sf::Color line{58, 64, 78};
    sf::Color grid{40, 45, 56};
    sf::Color text{214, 220, 232};
    sf::Color dim{126, 134, 150};
    sf::Color accent{255, 138, 46};
    sf::Color good{120, 210, 140};
    sf::Color alert{226, 68, 60};
    sf::Color intake{86, 168, 255};
    sf::Color exhaust{255, 104, 76};
    sf::Color metal{150, 158, 175};
    sf::Color block{46, 51, 62};
    sf::Color cover{70, 80, 96};
};

sf::Color hsl(double hueDeg, double sat01, double light01, std::uint8_t alpha = 255);
Palette   makePalette(int theme, double accentHue, double blockShade,
                      double coverHue, double coverSat);

// Everything the widgets need to know about the mouse and keyboard this frame.
struct InputState {
    sf::Vector2f mouse{};
    bool  down     = false;   // button is held
    bool  pressed  = false;   // went down this frame
    bool  released = false;
    float wheel    = 0.0f;    // notches, positive is away from the user
    bool  fine     = false;   // shift: smaller steps
    std::vector<std::uint32_t> typed;
    bool  backspace = false;
    bool  enter     = false;
};

// ---------------------------------------------------------------------------
// A small immediate-mode toolkit: no widget objects, no retained tree, just a
// cursor that walks down a column and leaves controls behind it. Identity comes
// from the address of the value being edited, which is unique and stable
// without anyone having to invent ids.
// ---------------------------------------------------------------------------
class Ui {
public:
    Ui(const sf::Font& font, const Palette& pal) : m_font(&font), m_pal(pal) {}

    void begin(sf::RenderTarget& rt, const InputState& in);
    void end();

    void setPalette(const Palette& p) { m_pal = p; }
    const Palette& pal() const { return m_pal; }
    const InputState& input() const { return m_in; }

    // ---- Raw drawing, shared with the rest of the program -------------------
    void rect(float x, float y, float w, float h, sf::Color fill,
              sf::Color outline = sf::Color::Transparent, float thickness = 0.0f);
    void line(sf::Vector2f a, sf::Vector2f b, sf::Color c);
    // For the few shapes that are not axis-aligned rectangles.
    // Batched geometry. A plot is one draw call, not one per segment.
    void vertices(const sf::VertexArray& v);
    void circleShape(const sf::CircleShape& s);
    void rectShape(const sf::RectangleShape& s);
    void text(const std::string& s, float x, float y, unsigned size, sf::Color col);
    void centred(const std::string& s, float cx, float y, unsigned size, sf::Color col);
    void right(const std::string& s, float rx, float y, unsigned size, sf::Color col);
    float textWidth(const std::string& s, unsigned size) const;

    // ---- Clipping and scrolling --------------------------------------------
    // Content drawn between these two calls is clipped to the rectangle and
    // shifted up by the scroll offset, which the wheel edits in place.
    void beginScroll(const sf::FloatRect& area, float& scroll, float contentHeight);
    void endScroll();

    // ---- Layout -------------------------------------------------------------
    void  column(float x, float y, float w) { m_x = x; m_y = y; m_w = w; }
    void  skip(float dy) { m_y += dy; }
    float cursorY() const { return m_y; }
    float columnX() const { return m_x; }
    float columnW() const { return m_w; }

    // ---- Widgets. Each returns true on the frames it changes the value. -----
    void heading(const std::string& s);
    void note(const std::string& s);
    void readout(const std::string& label, const std::string& value);
    void readout(const std::string& label, const std::string& value, sf::Color col);
    // `id` gives the control its identity across frames; it defaults to the
    // address of the value, which is what every direct caller wants. sliderInt
    // has to pass its own, because the double it edits lives on the stack.
    bool slider(const std::string& label, double& v, double lo, double hi,
                const char* fmt, double step, const void* id = nullptr);
    bool sliderInt(const std::string& label, int& v, int lo, int hi, const char* suffix);
    bool choice(const std::string& label, int& v, const char* const* names, int count);
    bool toggle(const std::string& label, bool& v);
    bool button(const std::string& label, float x, float y, float w, float h,
                bool active = false, sf::Color tint = sf::Color::Transparent);
    bool textField(const std::string& label, std::string& v, std::size_t maxLen);

    // A control is being dragged, so the frame behind should not also react.
    bool capturing() const { return m_active != nullptr; }
    // True while a text field has the caret, so the driving keys can stand down.
    bool textFocused() const { return m_focus != nullptr; }

private:
    bool hot(const sf::FloatRect& r) const;
    sf::Vector2f mouse() const;

    const sf::Font* m_font;
    Palette m_pal;
    sf::RenderTarget* m_rt = nullptr;
    InputState m_in;

    float m_x = 0.0f, m_y = 0.0f, m_w = 300.0f;

    const void* m_active = nullptr;   // widget being dragged
    const void* m_focus  = nullptr;   // text field with the caret

    bool  m_clipping = false;
    sf::FloatRect m_clipRect{};
    float m_clipScroll = 0.0f;
    sf::View m_savedView;
};

} // namespace ui
