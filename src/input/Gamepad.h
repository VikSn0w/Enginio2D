#pragma once
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Joystick.hpp>

#include <array>
#include <string>

namespace input {

// ---------------------------------------------------------------------------
// Gamepads, wheels and pedal boxes.
//
// SFML presents every device as a numbered set of axes and buttons and tells
// you nothing about what they mean, and the meaning is genuinely not knowable:
// an Xbox pad seen through DirectInput puts both triggers on one axis pulling
// in opposite directions, a DualShock puts them on two axes that rest at the
// bottom of their travel, and a wheel puts the pedals somewhere else again.
// Guessing one layout and calling it controller support works for one pad.
//
// So a binding here is a device, a control on it, and the two raw readings that
// mean "released" and "fully pressed". Those two numbers express all three
// layouts above without special cases, the defaults are picked by looking at
// where the axes rest when nothing is being touched, and anything the guess
// gets wrong can be pointed at the real control instead.
// ---------------------------------------------------------------------------

enum class Control {
    Throttle, Brake, Clutch,
    ShiftUp, ShiftDown, Starter, Ignition, Neutral,
    Count
};
constexpr int kControlCount = static_cast<int>(Control::Count);

const char* controlName(Control c);
// True for the controls that want a pedal rather than a switch, which is what
// decides whether a digital source has to be ramped to be usable.
bool controlIsAxis(Control c);

struct Binding {
    enum class Kind { None, Axis, Button };
    Kind  kind   = Kind::None;
    int   device = 0;       // SFML joystick index
    int   index  = 0;       // axis id (sf::Joystick::Axis) or button number
    // Raw axis readings, -100..100, that mean released and fully pressed. They
    // may be in either order, which is how one axis carries two triggers.
    float released = 0.0f;
    float pressed  = 100.0f;

    bool  bound() const { return kind != Kind::None; }
    // 0..1, before the dead zone.
    float read() const;
    std::string label() const;
};

struct Bindings {
    std::array<Binding, kControlCount> map{};
    // Applied at the released end of every axis, which is where the noise is.
    float deadzone = 0.10f;
    // Sticks and triggers are short and abrupt next to a pedal. This stretches
    // the bottom of the travel so the bite point is not two millimetres wide.
    float clutchGamma = 1.6f;

    Binding&       operator[](Control c)       { return map[static_cast<int>(c)]; }
    const Binding& operator[](Control c) const { return map[static_cast<int>(c)]; }
};

// What the axes read with nothing being touched. Sampled on connection, and
// what the default bindings are worked out from.
Bindings defaultBindings(int device);

class Gamepad {
public:
    // Picks up whatever is already plugged in and builds default bindings for
    // it unless some have been loaded.
    void start();
    // Poll the devices, resolve every control, and work out the edges.
    void update(float dt);
    // Connection and disconnection only; the values are polled, not evented.
    void handleEvent(const sf::Event& e);

    bool connected() const { return m_connected; }
    int  device() const    { return m_device; }
    const std::string& deviceName() const { return m_name; }

    float value(Control c) const { return m_value[static_cast<int>(c)]; }
    bool  held(Control c) const  { return m_value[static_cast<int>(c)] > 0.5f; }
    // True only on the frame the control went down.
    bool  pressed(Control c) const { return m_edge[static_cast<int>(c)]; }

    // ---- Rebinding ---------------------------------------------------------
    // Watch every connected device and bind the first thing that moves. An axis
    // is learnt from where it rested to where it was pushed, so a trigger and a
    // stick both come out right without being told which they are.
    void beginLearn(Control c);
    void cancelLearn();
    bool learning() const           { return m_learn >= 0; }
    Control learnTarget() const     { return static_cast<Control>(m_learn); }
    void clearBinding(Control c)    { m_bind[c] = Binding{}; }

    Bindings&       bindings()       { return m_bind; }
    const Bindings& bindings() const { return m_bind; }
    void setBindings(const Bindings& b) { m_bind = b; }

    // Raw reading of one axis on the active device, for the diagnostics view.
    float rawAxis(int axis) const;
    bool  rawButton(int button) const;
    int   buttonCount() const;

private:
    void adopt(int device, bool rebind);
    void sampleRest();

    Bindings m_bind{};
    bool  m_connected = false;
    int   m_device = -1;
    std::string m_name;
    std::array<float, kControlCount> m_value{};
    std::array<bool,  kControlCount> m_edge{};
    std::array<bool,  kControlCount> m_last{};

    int   m_learn = -1;           // Control being learnt, -1 for none
    float m_learnGuard = 0.0f;    // ignores whatever is still held when it starts
    std::array<std::array<float, sf::Joystick::AxisCount>, sf::Joystick::Count> m_rest{};
};

// Bindings are per person, not per engine, so they live beside the presets
// rather than inside a design.
bool saveBindings(const Bindings& b, const std::string& path);
bool loadBindings(Bindings& b, const std::string& path);
// "controls.json" next to the presets directory, wherever that turns out to be.
std::string bindingsPath();

} // namespace input
