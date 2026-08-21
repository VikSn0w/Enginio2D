#include "input/Gamepad.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace input {
namespace {

constexpr int kAxisCount = static_cast<int>(sf::Joystick::AxisCount);

const char* const kControlNames[kControlCount] = {
    "THROTTLE", "BRAKE", "CLUTCH",
    "SHIFT UP", "SHIFT DOWN", "STARTER", "IGNITION", "NEUTRAL"
};

const char* const kControlKeys[kControlCount] = {
    "throttle", "brake", "clutch",
    "shiftUp", "shiftDown", "starter", "ignition", "neutral"
};

const char* axisName(int a) {
    static const char* const names[] = {"X", "Y", "Z", "R", "U", "V", "PovX", "PovY"};
    return (a >= 0 && a < kAxisCount) ? names[a] : "?";
}

sf::Joystick::Axis axisOf(int a) {
    return static_cast<sf::Joystick::Axis>(std::clamp(a, 0, kAxisCount - 1));
}

bool hasAxis(int device, int a) {
    return device >= 0 && sf::Joystick::isConnected(static_cast<unsigned>(device)) &&
           sf::Joystick::hasAxis(static_cast<unsigned>(device), axisOf(a));
}

float readAxis(int device, int a) {
    if (!hasAxis(device, a)) return 0.0f;
    return sf::Joystick::getAxisPosition(static_cast<unsigned>(device), axisOf(a));
}

Binding axisBinding(int device, int axis, float released, float pressed) {
    Binding b;
    b.kind = Binding::Kind::Axis;
    b.device = device;
    b.index = axis;
    b.released = released;
    b.pressed = pressed;
    return b;
}

Binding buttonBinding(int device, int button) {
    Binding b;
    b.kind = Binding::Kind::Button;
    b.device = device;
    b.index = button;
    return b;
}

} // namespace

const char* controlName(Control c) {
    const int i = static_cast<int>(c);
    return (i >= 0 && i < kControlCount) ? kControlNames[i] : "?";
}

bool controlIsAxis(Control c) {
    return c == Control::Throttle || c == Control::Brake || c == Control::Clutch;
}

// ---------------------------------------------------------------------------
float Binding::read() const {
    if (kind == Binding::Kind::None) return 0.0f;
    if (device < 0 || !sf::Joystick::isConnected(static_cast<unsigned>(device))) return 0.0f;
    if (kind == Binding::Kind::Button) {
        if (index < 0 || index >= static_cast<int>(sf::Joystick::ButtonCount)) return 0.0f;
        return sf::Joystick::isButtonPressed(static_cast<unsigned>(device),
                                             static_cast<unsigned>(index)) ? 1.0f : 0.0f;
    }
    const float span = pressed - released;
    if (std::abs(span) < 1.0f) return 0.0f;
    return std::clamp((readAxis(device, index) - released) / span, 0.0f, 1.0f);
}

std::string Binding::label() const {
    char buf[64];
    switch (kind) {
        case Binding::Kind::Axis:
            // The sign is which way the control has to move, which is the only
            // part of a raw axis reading that means anything to a person.
            std::snprintf(buf, sizeof(buf), "PAD%d AXIS %s %s", device + 1,
                          axisName(index), pressed < released ? "-" : "+");
            return buf;
        case Binding::Kind::Button:
            std::snprintf(buf, sizeof(buf), "PAD%d BUTTON %d", device + 1, index + 1);
            return buf;
        default:
            return "not bound";
    }
}

// ---------------------------------------------------------------------------
// Defaults, worked out from where the axes are sitting rather than from a table
// of product ids that would be out of date by the time it shipped.
// ---------------------------------------------------------------------------
Bindings defaultBindings(int device) {
    Bindings b;
    if (device < 0 || !sf::Joystick::isConnected(static_cast<unsigned>(device))) return b;

    constexpr int kX = 0, kY = 1, kZ = 2, kU = 4, kV = 5;
    const bool hasZ = hasAxis(device, kZ);
    const bool hasU = hasAxis(device, kU), hasV = hasAxis(device, kV);
    // An axis sitting hard against one end with nothing being touched is a
    // trigger; one sitting in the middle is a stick, or a pair of triggers
    // sharing the axis between them.
    auto restsAtEnd = [&](int a) { return std::abs(readAxis(device, a)) > 60.0f; };

    if (hasU && hasV && restsAtEnd(kU) && restsAtEnd(kV)) {
        // Two triggers with an axis each, resting at the bottom of their
        // travel. This is what a DualShock reports through DirectInput.
        b[Control::Brake]    = axisBinding(device, kU, readAxis(device, kU), -readAxis(device, kU));
        b[Control::Throttle] = axisBinding(device, kV, readAxis(device, kV), -readAxis(device, kV));
    } else if (hasZ) {
        // Both triggers on one axis pulling opposite ways, which is how an Xbox
        // pad appears through DirectInput. Right is throttle, left is brake.
        b[Control::Throttle] = axisBinding(device, kZ, 0.0f, -100.0f);
        b[Control::Brake]    = axisBinding(device, kZ, 0.0f, 100.0f);
    } else {
        // No triggers at all: one stick has to do both ends of it.
        b[Control::Throttle] = axisBinding(device, kY, 0.0f, -100.0f);
        b[Control::Brake]    = axisBinding(device, kY, 0.0f, 100.0f);
    }

    // The clutch has nowhere obvious to go on a pad, so it gets the half of a
    // stick you pull towards you, which at least moves like a pedal.
    const bool triggersElsewhere = hasZ || (hasU && hasV);
    b[Control::Clutch] = axisBinding(device, triggersElsewhere ? kY : kX, 0.0f, 100.0f);

    // Button numbering is not standardised either, but the shoulder buttons sit
    // at 4 and 5 on almost everything that reports ten or more.
    const int buttons = static_cast<int>(sf::Joystick::getButtonCount(static_cast<unsigned>(device)));
    if (buttons >= 6) {
        b[Control::ShiftUp]   = buttonBinding(device, 5);
        b[Control::ShiftDown] = buttonBinding(device, 4);
    }
    if (buttons >= 1) b[Control::Starter]  = buttonBinding(device, 0);
    if (buttons >= 3) b[Control::Neutral]  = buttonBinding(device, 2);
    if (buttons >= 8) b[Control::Ignition] = buttonBinding(device, 7);
    return b;
}

// ---------------------------------------------------------------------------
std::vector<Gamepad::DeviceInfo> Gamepad::devices() {
    sf::Joystick::update();
    std::vector<DeviceInfo> out;
    for (int i = 0; i < static_cast<int>(sf::Joystick::Count); ++i) {
        const auto slot = static_cast<unsigned>(i);
        if (!sf::Joystick::isConnected(slot)) continue;
        DeviceInfo d;
        d.index = i;
        d.name = sf::Joystick::getIdentification(slot).name.toAnsiString();
        d.buttons = static_cast<int>(sf::Joystick::getButtonCount(slot));
        for (int a = 0; a < kAxisCount; ++a)
            if (sf::Joystick::hasAxis(slot, axisOf(a))) ++d.axes;
        out.push_back(d);
    }
    return out;
}

void Gamepad::selectDevice(int index) {
    sf::Joystick::update();
    sampleRest();
    adopt(index, true);
    m_bind.preferredDevice = m_name;
}

void Gamepad::start() {
    sf::Joystick::update();
    sampleRest();

    // Only build defaults if nothing has been loaded from disk.
    bool any = false;
    for (const Binding& x : m_bind.map) any = any || x.bound();

    // The remembered device wins wherever it turns up, and only if it is not
    // here does slot order get a say.
    if (!m_bind.preferredDevice.empty()) {
        for (const DeviceInfo& d : devices())
            if (d.name == m_bind.preferredDevice) { adopt(d.index, !any); return; }
    }
    for (int i = 0; i < static_cast<int>(sf::Joystick::Count); ++i) {
        if (sf::Joystick::isConnected(static_cast<unsigned>(i))) {
            adopt(i, !any);
            return;
        }
    }
}

void Gamepad::sampleRest() {
    for (int d = 0; d < static_cast<int>(sf::Joystick::Count); ++d)
        for (int a = 0; a < kAxisCount; ++a)
            m_rest[static_cast<std::size_t>(d)][static_cast<std::size_t>(a)] =
                sf::Joystick::isConnected(static_cast<unsigned>(d)) ? readAxis(d, a) : 0.0f;
}

void Gamepad::adopt(int device, bool rebind) {
    m_device = device;
    m_connected = device >= 0 && sf::Joystick::isConnected(static_cast<unsigned>(device));
    m_name = m_connected
        ? sf::Joystick::getIdentification(static_cast<unsigned>(device)).name.toAnsiString()
        : std::string();
    if (m_connected && rebind) {
        // The feel settings are the person's, not the device's, so they survive
        // a pad being swapped underneath them.
        const float dz = m_bind.deadzone;
        const float gamma = m_bind.clutchGamma;
        m_bind = defaultBindings(device);
        m_bind.deadzone = dz;
        m_bind.clutchGamma = gamma;
    }
}

void Gamepad::handleEvent(const sf::Event& e) {
    if (const auto* c = e.getIf<sf::Event::JoystickConnected>()) {
        sf::Joystick::update();
        sampleRest();
        // Adopt it if nothing was plugged in; otherwise leave the active device
        // alone, since a second device is usually a shifter or a pedal box that
        // the bindings can already be pointed at by hand. The one exception is
        // the device that was asked for by name: if that turns up, it is what
        // the person meant to drive with.
        const int slot = static_cast<int>(c->joystickId);
        const std::string name =
            sf::Joystick::getIdentification(c->joystickId).name.toAnsiString();
        if (!m_connected) adopt(slot, true);
        else if (!m_bind.preferredDevice.empty() && name == m_bind.preferredDevice &&
                 name != m_name)
            selectDevice(slot);
    }
    if (const auto* d = e.getIf<sf::Event::JoystickDisconnected>()) {
        if (static_cast<int>(d->joystickId) == m_device) {
            m_connected = false;
            // Fall back to any other device still plugged in.
            for (int i = 0; i < static_cast<int>(sf::Joystick::Count); ++i)
                if (sf::Joystick::isConnected(static_cast<unsigned>(i))) { adopt(i, false); break; }
        }
    }
}

void Gamepad::beginLearn(Control c) {
    sf::Joystick::update();
    sampleRest();
    m_learn = static_cast<int>(c);
    // Whatever is being held when the rebind was asked for must not be what
    // gets bound, so nothing counts for a moment.
    m_learnGuard = 0.30f;
}

void Gamepad::cancelLearn() { m_learn = -1; }

void Gamepad::update(float dt) {
    sf::Joystick::update();

    if (m_device >= 0 && !sf::Joystick::isConnected(static_cast<unsigned>(m_device)))
        m_connected = false;
    if (!m_connected) {
        for (int i = 0; i < static_cast<int>(sf::Joystick::Count); ++i) {
            if (sf::Joystick::isConnected(static_cast<unsigned>(i))) {
                bool any = false;
                for (const Binding& x : m_bind.map) any = any || x.bound();
                sampleRest();
                adopt(i, !any);
                break;
            }
        }
    }

    // ---- Learning -----------------------------------------------------------
    if (m_learn >= 0) {
        if (m_learnGuard > 0.0f) {
            m_learnGuard -= dt;
        } else {
            for (int d = 0; d < static_cast<int>(sf::Joystick::Count) && m_learn >= 0; ++d) {
                if (!sf::Joystick::isConnected(static_cast<unsigned>(d))) continue;
                const int buttons =
                    static_cast<int>(sf::Joystick::getButtonCount(static_cast<unsigned>(d)));
                for (int i = 0; i < buttons; ++i) {
                    if (sf::Joystick::isButtonPressed(static_cast<unsigned>(d),
                                                      static_cast<unsigned>(i))) {
                        m_bind.map[static_cast<std::size_t>(m_learn)] = buttonBinding(d, i);
                        m_learn = -1;
                        break;
                    }
                }
                if (m_learn < 0) break;
                for (int a = 0; a < kAxisCount; ++a) {
                    if (!hasAxis(d, a)) continue;
                    const float rest = m_rest[static_cast<std::size_t>(d)][static_cast<std::size_t>(a)];
                    const float now  = readAxis(d, a);
                    // A wide margin, so a stick that is merely drifting or a
                    // trigger being brushed does not steal the binding.
                    if (std::abs(now - rest) > 45.0f) {
                        // Full travel in the direction it was pushed is what
                        // fully pressed means, whether that is one end of a
                        // trigger range or one side of a stick.
                        m_bind.map[static_cast<std::size_t>(m_learn)] =
                            axisBinding(d, a, rest, now > rest ? 100.0f : -100.0f);
                        m_learn = -1;
                        break;
                    }
                }
            }
        }
    }

    // ---- Resolve every control ---------------------------------------------
    const float dz = std::clamp(m_bind.deadzone, 0.0f, 0.7f);
    for (int i = 0; i < kControlCount; ++i) {
        float v = m_bind.map[static_cast<std::size_t>(i)].read();
        // The dead zone comes off the released end and the rest is stretched
        // back out, so a control still reaches 1.0 when it is fully pressed.
        v = std::clamp((v - dz) / (1.0f - dz), 0.0f, 1.0f);
        if (static_cast<Control>(i) == Control::Clutch && m_bind.clutchGamma > 0.05f)
            v = std::pow(v, m_bind.clutchGamma);
        m_value[static_cast<std::size_t>(i)] = v;

        const bool down = v > 0.5f;
        m_edge[static_cast<std::size_t>(i)] = down && !m_last[static_cast<std::size_t>(i)];
        m_last[static_cast<std::size_t>(i)] = down;
    }
}

float Gamepad::rawAxis(int axis) const { return readAxis(m_device, axis); }

bool Gamepad::rawButton(int button) const {
    if (!m_connected || button < 0 || button >= static_cast<int>(sf::Joystick::ButtonCount))
        return false;
    return sf::Joystick::isButtonPressed(static_cast<unsigned>(m_device),
                                         static_cast<unsigned>(button));
}

int Gamepad::buttonCount() const {
    if (!m_connected) return 0;
    return static_cast<int>(sf::Joystick::getButtonCount(static_cast<unsigned>(m_device)));
}

// ---------------------------------------------------------------------------
// Persistence. The same flat-JSON-by-hand approach the designs use, for the
// same reason: the schema is a dozen numbers and it should stay readable.
// ---------------------------------------------------------------------------
std::string bindingsPath() {
    namespace fs = std::filesystem;
    std::error_code ec;
    // The executable runs from build/, so the presets it reads are one level
    // up; keeping the bindings beside them puts the file where a person would
    // look for it rather than inside a build directory.
    if (fs::is_directory("presets", ec)) return "controls.json";
    if (fs::is_directory("../presets", ec)) return "../controls.json";
    return "controls.json";
}

bool saveBindings(const Bindings& b, const std::string& path) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    out << "{\n";
    out << "  \"preferredDevice\": \"" << b.preferredDevice << "\",\n";
    out << "  \"deadzone\": " << b.deadzone << ",\n";
    out << "  \"clutchGamma\": " << b.clutchGamma << ",\n";
    for (int i = 0; i < kControlCount; ++i) {
        const Binding& x = b.map[static_cast<std::size_t>(i)];
        const char* kind = x.kind == Binding::Kind::Axis ? "axis"
                         : x.kind == Binding::Kind::Button ? "button" : "none";
        out << "  \"" << kControlKeys[i] << "\": {\"kind\": \"" << kind
            << "\", \"device\": " << x.device
            << ", \"index\": " << x.index
            << ", \"released\": " << x.released
            << ", \"pressed\": " << x.pressed << "}"
            << (i + 1 < kControlCount ? "," : "") << "\n";
    }
    out << "}\n";
    return static_cast<bool>(out);
}

bool loadBindings(Bindings& b, const std::string& path) {
    std::ifstream in(path);
    if (!in) return false;
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());

    // Small enough a grammar to scan rather than parse, as long as it only ever
    // reads back what this file writes.
    auto valueAfter = [&](std::size_t from, const char* key, double fallback) {
        if (from == std::string::npos) return fallback;
        const std::size_t at = text.find(std::string("\"") + key + "\"", from);
        if (at == std::string::npos) return fallback;
        const std::size_t colon = text.find(':', at);
        if (colon == std::string::npos) return fallback;
        return std::atof(text.c_str() + colon + 1);
    };

    Bindings out;
    {
        const std::size_t at = text.find("\"preferredDevice\"");
        if (at != std::string::npos) {
            const std::size_t colon = text.find(':', at);
            const std::size_t q1 = colon == std::string::npos ? colon : text.find('"', colon);
            const std::size_t q2 = q1 == std::string::npos ? q1 : text.find('"', q1 + 1);
            if (q1 != std::string::npos && q2 != std::string::npos)
                out.preferredDevice = text.substr(q1 + 1, q2 - q1 - 1);
        }
    }
    out.deadzone    = static_cast<float>(valueAfter(0, "deadzone", out.deadzone));
    out.clutchGamma = static_cast<float>(valueAfter(0, "clutchGamma", out.clutchGamma));

    bool anyFound = false;
    for (int i = 0; i < kControlCount; ++i) {
        const std::size_t at = text.find(std::string("\"") + kControlKeys[i] + "\"");
        if (at == std::string::npos) continue;
        const std::size_t brace = text.find('{', at);
        const std::size_t end   = text.find('}', at);
        if (brace == std::string::npos || end == std::string::npos || brace > end) continue;
        const std::size_t kindAt = text.find("\"kind\"", brace);
        if (kindAt == std::string::npos || kindAt > end) continue;
        const std::size_t colon = text.find(':', kindAt);
        const std::size_t q1 = colon == std::string::npos ? colon : text.find('"', colon);
        const std::size_t q2 = q1 == std::string::npos ? q1 : text.find('"', q1 + 1);
        if (q1 == std::string::npos || q2 == std::string::npos || q2 > end) continue;
        const std::string kind = text.substr(q1 + 1, q2 - q1 - 1);

        Binding x;
        x.kind = kind == "axis" ? Binding::Kind::Axis
               : kind == "button" ? Binding::Kind::Button : Binding::Kind::None;
        x.device   = static_cast<int>(valueAfter(brace, "device", 0));
        x.index    = static_cast<int>(valueAfter(brace, "index", 0));
        x.released = static_cast<float>(valueAfter(brace, "released", 0.0));
        x.pressed  = static_cast<float>(valueAfter(brace, "pressed", 100.0));
        out.map[static_cast<std::size_t>(i)] = x;
        anyFound = true;
    }
    if (!anyFound) return false;
    out.deadzone    = std::clamp(out.deadzone, 0.0f, 0.7f);
    out.clutchGamma = std::clamp(out.clutchGamma, 0.2f, 4.0f);
    b = out;
    return true;
}

} // namespace input
