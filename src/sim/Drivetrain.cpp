#include "sim/Drivetrain.h"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {
constexpr double kGravity = 9.81;
constexpr double kAirDensity = 1.2;
constexpr double kRpmPerRad = 30.0 / 3.14159265358979323846;
} // namespace

void Drivetrain::setGear(int g) {
    g = std::clamp(g, 0, gearCount());
    if (g == m_gear) return;
    m_gear = g;
    m_shiftTimer = m_p.shiftTime;   // clutch comes out while the gear changes
}

void Drivetrain::setBrake(double b) { m_brake = std::clamp(b, 0.0, 1.0); }

double Drivetrain::ratio() const {
    if (m_gear <= 0 || m_gear > gearCount()) return 0.0;
    return m_p.gearRatios[static_cast<std::size_t>(m_gear - 1)] * m_p.finalDrive;
}

double Drivetrain::step(double dt, double engineOmega) {
    if (m_shiftTimer > 0.0) m_shiftTimer -= dt;

    const double r = ratio();
    m_inputOmega = (m_speed / m_p.wheelRadius) * r;

    // Clutch engagement. It is open at low crank speed so the engine can never
    // be dragged to a stop, and comes home as either the engine or the road
    // speed rises - the behaviour of a driver feeding the clutch out, or of a
    // torque converter locking up.
    double target = 0.0;
    if (r > 0.0 && m_shiftTimer <= 0.0) {
        const double rpm = engineOmega * kRpmPerRad;
        target = std::clamp((rpm - m_p.slipRpm) / (m_p.lockRpm - m_p.slipRpm), 0.0, 1.0);
        if (m_inputOmega * kRpmPerRad > m_p.lockRpm) target = 1.0;
    }
    // Smooth the engagement so shifts are not steps in torque.
    m_clutch += (target - m_clutch) * std::min(1.0, 8.0 * dt);

    double torqueOnCrank = 0.0;
    m_wheelTorque = 0.0;
    m_slip = 0.0;
    if (r > 0.0 && m_clutch > 0.001) {
        m_slip = engineOmega - m_inputOmega;
        // tanh gives a friction clutch that saturates at its capacity instead
        // of an infinitely stiff link, so nothing explodes when it is dumped.
        const double t = m_clutch * m_p.clutchCapacity * std::tanh(m_slip / 12.0);
        torqueOnCrank = -t;
        m_wheelTorque = t * r;
    }

    // Vehicle: driving force at the contact patch against aerodynamic drag,
    // rolling resistance and the brakes.
    const double driveForce = m_wheelTorque / m_p.wheelRadius;
    const double drag = 0.5 * kAirDensity * m_p.dragArea * m_speed * m_speed;
    const double rolling = m_speed > 0.05 ? m_p.rollingCoef * m_p.mass * kGravity : 0.0;
    const double braking = m_speed > 0.05 ? m_brake * m_p.brakeTorque / m_p.wheelRadius : 0.0;

    const double net = driveForce - drag - rolling - braking;
    m_speed += net / m_p.mass * dt;
    if (m_speed < 0.0) m_speed = 0.0;   // no reverse, and no creeping backwards

    return torqueOnCrank;
}

} // namespace sim
