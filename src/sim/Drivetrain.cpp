#include "sim/Drivetrain.h"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {
constexpr double kGravity = 9.81;
constexpr double kAirDensity = 1.2;
// Width of the regularised Coulomb friction at the disc, in rad/s. About ten
// rpm: wide enough to integrate, narrow enough that a clutch which is home
// behaves as a shaft and not as a rubber band.
constexpr double kSlipWidth = 1.2;
// Floor under the speed the slip ratio is referred to. A slip ratio is
// (wheel speed - road speed) / road speed, which is singular at rest, so every
// vehicle model floors the denominator somewhere. This one keeps the tyre
// stiffness bounded without changing anything above walking pace.
constexpr double kSlipFloor = 1.5;
} // namespace

void Drivetrain::setParams(const DrivetrainParams& p) {
    // The wheels carry their own speed now, so a change of wheel size has to be
    // carried through to them: leaving the old speed on a new radius reads as
    // the tyres having suddenly started spinning, and the car lurches on an
    // edit that changed nothing about how fast it was going.
    const double oldR = m_p.wheelRadius;
    m_p = p;
    if (m_gear > gearCount()) m_gear = gearCount();
    const double newR = std::max(0.05, m_p.wheelRadius);
    if (oldR > 1.0e-9 && std::abs(newR - oldR) > 1.0e-9) m_wheelOmega *= oldR / newR;
}

bool Drivetrain::setGear(int g) {
    g = std::clamp(g, 0, gearCount());
    if (g == m_gear) return true;
    // Neutral is always available - that is what the lever falls into. Putting
    // it *into* a gear means getting the gearset to stop, and that is the
    // clutch's job. Without it the lever meets a shaft still turning at engine
    // speed, and the noise it makes is the point.
    if (g != 0 && m_pedal < m_p.shiftClutch) {
        m_grind = 1.0;
        return false;
    }
    m_gear = g;
    m_shiftTimer = m_p.shiftTime;
    return true;
}

void Drivetrain::setBrake(double b) { m_brake = std::clamp(b, 0.0, 1.0); }

void Drivetrain::setClutchPedal(double p) { m_pedal = std::clamp(p, 0.0, 1.0); }

double Drivetrain::ratio() const {
    if (m_gear <= 0 || m_gear > gearCount()) return 0.0;
    return m_p.gearRatios[static_cast<std::size_t>(m_gear - 1)] * m_p.finalDrive;
}

double Drivetrain::clutchCapacityNow() const {
    // Pedal travel to clamp force. Above the bite point the disc is free of the
    // flywheel; below the free-play point the diaphragm spring is fully on it;
    // in between the springs are taking up load, and that band is the whole of
    // feathering a clutch out.
    const double lo = std::clamp(m_p.clutchFreePlay, 0.0, 0.9);
    const double hi = std::max(m_p.clutchBite, lo + 0.05);
    const double u = std::clamp((hi - m_pedal) / (hi - lo), 0.0, 1.0);
    // Smooth at both ends - a diaphragm spring does not switch on.
    return u * u * (3.0 - 2.0 * u);
}

double Drivetrain::step(double dt, double engineOmega, double engineInertia) {
    if (m_shiftTimer > 0.0) m_shiftTimer -= dt;
    m_grind = std::max(0.0, m_grind - dt * 1.6);

    const double r  = ratio();
    const double rw = std::max(0.05, m_p.wheelRadius);
    const double eff = std::clamp(m_p.transmissionEff, 0.5, 1.0);
    m_inputOmega = m_wheelOmega * r;

    // Clamp force follows the pedal and nothing else. There is no controller
    // here any more: if the driver lets it home at 800 rpm against a stationary
    // car, the engine stalls, which is what should happen.
    m_clutch = clutchCapacityNow();
    // A gear that has not finished going in carries nothing, however far up the
    // clutch pedal is.
    const bool engaged = r > 0.0 && m_shiftTimer <= 0.0;

    // ---- Rotating inertia at the driven wheels ------------------------------
    // The wheels themselves plus everything on the gearbox input side, which
    // the gearing multiplies by the square of the ratio.
    const double Iw = std::max(0.05, m_p.wheelInertia + m_p.gearboxInertia * r * r);

    double torqueOnCrank = 0.0;
    double clutchTorque  = 0.0;
    m_wheelTorque = 0.0;
    m_slip = 0.0;
    if (engaged && m_clutch > 1.0e-4) {
        m_slip = engineOmega - m_inputOmega;
        clutchTorque = m_clutch * m_p.clutchCapacity * std::tanh(m_slip / kSlipWidth);

        // A disc cannot do more in one step than bring the two sides to the
        // same speed; asking it to overshoots and rings. Limiting the impulse
        // to what the reduced inertia of the two sides can absorb is what keeps
        // a 1500 N m clutch on a light crank stable at any step size, and it
        // only ever binds within a few rpm of lock-up.
        const double IwTot = Iw + m_p.mass * rw * rw;   // the tyre is stiff near lock
        const double invSum = 1.0 / std::max(engineInertia, 1.0e-3) + r * r / IwTot;
        const double reduced = 1.0 / std::max(invSum, 1.0e-9);
        const double cap = std::abs(m_slip) * reduced / std::max(dt, 1.0e-9);
        clutchTorque = std::clamp(clutchTorque, -cap, cap);

        torqueOnCrank = -clutchTorque;
        // Gear losses always oppose the direction the power is flowing: they
        // take from the wheels when the engine drives, and add to the engine
        // braking when the wheels drive.
        m_wheelTorque = clutchTorque * r * (clutchTorque >= 0.0 ? eff : 1.0 / eff);
    }

    // ---- Tyre ---------------------------------------------------------------
    // Load on the driven axle: its static share, plus whatever longitudinal
    // acceleration transfers on to it. That transfer is why a rear-drive car
    // hooks up harder the more it accelerates and a front-drive one runs out of
    // grip doing the same thing.
    const double transfer = m_p.cgHeightRatio * m_p.mass * m_accel *
                            (m_p.driveRear ? 1.0 : -1.0);
    const double load = std::max(0.0, m_p.driveShare * m_p.mass * kGravity + transfer);

    // Slip ratio into a simplified Pacejka curve: force rises steeply, peaks
    // around a fifth of slip, then falls away to the sliding coefficient. That
    // fall is why a spinning tyre puts down less than one on the edge of grip,
    // and why wheelspin costs time rather than just making noise.
    //
    // The ratio is referred to whichever of the two surfaces is moving faster,
    // not to the road speed alone. Referring it to the road speed makes a car
    // pulling away from rest read as fully sliding the instant the wheel turns
    // at all - the denominator is then the floor rather than anything physical -
    // and every launch comes out traction-limited into the sliding part of the
    // curve. Taking the larger of the two also bounds the ratio at one, which
    // is what a locked or a freely spinning wheel actually is.
    const double surface = m_wheelOmega * rw;
    const double vRef = std::max({std::abs(m_speed), std::abs(surface), kSlipFloor});
    const double kappa = (surface - m_speed) / vRef;
    m_tractionForce = m_p.tyreGrip * load * std::sin(1.62 * std::atan(7.4 * kappa));
    m_wheelSlip = std::clamp(std::abs(kappa), 0.0, 1.0);

    // ---- Driven wheels ------------------------------------------------------
    m_wheelOmega += (m_wheelTorque - m_tractionForce * rw) / Iw * dt;
    // Brakes, as an impulse that can bring a wheel to rest but never drive it
    // backwards through zero. The driven axle's brakes act on the wheel being
    // integrated here; the other axle has nothing spinning in this model, so
    // its share acts straight on the car below.
    const double brakeOnWheel = m_brake * m_p.brakeTorque * m_p.driveShare;
    if (brakeOnWheel > 0.0 && m_wheelOmega != 0.0) {
        const double dOmega = brakeOnWheel / Iw * dt;
        if (dOmega >= std::abs(m_wheelOmega)) m_wheelOmega = 0.0;
        else m_wheelOmega -= std::copysign(dOmega, m_wheelOmega);
    }

    // ---- Vehicle ------------------------------------------------------------
    const double drag    = 0.5 * kAirDensity * m_p.dragArea * m_speed * std::abs(m_speed);
    const double rolling = std::abs(m_speed) > 0.05 ? m_p.rollingCoef * m_p.mass * kGravity : 0.0;
    // The undriven axle can only ever brake, and only as hard as its own share
    // of the weight will hold.
    const double otherLoad = std::max(0.0, (1.0 - m_p.driveShare) * m_p.mass * kGravity - transfer);
    const double brakeForce = std::min(m_brake * m_p.brakeTorque * (1.0 - m_p.driveShare) / rw,
                                       m_p.tyreGrip * otherLoad);

    double net = m_tractionForce - drag;
    if (m_speed > 0.001)       net -= rolling + brakeForce;
    else if (m_speed < -0.001) net += rolling + brakeForce;

    m_accel += (net / m_p.mass - m_accel) * std::min(1.0, 12.0 * dt);
    m_speed += net / m_p.mass * dt;
    if (m_speed < 0.0) m_speed = 0.0;   // no reverse, and no creeping backwards

    return torqueOnCrank;
}

} // namespace sim
