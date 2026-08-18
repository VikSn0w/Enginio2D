#pragma once
#include <vector>

namespace sim {

struct DrivetrainParams {
    // Six forward ratios; index 0 is first gear. Gear 0 is neutral.
    std::vector<double> gearRatios{3.55, 2.10, 1.45, 1.08, 0.88, 0.72};
    double finalDrive   = 3.90;
    double wheelRadius  = 0.31;   // m
    double mass         = 1250.0; // kg
    double dragArea     = 0.62;   // Cd * A, m^2
    double rollingCoef  = 0.013;
    double clutchCapacity = 340.0; // N m the clutch can hold before it slips
    double shiftTime    = 0.35;    // s with the clutch out
    double lockRpm      = 1300.0;  // clutch fully home above this
    double slipRpm      = 550.0;   // below this it is fully open, so no stalling
    double brakeTorque  = 2600.0;  // N m at the wheels, full brake
};

// Clutch, gearbox and vehicle. The clutch is a slipping friction coupling
// rather than a rigid connection, which is what lets the engine idle in gear,
// pull away from rest, and be shifted without the crank speed jumping.
class Drivetrain {
public:
    explicit Drivetrain(DrivetrainParams p = {}) : m_p(std::move(p)) {}

    // Swap in new ratios and vehicle data while running. A gear that no longer
    // exists drops to the highest one that does.
    void setParams(const DrivetrainParams& p) {
        m_p = p;
        if (m_gear > gearCount()) m_gear = gearCount();
    }
    const DrivetrainParams& params() const { return m_p; }

    void setGear(int g);
    void shiftUp()   { setGear(m_gear + 1); }
    void shiftDown() { setGear(m_gear - 1); }
    void setBrake(double b);

    // Advance the vehicle and return the torque the clutch imposes on the
    // crank (negative while the engine is driving the car).
    double step(double dt, double engineOmega);

    int    gear() const        { return m_gear; }
    int    gearCount() const   { return static_cast<int>(m_p.gearRatios.size()); }
    double speed() const       { return m_speed; }               // m/s
    double speedKph() const    { return m_speed * 3.6; }
    double clutchLock() const  { return m_clutch; }              // 0..1
    double slip() const        { return m_slip; }                // rad/s
    double wheelTorque() const { return m_wheelTorque; }         // N m
    double brake() const       { return m_brake; }
    bool   shifting() const    { return m_shiftTimer > 0.0; }
    // Crank speed the gearing implies for the current road speed.
    double inputOmega() const  { return m_inputOmega; }

private:
    double ratio() const;

    DrivetrainParams m_p;
    int    m_gear       = 0;      // 0 = neutral
    double m_speed      = 0.0;    // m/s
    double m_brake      = 0.0;
    double m_clutch     = 0.0;
    double m_slip       = 0.0;
    double m_wheelTorque= 0.0;
    double m_inputOmega = 0.0;
    double m_shiftTimer = 0.0;
};

} // namespace sim
