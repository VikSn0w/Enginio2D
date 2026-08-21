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
    double shiftTime    = 0.14;    // s the dogs and synchros need once the lever moves
    double brakeTorque  = 2600.0;  // N m at the wheels, full brake

    // ---- Clutch pedal ------------------------------------------------------
    // The pedal is the driver's, not the simulation's: 0 is fully up (the disc
    // clamped, the engine and the gearbox one shaft) and 1 is on the floor (the
    // disc free). The two numbers below are where in that travel the disc
    // actually does something, which is what a driver means by the bite point.
    // Everything above freePlay is slack, everything below bite is fully home.
    double clutchFreePlay = 0.18;  // pedal position at which it is fully clamped
    double clutchBite     = 0.62;  // pedal position at which it lets go entirely
    // How far down the pedal has to be before the lever will go into a gear.
    // A synchro box can be shifted without it if the speeds happen to match,
    // but not reliably, and not without a noise.
    double shiftClutch    = 0.55;

    // ---- Tyres and rotating mass -------------------------------------------
    // Wheel torque only becomes acceleration while the tyres can hold it:
    // beyond that they spin, which is why first gear in anything quick is a
    // question of grip rather than of power.
    double tyreGrip     = 1.05;    // peak friction coefficient
    double driveShare   = 0.52;    // fraction of static weight over the driven wheels
    bool   driveRear    = true;    // which end is driven, for the weight transfer
    double cgHeightRatio= 0.22;    // CG height / wheelbase, sets how much transfers
    double wheelInertia = 1.6;     // kg m^2, driven wheels + hubs + brakes + shafts
    double gearboxInertia = 0.022; // kg m^2 on the gearbox input side
    double transmissionEff = 0.92; // gears and final drive
};

// Clutch, gearbox, tyres and vehicle.
//
// The clutch is a friction coupling the driver holds open or lets home; it is
// not something the simulation decides for itself. That is the difference
// between a car you can stall and a car that merely accelerates: pulling away
// is feeding a slipping disc enough torque to get the mass moving without
// dragging the crank below the speed it can keep running at.
//
// The driven wheels carry their own rotational state rather than being pinned
// to road speed. Without that a spinning tyre still loads the engine exactly as
// a gripping one does, and dropping the clutch in first produces a shove
// instead of noise and smoke.
class Drivetrain {
public:
    explicit Drivetrain(DrivetrainParams p = {}) : m_p(std::move(p)) {}

    // Swap in new ratios and vehicle data while running. A gear that no longer
    // exists drops to the highest one that does.
    void setParams(const DrivetrainParams& p);
    const DrivetrainParams& params() const { return m_p; }

    // Returns false when the lever would not go in, which is what happens when
    // the clutch is not down far enough to free the gearset.
    bool setGear(int g);
    bool shiftUp()   { return setGear(m_gear + 1); }
    bool shiftDown() { return setGear(m_gear - 1); }
    void setBrake(double b);
    // 0 = pedal up, the clutch clamped; 1 = pedal on the floor, the clutch free.
    void setClutchPedal(double p);

    // Advance the vehicle and return the torque the clutch imposes on the
    // crank (negative while the engine is driving the car). The crank's inertia
    // is wanted so the disc's friction can be integrated without overshooting
    // lock-up, which is what a stiff clutch on a light flywheel does otherwise.
    double step(double dt, double engineOmega, double engineInertia);

    int    gear() const        { return m_gear; }
    int    gearCount() const   { return static_cast<int>(m_p.gearRatios.size()); }
    double speed() const       { return m_speed; }               // m/s
    double speedKph() const    { return m_speed * 3.6; }
    double clutchPedal() const { return m_pedal; }               // 0..1
    double clutchLock() const  { return m_clutch; }              // 0..1, torque capacity in use
    double slip() const        { return m_slip; }                // rad/s across the disc
    double wheelTorque() const { return m_wheelTorque; }         // N m
    double wheelSlip() const   { return m_wheelSlip; }           // 0 = gripping, 1 = spinning
    double wheelOmega() const  { return m_wheelOmega; }          // rad/s, driven wheels
    double tractionForce() const { return m_tractionForce; }     // N at the contact patch
    double brake() const       { return m_brake; }
    // Decays after a shift the gearbox refused, for the readout and the noise.
    double grind() const       { return m_grind; }
    bool   shifting() const    { return m_shiftTimer > 0.0; }
    // Crank speed the gearing implies for the current driven-wheel speed.
    double inputOmega() const  { return m_inputOmega; }

private:
    double ratio() const;
    double clutchCapacityNow() const;

    DrivetrainParams m_p;
    int    m_gear       = 0;      // 0 = neutral
    double m_speed      = 0.0;    // m/s
    double m_wheelOmega = 0.0;    // rad/s, driven wheels
    double m_brake      = 0.0;
    double m_pedal      = 0.0;    // clutch pedal, 0 = up
    double m_clutch     = 0.0;    // fraction of capacity the disc can carry
    double m_slip       = 0.0;    // rad/s
    double m_wheelTorque= 0.0;    // N m delivered to the driven wheels
    double m_wheelSlip  = 0.0;    // 0 = gripping, 1 = spinning
    double m_tractionForce = 0.0; // N
    double m_accel      = 0.0;    // m/s^2, filtered, for the weight transfer
    double m_inputOmega = 0.0;
    double m_shiftTimer = 0.0;
    double m_grind      = 0.0;
};

} // namespace sim
