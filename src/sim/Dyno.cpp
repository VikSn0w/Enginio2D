#include "sim/Dyno.h"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {
constexpr double kPi = 3.14159265358979323846;
// Coarse enough to sweep quickly, fine enough that Engine::advance still
// sub-steps to well under a degree of crank rotation per integration step.
constexpr double kStepDt = 1.0 / 6000.0;
} // namespace

Dyno::~Dyno() { cancel(); }

void Dyno::cancel() {
    m_stop.store(true, std::memory_order_release);
    if (m_worker.joinable()) m_worker.join();
    m_stop.store(false, std::memory_order_release);
    m_running.store(false, std::memory_order_release);
}

DynoPoint Dyno::point(int i) const {
    if (i < 0 || i >= m_count.load(std::memory_order_acquire)) return {};
    return m_points[i];
}

void Dyno::start(const EngineParams& params, int points) {
    cancel();
    m_count.store(0, std::memory_order_release);
    m_progress.store(0.0f, std::memory_order_relaxed);
    m_running.store(true, std::memory_order_release);
    m_worker = std::thread(&Dyno::run, this, params, std::clamp(points, 4, kMaxPoints));
}

void Dyno::run(EngineParams params, int points) {
    Engine engine(params);
    engine.warmUp();
    engine.setIgnition(true);
    engine.setThrottle(1.0);
    engine.drivetrain().setGear(0);      // the brake is the only load on the bench

    const double lo = std::max(700.0, params.idleTargetRpm * 1.15);
    const double hi = std::max(lo + 500.0, params.redline);

    float bestPower = 0.0f, bestPowerRpm = 0.0f;
    float bestTorque = 0.0f, bestTorqueRpm = 0.0f;

    for (int i = 0; i < points; ++i) {
        if (m_stop.load(std::memory_order_acquire)) break;

        const double rpm = lo + (hi - lo) * i / std::max(1, points - 1);
        engine.setSpeedHold(true, rpm);

        // Settle, then measure. The settle window has to cover several complete
        // cycles at the lowest speed on the sweep, or the first points come back
        // reading whatever the previous one left in the pipes.
        const double cycleTime = 120.0 / rpm;             // one four-stroke cycle
        const double settle = std::max(0.22, 6.0 * cycleTime);
        const double measure = std::max(0.14, 4.0 * cycleTime);

        for (double t = 0.0; t < settle; t += kStepDt) engine.advance(kStepDt);

        double sum = 0.0;
        int n = 0;
        for (double t = 0.0; t < measure; t += kStepDt) {
            engine.advance(kStepDt);
            sum += engine.heldTorque();
            ++n;
        }

        const float torque = static_cast<float>(n > 0 ? sum / n : 0.0);
        const float power  = static_cast<float>(torque * rpm * 2.0 * kPi / 60.0 * 0.001);
        m_points[i] = {static_cast<float>(rpm), torque, power};

        if (power > bestPower)  { bestPower = power;   bestPowerRpm = static_cast<float>(rpm); }
        if (torque > bestTorque){ bestTorque = torque; bestTorqueRpm = static_cast<float>(rpm); }

        m_progress.store(static_cast<float>(i + 1) / points, std::memory_order_relaxed);
        // Publishing the count as we go lets the curve draw itself while the
        // sweep is still running, which is what a real dyno display does.
        m_count.store(i + 1, std::memory_order_release);
        m_peakPowerKw.store(bestPower, std::memory_order_relaxed);
        m_peakPowerRpm.store(bestPowerRpm, std::memory_order_relaxed);
        m_peakTorqueNm.store(bestTorque, std::memory_order_relaxed);
        m_peakTorqueRpm.store(bestTorqueRpm, std::memory_order_relaxed);
    }

    m_progress.store(1.0f, std::memory_order_relaxed);
    m_running.store(false, std::memory_order_release);
}

} // namespace sim
