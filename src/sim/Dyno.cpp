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
    m_mapCount.store(0, std::memory_order_release);
    m_count.store(0, std::memory_order_release);
    m_progress.store(0.0f, std::memory_order_relaxed);
    m_running.store(true, std::memory_order_release);
    m_worker = std::thread(&Dyno::run, this, params, std::clamp(points, 4, kMaxPoints));
}

MapCell Dyno::mapCell(int loadIndex, int rpmIndex) const {
    const int i = loadIndex * m_mapRpmPoints + rpmIndex;
    if (i < 0 || i >= m_mapCount.load(std::memory_order_acquire)) return {};
    return m_map[i];
}

void Dyno::startMap(const EngineParams& params, int rpmPoints, int loadPoints) {
    cancel();
    m_mapCount.store(0, std::memory_order_release);
    m_count.store(0, std::memory_order_release);
    m_progress.store(0.0f, std::memory_order_relaxed);
    m_stop.store(false, std::memory_order_release);
    m_running.store(true, std::memory_order_release);
    m_worker = std::thread(&Dyno::runMap, this, params, rpmPoints, loadPoints);
}

void Dyno::runMap(EngineParams params, int rpmPoints, int loadPoints) {
    rpmPoints  = std::clamp(rpmPoints, 2, 16);
    loadPoints = std::clamp(loadPoints, 2, 8);
    m_mapRpmPoints  = rpmPoints;
    m_mapLoadPoints = loadPoints;

    Engine engine(params);
    engine.warmUp();
    engine.setIgnition(true);
    engine.drivetrain().setGear(0);

    const double lo = std::max(700.0, params.idleTargetRpm * 1.15);
    const double hi = std::max(lo + 500.0, params.redline);
    const int total = rpmPoints * loadPoints;

    for (int li = 0; li < loadPoints; ++li) {
        // Below about a sixth of throttle most engines are pumping against a
        // closed plate and the numbers say more about the throttle than the
        // engine, so the map starts above that.
        const double throttle = 0.16 + (1.0 - 0.16) * li / std::max(1, loadPoints - 1);
        for (int ri = 0; ri < rpmPoints; ++ri) {
            if (m_stop.load(std::memory_order_acquire)) {
                m_running.store(false, std::memory_order_release);
                return;
            }
            const double rpm = lo + (hi - lo) * ri / std::max(1, rpmPoints - 1);
            engine.setThrottle(throttle);
            engine.setSpeedHold(true, rpm);

            const double cycleTime = 120.0 / rpm;
            const double settle  = std::max(0.25, 7.0 * cycleTime);
            const double measure = std::max(0.16, 5.0 * cycleTime);
            for (double t = 0.0; t < settle; t += kStepDt) engine.advance(kStepDt);

            double sumT = 0.0, sumFuel = 0.0;
            int n = 0;
            for (double t = 0.0; t < measure; t += kStepDt) {
                engine.advance(kStepDt);
                sumT += engine.heldTorque();
                sumFuel += engine.snapshot().fuelFlow;   // kg/h
                ++n;
            }
            const float torque = static_cast<float>(n > 0 ? sumT / n : 0.0);
            const float power  = static_cast<float>(torque * rpm * 2.0 * kPi / 60.0 * 0.001);
            const float fuel   = static_cast<float>(n > 0 ? sumFuel / n : 0.0);
            // Specific consumption is meaningless where the engine is not
            // producing anything: at light load and low speed it is only just
            // turning itself over.
            const float bsfc = power > 0.5f ? fuel * 1000.0f / power : 0.0f;

            const int i = li * rpmPoints + ri;
            if (i < kMaxMapCells)
                m_map[i] = {static_cast<float>(rpm), static_cast<float>(throttle),
                            torque, power, bsfc};
            m_progress.store(static_cast<float>(i + 1) / total, std::memory_order_relaxed);
            m_mapCount.store(i + 1, std::memory_order_release);
        }
    }

    m_progress.store(1.0f, std::memory_order_relaxed);
    m_running.store(false, std::memory_order_release);
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
