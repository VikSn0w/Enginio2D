#pragma once
#include "sim/Engine.h"

#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace sim {

struct DynoPoint {
    float rpm    = 0.0f;
    float torque = 0.0f;   // N m
    float power  = 0.0f;   // kW
};

// One cell of the part-load map: what the engine does at a given speed and
// throttle opening, and what it burns to do it.
struct MapCell {
    float rpm      = 0.0f;
    float throttle = 0.0f;  // 0..1
    float torque   = 0.0f;  // N m
    float power    = 0.0f;  // kW
    float bsfc     = 0.0f;  // g/kWh, 0 where the engine makes no useful power
};

// ---------------------------------------------------------------------------
// A steady-state dynamometer, run on a worker thread against a private copy of
// the engine so the one you are listening to keeps running.
//
// Each point is measured the way a real engine dyno measures one: hold the
// crank at a fixed speed with the throttle wide open, wait for the manifold,
// the pipes and the turbo to settle, then average the torque the brake has to
// absorb. That is the only honest way to answer "how much power does this
// make" for a design, because power is not something you set - it is what the
// geometry, the cam, the fuel and the boost add up to.
// ---------------------------------------------------------------------------
class Dyno {
public:
    static constexpr int kMaxPoints = 48;

    Dyno() = default;
    ~Dyno();
    Dyno(const Dyno&) = delete;
    Dyno& operator=(const Dyno&) = delete;

    // Sweeps from just above idle to the redline. Cancels any run in progress.
    void start(const EngineParams& params, int points = 26);

    // Wide-open throttle is not where anything is driven. This sweeps speed
    // against throttle opening and records specific fuel consumption in each
    // cell, which is the map an engine is actually judged on: where it is
    // efficient, and how big that island is.
    void startMap(const EngineParams& params, int rpmPoints = 9, int loadPoints = 6);
    bool  hasMap() const { return m_mapCount.load(std::memory_order_acquire) > 0; }
    int   mapRpmPoints()  const { return m_mapRpmPoints; }
    int   mapLoadPoints() const { return m_mapLoadPoints; }
    MapCell mapCell(int loadIndex, int rpmIndex) const;
    void cancel();

    bool  running()  const { return m_running.load(std::memory_order_acquire); }
    float progress() const { return m_progress.load(std::memory_order_relaxed); }
    // True once a sweep has completed and its results are readable.
    bool  hasResult() const { return m_count.load(std::memory_order_acquire) > 0; }

    int count() const { return m_count.load(std::memory_order_acquire); }
    DynoPoint point(int i) const;

    float peakPowerKw()    const { return m_peakPowerKw.load(std::memory_order_relaxed); }
    float peakPowerRpm()   const { return m_peakPowerRpm.load(std::memory_order_relaxed); }
    float peakTorqueNm()   const { return m_peakTorqueNm.load(std::memory_order_relaxed); }
    float peakTorqueRpm()  const { return m_peakTorqueRpm.load(std::memory_order_relaxed); }
    // The design the last completed sweep was measured on, so the display can
    // say when the curve on screen no longer matches the engine on the bench.
    const std::string& label() const { return m_label; }
    void setLabel(const std::string& s) { m_label = s; }

private:
    void run(EngineParams params, int points);
    void runMap(EngineParams params, int rpmPoints, int loadPoints);

    std::thread m_worker;
    std::atomic<bool>  m_running{false};
    std::atomic<bool>  m_stop{false};
    std::atomic<float> m_progress{0.0f};
    std::atomic<int>   m_count{0};
    std::atomic<float> m_peakPowerKw{0.0f}, m_peakPowerRpm{0.0f};
    std::atomic<float> m_peakTorqueNm{0.0f}, m_peakTorqueRpm{0.0f};
    // Written by the worker before m_count is published, read afterwards.
    DynoPoint m_points[kMaxPoints];
    static constexpr int kMaxMapCells = 128;
    std::atomic<int> m_mapCount{0};
    int m_mapRpmPoints = 0, m_mapLoadPoints = 0;
    MapCell m_map[kMaxMapCells];
    std::string m_label;
};

} // namespace sim
