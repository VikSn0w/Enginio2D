#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace sim {

// ---------------------------------------------------------------------------
// The travelling wave in a runner.
//
// The lumped runner volume next door carries the mass and the energy, and it
// gets the steady flow right - but it has no length in time, so nothing it does
// can depend on *when* a pulse arrives back. That is the whole of exhaust
// tuning and most of intake ram: the port launches a wave, the far end sends it
// back inverted, and if it returns while the valve is still open it either
// packs the cylinder or scavenges it. A lumped element cannot express that, no
// matter what length is written on it.
//
// So this carries the timing and nothing else. A port launches
//
//     p+ = c * mdot_into_the_pipe / A          (rho c u, in Pa)
//
// which is remembered for the round trip 2L/c and comes back scaled by the
// reflection coefficient of the far end - negative at both, because a runner
// opens into something much larger at each end, and an area expansion inverts
// a pulse. The result is added to the static runner pressure when the valve
// flow is worked out, which is exactly what it physically is: the pressure at
// the valve differing from the mean pressure in the pipe.
// ---------------------------------------------------------------------------
class PressureWave {
public:
    // The line runs on its own fixed tick so the delay stays a fixed time no
    // matter how the solver sub-steps, which it does differently at every
    // engine speed.
    static constexpr double kTick = 1.0 / 44100.0;

    void configure(double lengthM, double areaM2, double reflect, double damping,
                   double strength) {
        m_length  = std::max(0.02, lengthM);
        m_area    = std::max(1.0e-5, areaM2);
        m_reflect = std::clamp(reflect, -0.95, 0.95);
        m_damp    = std::clamp(damping, 0.02, 0.9);
        m_strength = std::clamp(strength, 0.0, 2.0);
        // Long enough for the slowest sound speed and the longest pipe allowed.
        m_buf.assign(2048, 0.0);
        reset();
    }

    void reset() {
        std::fill(m_buf.begin(), m_buf.end(), 0.0);
        m_head = 0;
        m_acc = m_flowAcc = 0.0;
        m_return = 0.0;
        m_lp = 0.0;
        m_slow = 0.0;
    }

    // portFlow is the mass flow into the pipe at the valve, kg/s: positive when
    // the cylinder is blowing into it, negative when it is drawing out of it.
    // Returns the pressure perturbation now arriving back at the valve, Pa.
    double step(double dt, double portFlow, double soundSpeed, double staticPressure) {
        if (m_buf.empty()) return 0.0;

        m_acc += dt;
        m_flowAcc += portFlow * dt;

        while (m_acc >= kTick) {
            m_acc -= kTick;
            const double meanFlow = m_flowAcc / kTick;
            m_flowAcc = 0.0;

            // Only the transient part of the port flow launches a wave. The
            // lumped runner next door already carries the quasi-steady inertia
            // of the column, so propagating the whole flow as well counts it
            // twice and turns a tuned pipe into a supercharger at every speed.
            // What is left after the slow part is removed is what a real pipe
            // sees: the crack of blowdown, and the step when a valve opens.
            const double raw = m_strength * soundSpeed * meanFlow / m_area;
            const double tau = 4.0 * 2.0 * m_length / std::max(60.0, soundSpeed);
            m_slow += (raw - m_slow) * std::clamp(kTick / std::max(tau, kTick), 0.0, 1.0);

            m_head = (m_head + 1) % m_buf.size();
            m_buf[m_head] = raw - m_slow;

            // Round trip, in ticks, at the current sound speed - which is why a
            // hot exhaust tunes differently from a cold one.
            const double delay = 2.0 * m_length / std::max(60.0, soundSpeed) / kTick;
            const std::size_t n = static_cast<std::size_t>(
                std::clamp(delay, 1.0, static_cast<double>(m_buf.size() - 2)));
            const double arrived = m_buf[(m_head + m_buf.size() - n) % m_buf.size()];

            // The far end is not a perfect mirror: it radiates, and the pipe
            // rounds the pulse off on the way. That smearing matters as much as
            // the reflection does - a collector is not a point, so what comes
            // back is a broadened version of what left, and a broader pulse
            // cannot swing the port as violently as a sharp one.
            m_lp += m_damp * (arrived - m_lp);
            m_return = m_reflect * m_lp;
        }

        // A returning wave can be a large fraction of the static pressure, but
        // it can never be so large that it drives the absolute pressure
        // negative, and clamping it here is what keeps the coupling stable.
        const double limit = 0.55 * std::max(staticPressure, 1.0e3);
        return std::clamp(m_return, -limit, limit);
    }

    double roundTripSeconds(double soundSpeed) const {
        return 2.0 * m_length / std::max(60.0, soundSpeed);
    }

private:
    std::vector<double> m_buf;
    std::size_t m_head = 0;
    double m_acc = 0.0, m_flowAcc = 0.0;
    double m_length = 0.3, m_area = 1.0e-3;
    double m_reflect = -0.5, m_damp = 0.25, m_strength = 0.5;
    double m_return = 0.0, m_lp = 0.0, m_slow = 0.0;
};

} // namespace sim
