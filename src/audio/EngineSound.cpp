#include "audio/EngineSound.h"

#include <algorithm>
#include <cmath>

namespace audio {
namespace {

constexpr float kHotSound = 480.0f;   // m/s in exhaust gas around 800 K
constexpr float kPi       = 3.14159265358979323846f;

// Asymmetric saturation. A real exhaust steepens the front of a pressure pulse
// far more than its back, and that asymmetry is a large part of why an engine
// sounds hard rather than like a filtered buzz.
inline float saturate(float x) {
    const float k = x > 0.0f ? 1.0f : 1.6f;
    const float y = x * k;
    return (y / (1.0f + std::abs(y))) / k;
}

} // namespace

// ---------------------------------------------------------------------------
void Waveguide::init(float lengthMetres, float sampleRate, float openReflect,
                     float closedReflect, float wallDamping) {
    const std::size_t n = std::max<std::size_t>(
        4, static_cast<std::size_t>(lengthMetres / kHotSound * sampleRate));
    m_fwd.assign(n, 0.0f);
    m_bwd.assign(n, 0.0f);
    m_i = 0;
    m_openReflect   = openReflect;
    m_closedReflect = closedReflect;
    m_wall          = wallDamping;
    m_lpOpen = m_lpWall = 0.0f;
}

void Waveguide::reset() {
    std::fill(m_fwd.begin(), m_fwd.end(), 0.0f);
    std::fill(m_bwd.begin(), m_bwd.end(), 0.0f);
    m_lpOpen = m_lpWall = 0.0f;
}

float Waveguide::tick(float in) {
    const float atOpen   = m_fwd[m_i];   // wave arriving at the open end
    const float atClosed = m_bwd[m_i];   // reflection arriving back at the port

    // The open end is not a mirror: high frequencies radiate away, low ones
    // bounce. One pole on the reflected wave stands in for that radiation
    // impedance.
    m_lpOpen += 0.45f * (atOpen - m_lpOpen);
    const float reflected = m_openReflect * m_lpOpen;

    // Wall losses on the return traverse, so a long pipe goes dull instead of
    // ringing like a comb filter. This is most of the difference between a
    // waveguide that sounds like a pipe and one that sounds like a toy.
    m_lpWall += m_wall * (atClosed - m_lpWall);

    m_bwd[m_i] = reflected;
    m_fwd[m_i] = in + m_closedReflect * m_lpWall;
    m_i = (m_i + 1) % m_fwd.size();

    return atOpen - reflected;   // what leaves the pipe
}

// ---------------------------------------------------------------------------
void SVF::set(float cutoffHz, float q, float sampleRate) {
    // Chamberlin topology: stable while damp < 2 - f, so the damping is
    // clamped against the tuning rather than trusted.
    m_f = 2.0f * std::sin(kPi * std::min(cutoffHz, sampleRate * 0.45f) / sampleRate);
    m_damp = std::clamp(1.0f / std::max(q, 0.25f), 0.05f, 1.9f - m_f);
}

float SVF::band(float in) {
    m_lo += m_f * m_bp;
    const float hi = in - m_lo - m_damp * m_bp;
    m_bp += m_f * hi;
    return m_bp;
}

float SVF::low(float in) {
    m_lo += m_f * m_bp;
    const float hi = in - m_lo - m_damp * m_bp;
    m_bp += m_f * hi;
    return m_lo;
}

// ---------------------------------------------------------------------------
void Synth::init(std::size_t cylinders, float sampleRate) {
    m_cylCount = std::min(kMaxCyl, cylinders);
    for (std::size_t i = 0; i < m_cylCount; ++i) {
        // Primaries of slightly unequal length, as on any real manifold. The
        // mismatch is what keeps the pulses from stacking into one
        // synthetic-sounding tone.
        const float len = 0.82f + 0.035f * static_cast<float>(i);
        m_primary[i].init(len, sampleRate, -0.40f, 0.52f, 0.42f);
        m_primaryGain[i] = 0.94f + 0.03f * static_cast<float>(i % 3);
    }

    m_collector.init(1.05f, sampleRate, -0.42f, 0.52f, 0.34f);
    m_tailpipe.init(0.72f, sampleRate, -0.34f, 0.48f, 0.26f);
    m_intakePipe.init(0.40f, sampleRate, -0.60f, 0.85f, 0.50f);

    m_valveFilter.set(3200.0f, 1.4f, sampleRate);
    m_whirrFilter.set(1800.0f, 0.8f, sampleRate);
    m_thumpFilter.set(320.0f, 1.1f, sampleRate);
    m_intakeVoice.set(900.0f, 0.9f, sampleRate);
}

float Synth::noise() {
    m_rng ^= m_rng << 13; m_rng ^= m_rng >> 17; m_rng ^= m_rng << 5;
    return static_cast<float>(static_cast<std::int32_t>(m_rng)) * (1.0f / 2147483648.0f);
}

void Synth::step(const sim::Engine& engine, float gain, float& left, float& right) {
    const auto& cyls = engine.cylinders();
    const float rpm = static_cast<float>(engine.rpm());
    const float speedFactor = std::clamp(rpm / 6000.0f, 0.0f, 1.4f);

    // ---- Exhaust: one primary per cylinder into a shared collector ----------
    float exhaust = 0.0f;
    for (std::size_t i = 0; i < m_cylCount; ++i) {
        const float raw = static_cast<float>(engine.exhaustRunnerGauge(i)) * 1.0e-4f;
        const float src = raw - m_runnerDcIn[i] + 0.9985f * m_runnerDcOut[i];
        m_runnerDcIn[i]  = raw;
        m_runnerDcOut[i] = src;
        exhaust += m_primaryGain[i] * m_primary[i].tick(src);

        // ---- Mechanical events ---------------------------------------------
        const sim::Cylinder& c = cyls[i];
        const float il = static_cast<float>(c.intakeLift);
        const float el = static_cast<float>(c.exhaustLift);
        // A valve seating is an impact, and impact energy goes with the square
        // of closing speed, so the clatter grows with engine speed.
        if (m_lastIntakeLift[i] > 1.0e-5f && il <= 1.0e-5f)
            m_valveBurst.trigger(0.05f + 0.22f * speedFactor, 0.9965f);
        if (m_lastExhaustLift[i] > 1.0e-5f && el <= 1.0e-5f)
            m_valveBurst.trigger(0.06f + 0.26f * speedFactor, 0.9962f);
        m_lastIntakeLift[i]  = il;
        m_lastExhaustLift[i] = el;

        // The pressure rise of combustion shakes the whole block, and that is
        // most of what you hear standing next to an idling engine.
        const float burn = static_cast<float>(c.burnFraction);
        if (burn > 0.02f && m_lastBurn[i] <= 0.02f)
            m_combustionBurst.trigger(0.35f, 0.9988f);
        m_lastBurn[i] = burn;
    }

    float x = m_collector.tick(exhaust * 0.5f);

    // Muffler: two cascaded poles, then the tailpipe's own resonance.
    m_muffler1 += 0.34f * (x - m_muffler1);
    m_muffler2 += 0.34f * (m_muffler1 - m_muffler2);
    x = m_tailpipe.tick(m_muffler2);

    // Radiation from an open pipe follows the rate of change of the wave rather
    // than the wave itself. Mixing in some of that derivative is what puts the
    // crack into the note.
    const float diff = x - m_radPrev;
    m_radPrev = x;
    x = x + 0.9f * diff;

    // ---- Intake -------------------------------------------------------------
    // Only the fluctuation radiates. The manifold sits 80 kPa below ambient on
    // a closed throttle, and feeding that standing offset in as signal made the
    // engine get *louder* when you lifted off.
    const float rawIntake = static_cast<float>(engine.intakeGauge()) * 0.45e-4f;
    const float intakeAc = rawIntake - m_intakeDcIn + 0.995f * m_intakeDcOut;
    m_intakeDcIn  = rawIntake;
    m_intakeDcOut = intakeAc;
    float intake = m_intakePipe.tick(intakeAc);
    intake = m_intakeVoice.band(intake) * 1.4f;

    // ---- Mechanical layer ---------------------------------------------------
    const float n = noise();
    const float valves = m_valveFilter.band(m_valveBurst.tick(n));
    const float thump  = m_thumpFilter.band(m_combustionBurst.tick(n));
    const float whirr  = m_whirrFilter.band(n) * 0.012f * speedFactor;

    // ---- Level --------------------------------------------------------------
    // Collector pulses really are ~50x stronger at full load than at idle. Left
    // alone that is unlistenable; driven hard into the saturator to compensate,
    // it becomes a constant buzz instead. So the level is set feed-forward from
    // a slow estimate of the pulse size in Pa, tapered by a quarter power:
    // about 26 dB of the range survives, and peaks stay just under the
    // saturator except at full load, where a little compression is what an
    // exhaust does anyway.
    // The estimate follows what actually drives the pipes - the summed primary
    // output - rather than the collector's absolute pressure, most of which is
    // steady backpressure and radiates nothing.
    m_level += (std::abs(exhaust) - m_level) * 2.5e-5f;   // ~1 s window
    const float taper = std::clamp(0.105f * std::pow(std::max(m_level, 1.0e-4f), -0.25f),
                                   0.02f, 6.0f);

    float body = x * taper;

    // Block the standing pressure offset: only the fluctuation radiates.
    const float dc = body - m_dcPrevIn + 0.997f * m_dcPrevOut;
    m_dcPrevIn  = body;
    m_dcPrevOut = dc;
    body = saturate(dc);

    const float mono = body + valves * 0.10f + thump * 0.17f + whirr * 0.35f + intake * 0.10f;

    // ---- Stereo -------------------------------------------------------------
    // The tailpipe is behind and to one side, the intake ahead of the other
    // ear; a fraction of a millisecond of delay is enough to open it up.
    m_widthDelay[m_widthIndex] = mono;
    const std::size_t tapIdx = (m_widthIndex + 64 - 18) % 64;
    const float delayed = m_widthDelay[tapIdx];
    m_widthIndex = (m_widthIndex + 1) % 64;

    left  = std::clamp((mono * 0.78f + delayed * 0.22f + intake * 0.22f) * gain, -1.0f, 1.0f);
    right = std::clamp((mono * 0.94f + delayed * 0.06f - intake * 0.10f) * gain, -1.0f, 1.0f);
}

// ---------------------------------------------------------------------------
EngineSound::EngineSound(sim::Engine& engine) : m_engine(engine) {
    m_buffer.resize(kChunk * 2);        // stereo, interleaved
    m_synth.init(engine.cylinders().size(), static_cast<float>(kSampleRate));
    for (auto& s : m_scope) s.store(0.0f, std::memory_order_relaxed);
    initialize(2, kSampleRate, {sf::SoundChannel::FrontLeft, sf::SoundChannel::FrontRight});
}

bool EngineSound::onGetData(Chunk& chunk) {
    const double dt = 1.0 / kSampleRate;

    m_engine.setThrottle(m_throttle.load(std::memory_order_relaxed));
    m_engine.setStarter(m_starterCmd.load(std::memory_order_relaxed));
    m_engine.setIgnition(m_ignitionCmd.load(std::memory_order_relaxed));
    m_engine.drivetrain().setBrake(m_brakeCmd.load(std::memory_order_relaxed));
    const int gearReq = m_gearRequest.exchange(-1, std::memory_order_relaxed);
    if (gearReq >= 0) m_engine.drivetrain().setGear(gearReq);

    const float gain = m_gain.load(std::memory_order_relaxed);
    std::size_t head = m_scopeHead.load(std::memory_order_relaxed);

    for (std::size_t f = 0; f < kChunk; ++f) {
        m_engine.advance(dt);

        float left = 0.0f, right = 0.0f;
        m_synth.step(m_engine, gain, left, right);

        m_buffer[f * 2 + 0] = static_cast<std::int16_t>(left  * 31000.0f);
        m_buffer[f * 2 + 1] = static_cast<std::int16_t>(right * 31000.0f);

        head = (head + 1) % kScope;
        m_scope[head].store(0.5f * (left + right), std::memory_order_relaxed);
    }

    m_scopeHead.store(head, std::memory_order_release);

    const int spare = 1 - m_snapIndex.load(std::memory_order_relaxed);
    m_snap[spare] = m_engine.snapshot();
    m_snapIndex.store(spare, std::memory_order_release);

    chunk.samples     = m_buffer.data();
    chunk.sampleCount = m_buffer.size();
    return true;
}

sim::Snapshot EngineSound::snapshot() const {
    return m_snap[m_snapIndex.load(std::memory_order_acquire)];
}

} // namespace audio
