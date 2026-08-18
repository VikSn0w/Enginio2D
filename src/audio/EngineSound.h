#pragma once
#include <SFML/Audio/SoundStream.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

#include "sim/Engine.h"

namespace audio {

// ---------------------------------------------------------------------------
// A length of pipe as a pair of delay lines (digital waveguide): one carrying
// the wave travelling away from the engine, one carrying the reflection coming
// back. Length sets the resonance; the loop filter is what stops it sounding
// like a comb filter, because a real pipe loses high frequencies on every
// traverse and radiates them at the open end.
// ---------------------------------------------------------------------------
class Waveguide {
public:
    void init(float lengthMetres, float sampleRate, float openReflect,
              float closedReflect, float wallDamping);
    float tick(float in);
    void  reset();

private:
    std::vector<float> m_fwd, m_bwd;
    std::size_t m_i = 0;
    float m_openReflect   = -0.75f;
    float m_closedReflect =  0.92f;
    float m_wall          =  0.35f;  // per-traverse loss of highs
    float m_lpOpen        =  0.0f;
    float m_lpWall        =  0.0f;
};

// Two-pole state-variable filter. Used for the band-passed mechanical noises
// and for the final voicing.
class SVF {
public:
    void set(float cutoffHz, float q, float sampleRate);
    float band(float in);
    float low(float in);

private:
    float m_f = 0.1f, m_damp = 0.5f;
    float m_lo = 0.0f, m_bp = 0.0f;
};

// One-shot exponentially decaying noise burst: valve seating, combustion
// knock, injector tick.
class Burst {
public:
    void trigger(float amplitude, float decayPerSample) {
        m_amp = amplitude > m_amp ? amplitude : m_amp;
        m_decay = decayPerSample;
    }
    float tick(float noise) {
        if (m_amp < 1.0e-5f) return 0.0f;
        const float out = m_amp * noise;
        m_amp *= m_decay;
        return out;
    }

private:
    float m_amp = 0.0f, m_decay = 0.999f;
};

// ---------------------------------------------------------------------------
// The voice of the engine, independent of any audio device so it can also be
// rendered offline.
//
// Every cylinder gets its own primary pipe fed by that cylinder's simulated
// exhaust runner pressure. Those pipes are summed per bank before the
// collector, which is the detail that makes layout audible: a crossplane V8
// fires evenly overall but unevenly within each bank, and it is the bank that
// shares a collector.
//
// On top of the gas noise sit the mechanical sounds an engine actually makes:
// valves seating, the crack of each combustion event, detonation, and the
// broadband whirr of the valvetrain.
// ---------------------------------------------------------------------------
class Synth {
public:
    static constexpr std::size_t kMaxCyl   = sim::kMaxCylinders;
    static constexpr std::size_t kMaxBanks = 4;

    // Pipe lengths, damping and loudness all come out of the engine
    // specification, so re-voicing the exhaust is part of editing the engine.
    void init(const sim::EngineParams& p, float sampleRate);
    // Call once per sample, after the engine has been advanced by 1/sampleRate.
    void step(const sim::Engine& engine, float gain, float& left, float& right);
    // Slow estimate of how hard the pipes are being driven. Exposed for
    // offline level checks.
    float driveLevel() const { return m_level; }

private:
    float noise();

    std::array<Waveguide, kMaxCyl> m_primary;
    std::array<float, kMaxCyl> m_primaryGain{};
    std::array<int, kMaxCyl>   m_bank{};
    std::array<float, kMaxCyl> m_lastIntakeLift{};
    std::array<float, kMaxCyl> m_lastExhaustLift{};
    std::array<float, kMaxCyl> m_lastBurn{};
    std::array<float, kMaxCyl> m_lastKnock{};
    // Per-cylinder DC blockers: a runner sits well above ambient at full load,
    // and that steady backpressure is not a sound.
    std::array<float, kMaxCyl> m_runnerDcIn{};
    std::array<float, kMaxCyl> m_runnerDcOut{};
    std::size_t m_cylCount = 0;
    std::size_t m_bankCount = 1;

    std::array<Waveguide, kMaxBanks> m_collector;
    Waveguide m_tailpipe;
    Waveguide m_intakePipe;

    SVF m_valveFilter;      // valvetrain clatter
    SVF m_whirrFilter;      // chain and accessory hiss, its own state
    SVF m_thumpFilter;      // combustion body
    SVF m_knockFilter;      // detonation, much brighter and metallic
    SVF m_intakeVoice;
    Burst m_valveBurst;
    Burst m_combustionBurst;
    Burst m_knockBurst;

    std::uint32_t m_rng = 0x9E3779B9u;

    float m_mufflerCoef = 0.34f;
    float m_loudness    = 1.0f;
    float m_level     = 0.05f;    // slow estimate of how hard the pipes are driven
    float m_intakeDcIn  = 0.0f;   // the manifold's standing vacuum is not a sound
    float m_intakeDcOut = 0.0f;
    float m_dcPrevIn  = 0.0f;
    float m_dcPrevOut = 0.0f;
    float m_muffler1  = 0.0f;
    float m_muffler2  = 0.0f;
    float m_radPrev   = 0.0f;   // radiation differentiator state
    float m_widthDelay[64]{};   // short delay for stereo width
    std::size_t m_widthIndex = 0;
};

// Drives the simulation at audio rate on the audio thread and streams the
// result to the device.
class EngineSound : public sf::SoundStream {
public:
    static constexpr unsigned kSampleRate = 44100;
    static constexpr std::size_t kChunk   = 512;   // frames
    static constexpr std::size_t kScope   = 1024;

    explicit EngineSound(sim::Engine& engine);

    void setThrottle(float t) { m_throttle.store(t, std::memory_order_relaxed); }
    void setBrake(float b)    { m_brakeCmd.store(b, std::memory_order_relaxed); }
    void setStarter(bool s)   { m_starterCmd.store(s, std::memory_order_relaxed); }
    void setIgnition(bool i)  { m_ignitionCmd.store(i, std::memory_order_relaxed); }
    void setVolume01(float v) { m_gain.store(v, std::memory_order_relaxed); }
    // Gear changes are queued here and applied on the audio thread, which owns
    // the simulation.
    void requestGear(int g)   { m_gearRequest.store(g, std::memory_order_relaxed); }

    // Hand a new specification to the audio thread, which owns the engine and
    // is the only thread allowed to rebuild it. Returns false if a previous
    // request has not been picked up yet, in which case try again next frame -
    // that is what keeps the hand-off buffer from being written while it is
    // being read.
    bool requestConfig(const sim::EngineParams& p, const sim::DrivetrainParams& t);
    bool configPending() const {
        return m_configGen.load(std::memory_order_acquire) !=
               m_configAck.load(std::memory_order_acquire);
    }

    sim::Snapshot snapshot() const;
    float scopeAt(std::size_t i) const { return m_scope[i].load(std::memory_order_relaxed); }
    std::size_t scopeHead() const { return m_scopeHead.load(std::memory_order_relaxed); }

protected:
    bool onGetData(Chunk& chunk) override;
    void onSeek(sf::Time) override {}

private:
    sim::Engine& m_engine;
    Synth m_synth;
    std::vector<std::int16_t> m_buffer;   // interleaved stereo

    std::atomic<float> m_throttle{0.0f};
    std::atomic<float> m_brakeCmd{0.0f};
    std::atomic<float> m_gain{0.7f};
    std::atomic<bool>  m_starterCmd{false};
    std::atomic<bool>  m_ignitionCmd{true};
    std::atomic<int>   m_gearRequest{-1};

    // Configuration hand-off: written by the UI thread, read by the audio
    // thread, with a generation counter to publish and an acknowledgement to
    // stop the writer running ahead of the reader.
    sim::EngineParams     m_cfgParams[2];
    sim::DrivetrainParams m_cfgDrive[2];
    std::atomic<int>      m_cfgIndex{0};
    std::atomic<unsigned> m_configGen{0};
    std::atomic<unsigned> m_configAck{0};

    sim::Snapshot m_snap[2];
    std::atomic<int> m_snapIndex{0};

    std::array<std::atomic<float>, kScope> m_scope{};
    std::atomic<std::size_t> m_scopeHead{0};
};

} // namespace audio
