// Switch audio device: libnx audout fed from the mixer's sealed blocks by a
// thread of our own.
//
// Why not SDL's audio (audio_device_sdl.cpp), which the first console builds
// used: devkitPro's SDL2 maps every thread priority except HIGH to 0x3B
// (SDL_SYS_SetThreadPriority in libSDL2.a), so its audio thread, which asks
// for TIME_CRITICAL, runs below the emulation thread (0x2C) -- and as a libnx
// pthread it is created on the process's default core, the emulation core.
// Horizon gives it the CPU only while emulation sleeps: under load (replays,
// many players) the renderer ran dry, and SDL's queue kept growing up to
// 400 ms of latency, the "piling up" heard in the menus.
//
// Here the feeder thread has priority 0x2B (above emulation) on a worker core,
// and the FIFO in front of audout is bounded: beyond kMaxQueuedMs the oldest
// audio is dropped, so latency cannot accumulate.  After an underrun the
// device plays silence until kPrebufferMs is queued again.
//
// Rate control: the mixer seals audio on guest time, so when the emulation
// runs at 92 % (replays, heavy scenes) it produces 92 % of real time and a
// fixed-rate device ran dry every ~0.5 s (a 40 ms gap each time). The feeder
// instead resamples the FIFO at a rate steered by its smoothed fill level --
// down to kMinRate when starving, up to kMaxRate to drain excess -- so a slow
// stretch plays continuous, slightly lower-pitched audio that stays in step
// with the equally slowed picture. PES6_AUDIO_RATE_CONTROL=0 turns it off.
//
// PSPRECOMP_AUDIO_DIAG, or PSPRECOMP_FRAME_TIME_DIAG (on by default on
// Switch), logs an [audio-device] line every ~5 s.
#include "audio_device.hpp"

#include "host_platform.hpp"

#include <switch.h>

#include <malloc.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace pes6 {
namespace {

constexpr std::uint32_t kSampleRate = 48000u;
constexpr std::uint32_t kChannels = 2u;
constexpr std::uint32_t kBufferFrames = 480u;  // 10 ms per audout buffer
constexpr std::uint32_t kBuffers = 4u;
constexpr std::uint32_t kFramesPerMs = kSampleRate / 1000u;
constexpr std::uint32_t kPrebufferMs = 40u;
constexpr std::uint32_t kMaxQueuedMs = 200u;
constexpr std::size_t kFifoFrames = kMaxQueuedMs * kFramesPerMs + 4096u;
constexpr int kThreadPriority = 0x2B;
constexpr std::uint64_t kStatsBlocks = 470u;  // ~5 s of the mixer's 512-frame blocks
// Rate control: FIFO frames consumed per output frame.
constexpr std::uint32_t kTargetMs = 90u;
constexpr double kMinRate = 0.85;
// Above 1 it drains what the frame limiter's catch-up (frames run back to back
// after a late one) adds. Kept small on purpose: 1.06 reached the cap on
// nearly every 30 fps burst and the pitch audibly pumped up and down; the
// 200 ms kMaxQueuedMs absorbs the catch-up that 1.02 drains slowly.
constexpr double kMaxRate = 1.02;
constexpr double kRateGain = 0.3;      // rate change per target-level of error
constexpr double kLevelSmoothing = 0.03;  // per 10 ms buffer: ~330 ms time constant

struct DeviceState {
    std::mutex mutex;
    // Interleaved stereo ring, guarded by mutex.
    std::vector<std::int16_t> fifo;
    std::size_t head{};    // frame index of the oldest queued frame
    std::size_t queued{};  // frames queued
    bool primed{};
    // Feeder-only resampling state.
    double phase{};        // fractional FIFO position of the next output frame
    double level_ms{};     // smoothed fill level
    // Statistics since the last log line (guarded by mutex).
    double min_rate{1.0};
    double max_rate{1.0};
    std::uint64_t underruns{};
    std::uint64_t dropped_frames{};
    std::size_t min_queued{~std::size_t{0}};
    std::size_t max_queued{};
    std::uint64_t blocks{};

    std::array<AudioOutBuffer, kBuffers> buffers{};
    void *memory{};
    std::size_t buffer_bytes{};
    Thread thread{};
    std::atomic<bool> running{};
    bool open{};
    bool failed{};
};

DeviceState &device_state() {
    static DeviceState state;
    return state;
}

bool environment_flag(const char *name) {
    const char *text = std::getenv(name);
    return text != nullptr && *text != '\0' && std::string(text) != "0";
}

bool rate_control_enabled() {
    static const bool enabled = [] {
        const char *text = std::getenv("PES6_AUDIO_RATE_CONTROL");
        return text == nullptr || *text == '\0' || std::string(text) != "0";
    }();
    return enabled;
}

bool stats_enabled() {
    static const bool enabled =
        environment_flag("PSPRECOMP_AUDIO_DIAG") || environment_flag("PSPRECOMP_FRAME_TIME_DIAG");
    return enabled;
}

// Fills one audout buffer from the FIFO (resampled by the rate controller),
// silence for whatever it cannot cover. Called by the feeder thread.
void fill_buffer(DeviceState &state, AudioOutBuffer &buffer) {
    auto *out = static_cast<std::int16_t *>(buffer.buffer);
    std::size_t produced = 0u;
    {
        std::lock_guard<std::mutex> guard(state.mutex);
        if (!state.primed && state.queued >= kPrebufferMs * kFramesPerMs) {
            state.primed = true;
            state.phase = 0.0;
            state.level_ms = static_cast<double>(state.queued) / kFramesPerMs;
        }
        if (state.primed) {
            double rate = 1.0;
            if (rate_control_enabled()) {
                const double level = static_cast<double>(state.queued) / kFramesPerMs;
                state.level_ms += (level - state.level_ms) * kLevelSmoothing;
                rate = std::clamp(1.0 + kRateGain * (state.level_ms - kTargetMs) / kTargetMs,
                                  kMinRate, kMaxRate);
                state.min_rate = std::min(state.min_rate, rate);
                state.max_rate = std::max(state.max_rate, rate);
            }
            // Linear interpolation between FIFO frames; frame index + 1 must
            // be queued.
            const auto sample = [&](std::size_t index, std::size_t channel) {
                return static_cast<double>(state.fifo[((state.head + index) % kFifoFrames) * kChannels + channel]);
            };
            for (; produced < kBufferFrames; ++produced) {
                const auto index = static_cast<std::size_t>(state.phase);
                if (index + 1u >= state.queued) break;
                const double fraction = state.phase - static_cast<double>(index);
                for (std::size_t channel = 0u; channel < kChannels; ++channel) {
                    const double a = sample(index, channel);
                    const double b = sample(index + 1u, channel);
                    out[produced * kChannels + channel] = static_cast<std::int16_t>(std::lround(a + (b - a) * fraction));
                }
                state.phase += rate;
            }
            const auto consumed = std::min(static_cast<std::size_t>(state.phase), state.queued);
            state.head = (state.head + consumed) % kFifoFrames;
            state.queued -= consumed;
            state.phase -= static_cast<double>(consumed);
            if (produced < kBufferFrames) {
                // Ran dry even at the lowest rate: play silence until a
                // reserve is queued again rather than a train of fragments.
                state.primed = false;
                ++state.underruns;
            }
        }
    }
    std::memset(out + produced * kChannels, 0, (kBufferFrames - produced) * kChannels * sizeof(std::int16_t));
    buffer.data_size = kBufferFrames * kChannels * sizeof(std::int16_t);
    buffer.data_offset = 0u;
    armDCacheFlush(buffer.buffer, buffer.data_size);
}

void feeder_thread(void *) {
    DeviceState &state = device_state();
    for (AudioOutBuffer &buffer : state.buffers) {
        fill_buffer(state, buffer);
        audoutAppendAudioOutBuffer(&buffer);
    }
    while (state.running.load(std::memory_order_acquire)) {
        AudioOutBuffer *released = nullptr;
        std::uint32_t count = 0u;
        // 100 ms timeout so a stop request is noticed even if audout stalls.
        if (R_FAILED(audoutWaitPlayFinish(&released, &count, 100'000'000ull)) || released == nullptr)
            continue;
        while (released != nullptr) {
            fill_buffer(state, *released);
            audoutAppendAudioOutBuffer(released);
            released = nullptr;
            count = 0u;
            if (R_FAILED(audoutGetReleasedAudioOutBuffer(&released, &count)) || count == 0u) break;
        }
    }
}

} // namespace

bool audio_device_supported() {
    static const bool supported = [] {
        const char *text = std::getenv("PSPRECOMP_AUDIO_DEVICE");
        return text == nullptr || *text == '\0' || std::string(text) != "0";
    }();
    return supported;
}

bool audio_device_open(std::uint32_t sample_rate) {
    DeviceState &state = device_state();
    if (state.open) return true;
    if (state.failed) return false;
    state.failed = true;  // until everything below succeeds
    if (sample_rate != kSampleRate) {
        std::cerr << "[audio-host] audout needs " << kSampleRate << " Hz, mixer gives " << sample_rate << "\n";
        return false;
    }
    if (R_FAILED(audoutInitialize())) {
        std::cerr << "[audio-host] audoutInitialize failed\n";
        return false;
    }
    if (R_FAILED(audoutStartAudioOut())) {
        std::cerr << "[audio-host] audoutStartAudioOut failed\n";
        audoutExit();
        return false;
    }
    const std::size_t data_bytes = kBufferFrames * kChannels * sizeof(std::int16_t);
    state.buffer_bytes = (data_bytes + 0xFFFu) & ~std::size_t{0xFFFu};
    state.memory = memalign(0x1000u, state.buffer_bytes * kBuffers);
    if (state.memory == nullptr) {
        audoutStopAudioOut();
        audoutExit();
        return false;
    }
    std::memset(state.memory, 0, state.buffer_bytes * kBuffers);
    for (std::uint32_t index = 0u; index < kBuffers; ++index) {
        AudioOutBuffer &buffer = state.buffers[index];
        buffer = AudioOutBuffer{};
        buffer.buffer = static_cast<std::uint8_t *>(state.memory) + index * state.buffer_bytes;
        buffer.buffer_size = state.buffer_bytes;
    }
    state.fifo.assign(kFifoFrames * kChannels, 0);
    state.head = state.queued = 0u;
    state.primed = false;

    // Last worker core: the emulation core is busy, and above 0x2C this thread
    // preempts whatever runs there for the few microseconds a buffer takes.
    const std::vector<int> &cores = platform_worker_cores();
    const int core = cores.empty() ? -2 : cores.back();
    state.running.store(true, std::memory_order_release);
    if (R_FAILED(threadCreate(&state.thread, feeder_thread, nullptr, nullptr, 0x4000u, kThreadPriority, core)) ||
        R_FAILED(threadStart(&state.thread))) {
        std::cerr << "[audio-host] cannot start the audio thread\n";
        state.running.store(false, std::memory_order_release);
        threadClose(&state.thread);
        audoutStopAudioOut();
        audoutExit();
        std::free(state.memory);
        state.memory = nullptr;
        return false;
    }
    state.open = true;
    state.failed = false;
    std::cerr << "[audio-host] audout " << kSampleRate << " Hz stereo, " << kBuffers << "x"
              << kBufferFrames / kFramesPerMs << " ms buffers, prebuffer " << kPrebufferMs
              << " ms, max " << kMaxQueuedMs << " ms, rate control "
              << (rate_control_enabled() ? "on" : "off") << ", thread prio 0x" << std::hex
              << kThreadPriority << std::dec << " core " << core << "\n";
    return true;
}

void audio_device_queue(const std::int16_t *samples, std::size_t frames) {
    DeviceState &state = device_state();
    if (!state.open || frames == 0u) return;
    std::lock_guard<std::mutex> guard(state.mutex);
    // Level as the block arrives: how much the feeder had left.
    state.min_queued = std::min(state.min_queued, state.queued);
    const std::size_t limit = kMaxQueuedMs * kFramesPerMs;
    if (state.queued + frames > limit) {
        // Bounded latency: the oldest audio goes, not the newest.
        const std::size_t drop = std::min(state.queued, state.queued + frames - limit);
        state.head = (state.head + drop) % kFifoFrames;
        state.queued -= drop;
        state.dropped_frames += drop;
    }
    for (std::size_t frame = 0u; frame < frames; ++frame) {
        const std::size_t slot = ((state.head + state.queued + frame) % kFifoFrames) * kChannels;
        state.fifo[slot] = samples[frame * kChannels];
        state.fifo[slot + 1u] = samples[frame * kChannels + 1u];
    }
    state.queued += frames;

    if (!stats_enabled()) return;
    state.max_queued = std::max(state.max_queued, state.queued);
    if (++state.blocks % kStatsBlocks == 0u) {
        std::cerr << "[audio-device] queued_ms min=" << state.min_queued / kFramesPerMs
                  << " max=" << state.max_queued / kFramesPerMs << " underruns=" << state.underruns
                  << " dropped_ms=" << state.dropped_frames / kFramesPerMs
                  << " min_rate=" << static_cast<int>(state.min_rate * 100.0 + 0.5) << "%"
                  << " max_rate=" << static_cast<int>(state.max_rate * 100.0 + 0.5) << "%\n";
        state.min_rate = 1.0;
        state.max_rate = 1.0;
        state.min_queued = ~std::size_t{0};
        state.max_queued = 0u;
        state.underruns = 0u;
        state.dropped_frames = 0u;
    }
}

void audio_device_close() {
    DeviceState &state = device_state();
    if (!state.open) return;
    state.running.store(false, std::memory_order_release);
    threadWaitForExit(&state.thread);
    threadClose(&state.thread);
    audoutStopAudioOut();
    audoutExit();
    std::free(state.memory);
    state.memory = nullptr;
    state.open = false;
}

} // namespace pes6
