// SDL2 audio device: the mixer's sealed blocks go to SDL's audio queue.
//
// PES6_HEADLESS=1 (automated runs) or PSPRECOMP_AUDIO_DEVICE=0 disable it.
// PSPRECOMP_AUDIO_DEVICE_SILENT=1 queues silence instead (real device timing,
// nothing audible); with PSPRECOMP_AUDIO_DIAG the queue depth is logged ~1/s.
// Blocks are sealed on guest virtual time, so the queue only stays short while
// the frame limiter keeps the guest near real time; beyond kMaxQueuedMs a
// block is dropped instead of letting latency grow without bound.
#include "audio_device.hpp"

#include <SDL.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace pes6 {
namespace {

// Start (and restart after an underrun) with this much audio queued.
constexpr std::uint32_t kStartPrebufferMs = 60u;
constexpr std::uint32_t kRecoveryPrebufferMs = 120u;
constexpr std::uint32_t kMaxQueuedMs = 400u;

struct DeviceState {
    SDL_AudioDeviceID device{};
    std::uint32_t bytes_per_ms{};
    std::uint32_t prebuffer_ms{kStartPrebufferMs};
    bool playing{};
    bool failed{};
    std::uint64_t underruns{};
    std::uint64_t dropped_blocks{};
    std::uint64_t queued_blocks{};
    std::uint32_t min_queued_ms{~0u};
    std::uint32_t max_queued_ms{};
};

DeviceState &device_state() {
    static DeviceState state;
    return state;
}

bool environment_flag(const char *name) {
    const char *text = std::getenv(name);
    return text != nullptr && *text != '\0' && std::string(text) != "0";
}

bool diagnostics_enabled() {
    static const bool enabled = std::getenv("PSPRECOMP_AUDIO_DIAG") != nullptr;
    return enabled;
}

} // namespace

bool audio_device_supported() {
    static const bool supported = [] {
        if (environment_flag("PES6_HEADLESS")) return false;
        const char *text = std::getenv("PSPRECOMP_AUDIO_DEVICE");
        return text == nullptr || *text == '\0' || std::string(text) != "0";
    }();
    return supported;
}

bool audio_device_open(std::uint32_t sample_rate) {
    DeviceState &state = device_state();
    if (state.device != 0u) return true;
    if (state.failed) return false;
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        std::cerr << "[audio-host] SDL audio init failed: " << SDL_GetError() << "\n";
        state.failed = true;
        return false;
    }
    SDL_AudioSpec wanted{};
    wanted.freq = static_cast<int>(sample_rate);
    wanted.format = AUDIO_S16SYS;
    wanted.channels = 2;
    wanted.samples = 512;
    SDL_AudioSpec obtained{};
    // No allowed changes: SDL converts if the hardware wants something else.
    state.device = SDL_OpenAudioDevice(nullptr, 0, &wanted, &obtained, 0);
    if (state.device == 0u) {
        std::cerr << "[audio-host] SDL_OpenAudioDevice failed: " << SDL_GetError() << "\n";
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        state.failed = true;
        return false;
    }
    state.bytes_per_ms = sample_rate * 2u * sizeof(std::int16_t) / 1000u;
    state.prebuffer_ms = kStartPrebufferMs;
    state.playing = false;
    if (diagnostics_enabled())
        std::cerr << "[audio-host] SDL device " << sample_rate << " Hz stereo, driver "
                  << SDL_GetCurrentAudioDriver() << "\n";
    return true;
}

void audio_device_queue(const std::int16_t *samples, std::size_t frames) {
    DeviceState &state = device_state();
    if (state.device == 0u || frames == 0u) return;
    const std::uint32_t queued = SDL_GetQueuedAudioSize(state.device);
    if (diagnostics_enabled()) {
        const std::uint32_t queued_ms = queued / state.bytes_per_ms;
        state.min_queued_ms = std::min(state.min_queued_ms, queued_ms);
        state.max_queued_ms = std::max(state.max_queued_ms, queued_ms);
        if (++state.queued_blocks % 86u == 0u) {
            std::cerr << "[audio-device] queued_ms=" << queued_ms << " min=" << state.min_queued_ms
                      << " max=" << state.max_queued_ms << " underruns=" << state.underruns
                      << " dropped=" << state.dropped_blocks << " ticks_ms=" << SDL_GetTicks() << "\n";
            state.min_queued_ms = ~0u;
            state.max_queued_ms = 0u;
        }
    }
    if (state.playing && queued == 0u) {
        // Drained: playing block by block from here would leave a train of
        // gaps, so pause and rebuild a deeper reserve first.
        SDL_PauseAudioDevice(state.device, 1);
        state.playing = false;
        state.prebuffer_ms = kRecoveryPrebufferMs;
        ++state.underruns;
        if (diagnostics_enabled())
            std::cerr << "[audio-host] underrun #" << state.underruns << "\n";
    }
    if (queued >= kMaxQueuedMs * state.bytes_per_ms) {
        ++state.dropped_blocks;
        if (diagnostics_enabled() && (state.dropped_blocks & (state.dropped_blocks - 1u)) == 0u)
            std::cerr << "[audio-host] queue full (" << queued / state.bytes_per_ms
                      << " ms), dropped block #" << state.dropped_blocks << "\n";
        return;
    }
    const auto bytes = static_cast<std::uint32_t>(frames * 2u * sizeof(std::int16_t));
    static const bool silent = environment_flag("PSPRECOMP_AUDIO_DEVICE_SILENT");
    if (silent) {
        static std::vector<std::int16_t> zeros;
        zeros.assign(frames * 2u, 0);
        samples = zeros.data();
    }
    if (SDL_QueueAudio(state.device, samples, bytes) != 0) return;
    if (!state.playing && queued + bytes >= state.prebuffer_ms * state.bytes_per_ms) {
        SDL_PauseAudioDevice(state.device, 0);
        state.playing = true;
    }
}

void audio_device_close() {
    DeviceState &state = device_state();
    if (state.device == 0u) return;
    SDL_CloseAudioDevice(state.device);
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    if (diagnostics_enabled())
        std::cerr << "[audio-host] SDL device closed underruns=" << state.underruns
                  << " dropped_blocks=" << state.dropped_blocks << "\n";
    state = DeviceState{};
}

} // namespace pes6
