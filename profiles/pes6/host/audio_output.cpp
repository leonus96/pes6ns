// Host audio sink for the sceAudio HLE: mixes every PSP channel on the guest's
// virtual-time line and hands sealed stereo blocks (44.1 kHz; 48 kHz on
// Switch, see StreamingLinearResampler::kOutputRate) to the native
// device (audio_device.hpp) and, optionally, to a WAV capture.
//
// Mixing model (from the VCS profile's waveOut sink, minus the Windows device
// handling): each submission is resampled to the output rate and added into a ring
// at the output frame matching the virtual time at which the PSP would start
// playing it.  A region is sealed (clamped to s16, queued, zeroed) from the
// vblank path once virtual time is kMixSafetyFrames past it, so channels submitted a little apart still
// land on the same frames.  Because sealing follows virtual time, a headless
// run without the frame limiter produces exactly the same samples as a real-
// time one.
//
// Environment:
//   PSPRECOMP_AUDIO=0         no mixing at all (the HLE skips copying PCM)
//   PSPRECOMP_AUDIO=1         mix even without a device or a capture
//   PSPRECOMP_AUDIO_WAV=path  write everything that was sealed to a WAV file
//   PSPRECOMP_AUDIO_DIAG      device and resync diagnostics
//   PSPRECOMP_AUDIO_CHANNEL_MASK=0x40  mix only these PSP channels (bit = channel)
#include "audio_output.hpp"

#include "audio_device.hpp"
#include "audio_resampler.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace pes6 {
namespace {

constexpr std::uint32_t kSampleRate = StreamingLinearResampler::kOutputRate;
constexpr std::size_t kOutputChannels = 2u;
constexpr std::size_t kBlockFrames = 512u;
// Do not seal the newest ~23 ms of the guest timeline: other PSP channels can
// still submit samples for that region.
constexpr std::uint64_t kMixSafetyFrames = 1024u;
constexpr std::size_t kRingFrames = kSampleRate * 2u;
constexpr std::size_t kGuestChannels = 9u;
// A channel whose next buffer is scheduled further than this from where its
// previous one ended restarts its stream instead of smearing the gap.
constexpr std::uint64_t kChannelDiscontinuityFrames = 64u;
// Rewrite the WAV header every ~5 s so a killed run still leaves a valid file.
constexpr std::uint64_t kWavHeaderRefreshFrames = kSampleRate * 5u;

struct ChannelStream {
    StreamingLinearResampler resampler;
    std::uint64_t cursor{};
    std::uint32_t source_rate{kSampleRate};
    bool stereo{true};
    bool active{};
};

struct AudioState {
    std::mutex mutex;
    std::vector<std::int32_t> ring;
    std::vector<std::int16_t> block;
    // First frame not yet sealed.
    std::uint64_t output_frame{};
    // Guest virtual time of output frame 0.
    std::uint64_t guest_anchor_us{};
    bool timeline_anchored{};
    bool started{};
    bool device_open{};
    std::array<ChannelStream, kGuestChannels> channels{};
    std::ofstream wav;
    std::uint64_t wav_frames{};
    std::uint64_t wav_frames_at_header{};
    std::uint64_t late_frames_dropped{};
    std::uint64_t overrun_frames_dropped{};
    std::uint64_t timeline_resyncs{};
    std::uint64_t submitted_us{};
    std::chrono::steady_clock::time_point last_seal{};
    std::chrono::steady_clock::duration max_seal_gap{};
};

AudioState &audio_state() {
    static AudioState state;
    return state;
}

bool diagnostics_enabled() {
    static const bool enabled = std::getenv("PSPRECOMP_AUDIO_DIAG") != nullptr;
    return enabled;
}

const char *wav_path() {
    const char *path = std::getenv("PSPRECOMP_AUDIO_WAV");
    return path != nullptr && *path != '\0' ? path : nullptr;
}

void write_le(std::ostream &out, std::uint32_t value, int bytes) {
    for (int index = 0; index < bytes; ++index)
        out.put(static_cast<char>((value >> (8 * index)) & 0xFFu));
}

void write_wav_header(std::ostream &out, std::uint64_t frames) {
    const std::uint64_t payload64 = frames * kOutputChannels * sizeof(std::int16_t);
    const auto payload = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(payload64, 0xFFFFFFFFull - 44u));
    out.write("RIFF", 4); write_le(out, 36u + payload, 4);
    out.write("WAVEfmt ", 8); write_le(out, 16u, 4);
    write_le(out, 1u, 2); write_le(out, kOutputChannels, 2);
    write_le(out, kSampleRate, 4);
    write_le(out, kSampleRate * kOutputChannels * sizeof(std::int16_t), 4);
    write_le(out, kOutputChannels * sizeof(std::int16_t), 2);
    write_le(out, 16u, 2);
    out.write("data", 4); write_le(out, payload, 4);
}

void refresh_wav_header(AudioState &state) {
    const std::streampos end = state.wav.tellp();
    state.wav.seekp(0, std::ios::beg);
    write_wav_header(state.wav, state.wav_frames);
    state.wav.seekp(end);
    state.wav.flush();
    state.wav_frames_at_header = state.wav_frames;
}

void start_locked(AudioState &state) {
    if (state.started) return;
    state.started = true;
    state.ring.assign(kRingFrames * kOutputChannels, 0);
    state.block.assign(kBlockFrames * kOutputChannels, 0);
    if (const char *path = wav_path()) {
        state.wav.open(path, std::ios::binary | std::ios::trunc);
        if (state.wav) write_wav_header(state.wav, 0u);
        else std::cerr << "[audio-host] cannot create WAV capture " << path << "\n";
    }
    if (audio_device_supported()) state.device_open = audio_device_open(kSampleRate);
}

std::uint64_t guest_frame_for(const AudioState &state, std::uint64_t guest_time_us) {
    if (!state.timeline_anchored || guest_time_us <= state.guest_anchor_us) return 0u;
    // Rounded to the nearest frame so ceil-rounded PSP blocking durations do
    // not accumulate a frame of drift every few buffers.
    return ((guest_time_us - state.guest_anchor_us) * kSampleRate + 500000u) / 1000000u;
}

void seal_one_block(AudioState &state) {
    for (std::size_t frame = 0u; frame < kBlockFrames; ++frame) {
        const std::size_t slot =
            static_cast<std::size_t>((state.output_frame + frame) % kRingFrames) * kOutputChannels;
        for (std::size_t channel = 0u; channel < kOutputChannels; ++channel) {
            state.block[frame * kOutputChannels + channel] =
                static_cast<std::int16_t>(std::clamp(state.ring[slot + channel], -32768, 32767));
            state.ring[slot + channel] = 0;
        }
    }
    if (state.device_open) audio_device_queue(state.block.data(), kBlockFrames);
    if (state.wav.is_open()) {
        state.wav.write(reinterpret_cast<const char *>(state.block.data()),
                        static_cast<std::streamsize>(state.block.size() * sizeof(std::int16_t)));
        state.wav_frames += kBlockFrames;
        if (state.wav_frames - state.wav_frames_at_header >= kWavHeaderRefreshFrames)
            refresh_wav_header(state);
    }
    state.output_frame += kBlockFrames;
}

void advance_locked(AudioState &state, std::uint64_t guest_time_us) {
    if (!state.timeline_anchored) return;
    const std::uint64_t guest_frame = guest_frame_for(state, guest_time_us);
    const std::uint64_t sealed_frame =
        guest_frame > kMixSafetyFrames ? guest_frame - kMixSafetyFrames : 0u;
    if (sealed_frame < state.output_frame + kBlockFrames) return;
    while (sealed_frame >= state.output_frame + kBlockFrames) seal_one_block(state);
    const auto now = std::chrono::steady_clock::now();
    if (state.last_seal != std::chrono::steady_clock::time_point{})
        state.max_seal_gap = std::max(state.max_seal_gap, now - state.last_seal);
    state.last_seal = now;
}

} // namespace

bool audio_output_enabled() {
    static const bool enabled = [] {
        if (const char *text = std::getenv("PSPRECOMP_AUDIO"); text != nullptr && *text != '\0')
            return std::string(text) != "0";
        return wav_path() != nullptr || audio_device_supported();
    }();
    return enabled;
}

void audio_output_submit(std::span<const std::int16_t> pcm, std::uint32_t frames,
                         bool stereo, std::uint32_t left, std::uint32_t right,
                         std::uint32_t source_rate, std::uint32_t channel,
                         std::uint64_t guest_time_us) {
    if (!audio_output_enabled() || frames == 0u || channel >= kGuestChannels) return;
    static const std::uint32_t channel_mask = [] {
        const char *text = std::getenv("PSPRECOMP_AUDIO_CHANNEL_MASK");
        return text != nullptr && *text != '\0'
            ? static_cast<std::uint32_t>(std::strtoul(text, nullptr, 0)) : 0x1FFu;
    }();
    if ((channel_mask & (1u << channel)) == 0u) return;
    if (source_rate == 0u) source_rate = kSampleRate;
    if (pcm.size() < static_cast<std::size_t>(frames) * (stereo ? 2u : 1u)) return;

    AudioState &state = audio_state();
    std::lock_guard<std::mutex> guard(state.mutex);
    start_locked(state);
    state.submitted_us += static_cast<std::uint64_t>(frames) * 1000000u / source_rate;
    if (!state.timeline_anchored) {
        state.guest_anchor_us = guest_time_us;
        state.timeline_anchored = true;
        state.output_frame = 0u;
    }
    // No sealing here: guest_time_us is when this buffer *starts*, which for a
    // queued channel can be tens of ms ahead of virtual time.  Sealing up to it
    // would leave every other channel's next buffer behind the sealed region.
    // audio_output_advance() seals on the real virtual clock instead.

    ChannelStream &stream = state.channels[channel];
    const std::uint64_t scheduled = guest_frame_for(state, guest_time_us);
    const std::uint64_t gap = stream.cursor > scheduled ? stream.cursor - scheduled
                                                        : scheduled - stream.cursor;
    const bool format_changed = stream.active &&
        (stream.source_rate != source_rate || stream.stereo != stereo);
    const bool discontinuity = stream.active && gap > kChannelDiscontinuityFrames;
    if (!stream.active || format_changed || discontinuity) {
        if (diagnostics_enabled() && discontinuity)
            std::cerr << "[audio-host] channel " << channel << " resync old=" << stream.cursor
                      << " scheduled=" << scheduled << "\n";
        if (discontinuity) ++state.timeline_resyncs;
        stream = ChannelStream{};
        stream.active = true;
        stream.source_rate = source_rate;
        stream.stereo = stereo;
        stream.resampler.reset(source_rate, stereo);
        stream.cursor = std::max(scheduled, state.output_frame);
    }
    if (stream.cursor < state.output_frame) {
        state.late_frames_dropped += state.output_frame - stream.cursor;
        stream.cursor = state.output_frame;
        stream.resampler.reset(source_rate, stereo);
    }

    // PSP channel volumes are 0..0x8000 for unity gain.
    const auto left_gain = static_cast<std::int64_t>(left);
    const auto right_gain = static_cast<std::int64_t>(right);
    const std::uint64_t ring_limit = state.output_frame + kRingFrames - kBlockFrames;
    constexpr std::int64_t kMin = std::numeric_limits<std::int32_t>::min();
    constexpr std::int64_t kMax = std::numeric_limits<std::int32_t>::max();
    stream.resampler.process(pcm, frames, stereo, source_rate,
        [&](std::int16_t source_left, std::int16_t source_right) {
            if (stream.cursor >= ring_limit) {
                ++state.overrun_frames_dropped;
                ++stream.cursor;
                return;
            }
            const std::size_t slot =
                static_cast<std::size_t>(stream.cursor % kRingFrames) * kOutputChannels;
            const std::int64_t mixed_left = state.ring[slot] +
                ((static_cast<std::int64_t>(source_left) * left_gain) >> 15);
            const std::int64_t mixed_right = state.ring[slot + 1u] +
                ((static_cast<std::int64_t>(source_right) * right_gain) >> 15);
            state.ring[slot] = static_cast<std::int32_t>(std::clamp(mixed_left, kMin, kMax));
            state.ring[slot + 1u] = static_cast<std::int32_t>(std::clamp(mixed_right, kMin, kMax));
            ++stream.cursor;
        });
}

void audio_output_advance(std::uint64_t guest_time_us) {
    if (!audio_output_enabled()) return;
    AudioState &state = audio_state();
    std::lock_guard<std::mutex> guard(state.mutex);
    advance_locked(state, guest_time_us);
}

AudioOutputCounters audio_output_take_counters() {
    AudioState &state = audio_state();
    std::lock_guard<std::mutex> guard(state.mutex);
    const auto gap_us = std::chrono::duration_cast<std::chrono::microseconds>(state.max_seal_gap).count();
    state.max_seal_gap = {};
    return {state.submitted_us / 1000u, state.output_frame * 1000u / kSampleRate,
            static_cast<std::uint64_t>(gap_us)};
}

void audio_output_reset_channel(std::uint32_t channel) {
    if (channel >= kGuestChannels) return;
    AudioState &state = audio_state();
    std::lock_guard<std::mutex> guard(state.mutex);
    state.channels[channel] = ChannelStream{};
}

void audio_output_shutdown() {
    AudioState &state = audio_state();
    std::lock_guard<std::mutex> guard(state.mutex);
    if (!state.started) return;
    if (state.device_open) audio_device_close();
    if (state.wav.is_open()) {
        refresh_wav_header(state);
        state.wav.close();
    }
    if (diagnostics_enabled())
        std::cerr << "[audio-host] shutdown sealed_frames=" << state.output_frame
                  << " late_frames=" << state.late_frames_dropped
                  << " overrun_frames=" << state.overrun_frames_dropped
                  << " resyncs=" << state.timeline_resyncs << "\n";
    state.ring.clear();
    state.block.clear();
    state.output_frame = 0u;
    state.timeline_anchored = false;
    state.started = false;
    state.device_open = false;
    state.channels = {};
    state.wav_frames = 0u;
    state.wav_frames_at_header = 0u;
    state.late_frames_dropped = 0u;
    state.overrun_frames_dropped = 0u;
    state.timeline_resyncs = 0u;
}

} // namespace pes6
