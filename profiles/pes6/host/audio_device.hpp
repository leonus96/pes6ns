#pragma once

#include <cstddef>
#include <cstdint>

namespace pes6 {

// Native audio device behind the virtual-time mixer in audio_output.cpp.
// Implementations: audio_device_sdl.cpp (desktop, and devkitPro's switch-sdl2)
// and audio_device_null.cpp (no device).  The mixer always produces 44.1 kHz
// interleaved stereo s16 in fixed-size blocks.

// Whether a device should be used in this run (built in and not disabled, e.g.
// by PES6_HEADLESS).  Decides, together with the WAV capture, whether the HLE
// hands PCM to the mixer at all.
[[nodiscard]] bool audio_device_supported();

// Opens the device paused. Returns false (and the mixer keeps running without
// a device) when it cannot be opened.
[[nodiscard]] bool audio_device_open(std::uint32_t sample_rate);

// Queues one sealed block. The device starts playing once enough audio is
// buffered and rebuilds that reserve after an underrun.
void audio_device_queue(const std::int16_t *samples, std::size_t frames);

void audio_device_close();

} // namespace pes6
