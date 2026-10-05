// No native audio device: the mixer can still write a WAV capture.
#include "audio_device.hpp"

namespace pes6 {

bool audio_device_supported() { return false; }
bool audio_device_open(std::uint32_t) { return false; }
void audio_device_queue(const std::int16_t *, std::size_t) {}
void audio_device_close() {}

} // namespace pes6
