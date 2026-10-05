// Silent audio sink: accepts every submission and discards it.
#include "audio_output.hpp"

namespace pes6 {

bool audio_output_enabled() { return false; }
void audio_output_submit(std::span<const std::int16_t>, std::uint32_t, bool,
                         std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
                         std::uint64_t) {}
void audio_output_advance(std::uint64_t) {}
void audio_output_reset_channel(std::uint32_t) {}
void audio_output_shutdown() {}

} // namespace pes6
