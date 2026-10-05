// Build without FFmpeg: every open() fails and the HLE produces silence.
#include "atrac_decoder.hpp"

#include <iostream>

namespace pes6 {

struct AtracFrameDecoder::Impl {};

AtracFrameDecoder::AtracFrameDecoder() = default;
AtracFrameDecoder::~AtracFrameDecoder() = default;
AtracFrameDecoder::AtracFrameDecoder(AtracFrameDecoder &&) noexcept = default;
AtracFrameDecoder &AtracFrameDecoder::operator=(AtracFrameDecoder &&) noexcept = default;

bool AtracFrameDecoder::open(bool, std::uint32_t, std::uint32_t, std::uint32_t,
                             std::span<const std::uint8_t>) {
    static bool reported = false;
    if (!reported) {
        reported = true;
        std::cerr << "[atrac] built without FFmpeg: ATRAC3/ATRAC3+ streams play as silence\n";
    }
    return false;
}
bool AtracFrameDecoder::is_open() const noexcept { return false; }
std::uint32_t AtracFrameDecoder::decode(std::span<const std::uint8_t>, std::span<std::int16_t>) {
    return 0u;
}
void AtracFrameDecoder::reset() noexcept {}
void AtracFrameDecoder::close() noexcept {}

} // namespace pes6
