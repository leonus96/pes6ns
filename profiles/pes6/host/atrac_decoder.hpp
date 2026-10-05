#pragma once

#include <cstdint>
#include <memory>
#include <span>

namespace pes6 {

// Frame-by-frame ATRAC3 / ATRAC3plus decoder for the sceAtrac3plus HLE.  The
// HLE owns the encoded stream (it comes from guest buffers, PES6 keeps its
// music inside .afs archives) and hands this class one block_align-sized frame
// at a time.  atrac_decoder_ffmpeg.cpp implements it with libavcodec;
// atrac_decoder_null.cpp is the build without FFmpeg (open() fails, the HLE
// then produces silence).
class AtracFrameDecoder {
public:
    AtracFrameDecoder();
    ~AtracFrameDecoder();
    AtracFrameDecoder(AtracFrameDecoder &&) noexcept;
    AtracFrameDecoder &operator=(AtracFrameDecoder &&) noexcept;
    AtracFrameDecoder(const AtracFrameDecoder &) = delete;
    AtracFrameDecoder &operator=(const AtracFrameDecoder &) = delete;

    // `codec_extradata` is the RIFF fmt chunk past its 18-byte WAVEFORMATEX
    // head; ATRAC3 needs it, ATRAC3plus ignores it.
    [[nodiscard]] bool open(bool atrac3plus, std::uint32_t channels, std::uint32_t sample_rate,
                            std::uint32_t block_align, std::span<const std::uint8_t> codec_extradata);
    [[nodiscard]] bool is_open() const noexcept;

    // Decodes one frame into interleaved stereo s16 (a mono stream is
    // duplicated to both sides).  Returns the number of stereo samples written,
    // at most out.size() / 2; 0 when the frame could not be decoded.
    [[nodiscard]] std::uint32_t decode(std::span<const std::uint8_t> frame, std::span<std::int16_t> out);

    // Drops decoder state after a seek or loop.
    void reset() noexcept;
    void close() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace pes6
