#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>

namespace pes6 {

// PMF movie decoding for the sceMpeg HLE, adapted from the VCS profile
// (vcs_media_decoder.*).  The HLE keeps the guest's ring-buffer bookkeeping and
// identifies which .pmf on disc is playing; these classes decode that file
// directly from the host path.  media_decoder_ffmpeg.cpp implements them with
// libavformat/libavcodec/libswscale; media_decoder_null.cpp is the build
// without them (open() fails; the HLE then skips the intro by default).

// False in the build without FFmpeg: the intro is then skipped by default.
[[nodiscard]] bool movie_decoding_available() noexcept;

// The ATRAC3+ soundtrack: every private_stream_1 (0xBD) packet of the PMF,
// decoded whole at open() into interleaved stereo s16 at 44100 Hz.
class PmfAudioDecoder {
public:
    PmfAudioDecoder();
    ~PmfAudioDecoder();
    PmfAudioDecoder(const PmfAudioDecoder &) = delete;
    PmfAudioDecoder &operator=(const PmfAudioDecoder &) = delete;
    PmfAudioDecoder(PmfAudioDecoder &&) noexcept;
    PmfAudioDecoder &operator=(PmfAudioDecoder &&) noexcept;

    [[nodiscard]] bool open(const std::filesystem::path &path);
    // Returns bytes written; less than the span means the soundtrack ended.
    [[nodiscard]] std::size_t read(std::span<std::uint8_t> output);
    [[nodiscard]] bool is_open() const noexcept;
    void close() noexcept;

private:
    struct State;
    std::unique_ptr<State> state_;
};

// The AVC picture stream: RGBA8888, tightly packed, frame after frame.
class VideoStreamDecoder {
public:
    VideoStreamDecoder();
    ~VideoStreamDecoder();
    VideoStreamDecoder(const VideoStreamDecoder &) = delete;
    VideoStreamDecoder &operator=(const VideoStreamDecoder &) = delete;
    VideoStreamDecoder(VideoStreamDecoder &&) noexcept;
    VideoStreamDecoder &operator=(VideoStreamDecoder &&) noexcept;

    [[nodiscard]] bool open(const std::filesystem::path &path);
    // Returns bytes written; less than the span means the stream ended.
    [[nodiscard]] std::size_t read(std::span<std::uint8_t> output);
    [[nodiscard]] bool is_open() const noexcept;
    void close() noexcept;

private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace pes6
