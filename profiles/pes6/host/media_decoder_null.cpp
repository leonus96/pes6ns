// Build without FFmpeg's libavformat/libswscale: every open() fails, movies
// report "no frame" and play silent.
#include "media_decoder.hpp"

#include <iostream>

namespace pes6 {
namespace {

void report_once() {
    static bool reported = false;
    if (!reported) {
        reported = true;
        std::cerr << "[media] built without FFmpeg (libavformat/libswscale): PMF movies are not decoded\n";
    }
}

} // namespace

bool movie_decoding_available() noexcept { return false; }

struct PmfAudioDecoder::State {};
PmfAudioDecoder::PmfAudioDecoder() = default;
PmfAudioDecoder::~PmfAudioDecoder() = default;
PmfAudioDecoder::PmfAudioDecoder(PmfAudioDecoder &&) noexcept = default;
PmfAudioDecoder &PmfAudioDecoder::operator=(PmfAudioDecoder &&) noexcept = default;
bool PmfAudioDecoder::open(const std::filesystem::path &) { report_once(); return false; }
std::size_t PmfAudioDecoder::read(std::span<std::uint8_t>) { return 0u; }
bool PmfAudioDecoder::is_open() const noexcept { return false; }
void PmfAudioDecoder::close() noexcept {}

struct VideoStreamDecoder::State {};
VideoStreamDecoder::VideoStreamDecoder() = default;
VideoStreamDecoder::~VideoStreamDecoder() = default;
VideoStreamDecoder::VideoStreamDecoder(VideoStreamDecoder &&) noexcept = default;
VideoStreamDecoder &VideoStreamDecoder::operator=(VideoStreamDecoder &&) noexcept = default;
bool VideoStreamDecoder::open(const std::filesystem::path &) { report_once(); return false; }
std::size_t VideoStreamDecoder::read(std::span<std::uint8_t>) { return 0u; }
bool VideoStreamDecoder::is_open() const noexcept { return false; }
void VideoStreamDecoder::close() noexcept {}

} // namespace pes6
