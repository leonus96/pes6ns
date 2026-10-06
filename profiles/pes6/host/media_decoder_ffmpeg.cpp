// PMF movie decoding with FFmpeg (LGPL, linked, not vendored), adapted from
// the VCS profile's vcs_media_decoder.cpp.  The picture goes through
// libavformat's MPEG-PS demuxer; the soundtrack is walked out of the container
// by hand (see PmfAudioDecoder below).
#include "media_decoder.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace pes6 {
namespace {

// Both decoders pull one packet at a time and keep whatever the decoder hands
// back in a buffer, because the caller asks for byte counts that have nothing
// to do with frame boundaries -- it is draining a stream, exactly as it did
// from the pipe.
struct DecodeCommon {
    AVFormatContext *format{};
    AVCodecContext *codec{};
    AVPacket *packet{};
    AVFrame *frame{};
    int stream_index{-1};
    bool eof{};
    std::vector<std::uint8_t> pending;
    std::size_t pending_read{};

    ~DecodeCommon() { release(); }

    void release() noexcept {
        if (frame != nullptr) av_frame_free(&frame);
        if (packet != nullptr) av_packet_free(&packet);
        if (codec != nullptr) avcodec_free_context(&codec);
        if (format != nullptr) avformat_close_input(&format);
        stream_index = -1;
        eof = false;
        pending.clear();
        pending_read = 0u;
    }

    [[nodiscard]] bool open_stream(const std::filesystem::path &path, AVMediaType type) {
        release();
        if (avformat_open_input(&format, path.string().c_str(), nullptr, nullptr) < 0) return false;
        if (avformat_find_stream_info(format, nullptr) < 0) return false;
        const AVCodec *decoder = nullptr;
        stream_index = av_find_best_stream(format, type, -1, -1, &decoder, 0);
        if (stream_index < 0 || decoder == nullptr) return false;
        codec = avcodec_alloc_context3(decoder);
        if (codec == nullptr) return false;
        if (avcodec_parameters_to_context(codec, format->streams[stream_index]->codecpar) < 0)
            return false;
        if (avcodec_open2(codec, decoder, nullptr) < 0) return false;
        packet = av_packet_alloc();
        frame = av_frame_alloc();
        return packet != nullptr && frame != nullptr;
    }

    // Hands the next decoded frame to `consume`, which appends its bytes to
    // `pending`. Returns false once the file and the decoder are both drained.
    template <typename Consume>
    bool decode_one(Consume &&consume) {
        for (;;) {
            const int received = avcodec_receive_frame(codec, frame);
            if (received == 0) {
                consume(frame);
                av_frame_unref(frame);
                return true;
            }
            if (received != AVERROR(EAGAIN) && received != AVERROR_EOF) return false;
            if (received == AVERROR_EOF) return false;
            if (eof) {
                // Flush: a decoder can be holding frames after the last packet.
                if (avcodec_send_packet(codec, nullptr) < 0) return false;
                const int flushed = avcodec_receive_frame(codec, frame);
                if (flushed < 0) return false;
                consume(frame);
                av_frame_unref(frame);
                return true;
            }
            const int read = av_read_frame(format, packet);
            if (read < 0) {
                eof = true;
                continue;
            }
            if (packet->stream_index != stream_index) {
                av_packet_unref(packet);
                continue;
            }
            const int sent = avcodec_send_packet(codec, packet);
            av_packet_unref(packet);
            if (sent < 0 && sent != AVERROR(EAGAIN)) return false;
        }
    }

    // Serves `output` out of `pending`, refilling through `refill` as needed.
    template <typename Refill>
    std::size_t drain(std::span<std::uint8_t> output, Refill &&refill) {
        std::size_t written = 0u;
        while (written < output.size()) {
            if (pending_read >= pending.size()) {
                pending.clear();
                pending_read = 0u;
                if (!refill()) break;
                if (pending.empty()) break;
            }
            const std::size_t available = pending.size() - pending_read;
            const std::size_t take = std::min(available, output.size() - written);
            std::copy_n(pending.begin() + static_cast<std::ptrdiff_t>(pending_read), take,
                        output.begin() + static_cast<std::ptrdiff_t>(written));
            pending_read += take;
            written += take;
        }
        return written;
    }
};

} // namespace

bool movie_decoding_available() noexcept { return true; }

// ---------------------------------------------------------------------------

namespace {

// Every 0xBD PES payload, minus its four-byte PSP substream header.
std::vector<std::uint8_t> extract_pmf_private_stream(const std::filesystem::path &path) {
    std::vector<std::uint8_t> file;
    {
        std::FILE *handle = std::fopen(path.string().c_str(), "rb");
        if (handle == nullptr) return {};
        std::fseek(handle, 0, SEEK_END);
        const long size = std::ftell(handle);
        std::fseek(handle, 0, SEEK_SET);
        if (size > 0) {
            file.resize(static_cast<std::size_t>(size));
            if (std::fread(file.data(), 1u, file.size(), handle) != file.size()) file.clear();
        }
        std::fclose(handle);
    }
    std::vector<std::uint8_t> elementary;
    for (std::size_t i = 0u; i + 9u < file.size();) {
        if (!(file[i] == 0x00u && file[i + 1u] == 0x00u && file[i + 2u] == 0x01u &&
              file[i + 3u] == 0xBDu)) {
            ++i;
            continue;
        }
        const std::size_t packet_length =
            static_cast<std::size_t>(file[i + 4u]) * 256u + file[i + 5u];
        const std::size_t header_data_length = file[i + 8u];
        const std::size_t payload = i + 9u + header_data_length;
        // The length counts everything after the length field itself.
        if (packet_length < 3u + header_data_length) { ++i; continue; }
        const std::size_t payload_size = packet_length - 3u - header_data_length;
        constexpr std::size_t kSubstreamHeader = 4u;
        if (payload + payload_size > file.size() || payload_size <= kSubstreamHeader) {
            ++i;
            continue;
        }
        elementary.insert(elementary.end(), file.begin() + static_cast<std::ptrdiff_t>(payload + kSubstreamHeader),
                          file.begin() + static_cast<std::ptrdiff_t>(payload + payload_size));
        i = payload + payload_size;
    }
    return elementary;
}

// ATRAC3+ frames start with a 0x0FD0 sync. The frame size is not in the PMF in
// any form this code trusts, so it is measured: the distance between the first
// two syncs is the frame size, and the decoder is configured with it.
// Bytes of PSP frame header before the ATRAC3+ payload in a PMF.
constexpr std::size_t kPmfFrameHeader = 8u;

std::size_t measure_atrac3p_frame_size(std::span<const std::uint8_t> stream) {
    const auto sync_at = [&](std::size_t index) {
        return index + 1u < stream.size() && stream[index] == 0x0Fu && stream[index + 1u] == 0xD0u;
    };
    std::size_t first = stream.size();
    for (std::size_t i = 0u; i + 1u < stream.size(); ++i) {
        if (sync_at(i)) { first = i; break; }
    }
    if (first == stream.size()) return 0u;
    for (std::size_t i = first + 2u; i + 1u < stream.size(); ++i) {
        if (sync_at(i)) return i - first;
    }
    return 0u;
}

} // namespace

struct PmfAudioDecoder::State {
    AVCodecContext *codec{};
    AVPacket *packet{};
    AVFrame *frame{};
    std::vector<std::uint8_t> stream;
    std::size_t cursor{};
    std::size_t frame_size{};
    bool diag{};
    // The whole soundtrack, decoded at open. See PmfAudioDecoder::open.
    std::vector<std::uint8_t> pcm;
    std::size_t pcm_read{};

    ~State() { release(); }

    void release() noexcept {
        if (frame != nullptr) av_frame_free(&frame);
        if (packet != nullptr) av_packet_free(&packet);
        if (codec != nullptr) avcodec_free_context(&codec);
        stream.clear();
        pcm.clear();
        pcm.shrink_to_fit();
        cursor = 0u;
        pcm_read = 0u;
        frame_size = 0u;
    }
};

PmfAudioDecoder::PmfAudioDecoder() : state_(std::make_unique<State>()) {}
PmfAudioDecoder::~PmfAudioDecoder() = default;
PmfAudioDecoder::PmfAudioDecoder(PmfAudioDecoder &&) noexcept = default;
PmfAudioDecoder &PmfAudioDecoder::operator=(PmfAudioDecoder &&) noexcept = default;

bool PmfAudioDecoder::is_open() const noexcept { return state_->codec != nullptr; }
void PmfAudioDecoder::close() noexcept { state_->release(); }

bool PmfAudioDecoder::open(const std::filesystem::path &path) {
    close();
    State &state = *state_;
    state.stream = extract_pmf_private_stream(path);
    const bool diag = std::getenv("PSPRECOMP_MPEG_DIAG") != nullptr;
    state.diag = diag;
    if (diag) std::fprintf(stderr, "[pmf-audio] elementary=%zu bytes\n", state.stream.size());
    if (state.stream.empty()) return false;
    state.frame_size = measure_atrac3p_frame_size(state.stream);
    if (diag) std::fprintf(stderr, "[pmf-audio] frame_size=%zu\n", state.frame_size);
    if (state.frame_size == 0u) return false;
    // Skip whatever precedes the first sync.
    for (std::size_t i = 0u; i + 1u < state.stream.size(); ++i) {
        if (state.stream[i] == 0x0Fu && state.stream[i + 1u] == 0xD0u) { state.cursor = i; break; }
    }

    const AVCodec *decoder = avcodec_find_decoder(AV_CODEC_ID_ATRAC3P);
    if (decoder == nullptr) return false;
    state.codec = avcodec_alloc_context3(decoder);
    if (state.codec == nullptr) return false;
    // PSP video audio is 44100 Hz stereo; the decoder needs the frame size as
    // block_align because there is no container to tell it.
    // Each 752-byte block is an 8-byte PSP frame header -- the 0x0FD0 sync and
    // three more words -- followed by 744 bytes of ATRAC3+. The sync is not
    // part of the frame: the .AT3 files the radio decodes start at 0x39, not at
    // 0x0FD0, and feeding the header through made the decoder reject every
    // packet as invalid data. Measured by wrapping the extracted stream in a
    // known-good AT3 header and walking the skip until it decoded: 8 is the
    // only value that does.
    state.codec->sample_rate = 44100;
    state.codec->block_align = static_cast<int>(state.frame_size - kPmfFrameHeader);
    av_channel_layout_default(&state.codec->ch_layout, 2);
    if (avcodec_open2(state.codec, decoder, nullptr) < 0) {
        if (diag) std::fprintf(stderr, "[pmf-audio] avcodec_open2 failed\n");
        return false;
    }
    state.packet = av_packet_alloc();
    state.frame = av_frame_alloc();
    if (state.packet == nullptr || state.frame == nullptr) return false;

    // Decoded whole, here, instead of a frame at a time while the movie plays.
    // A PMF soundtrack is about 1.9 MB of PCM and takes milliseconds, and doing
    // it up front takes the work off the guest's timeline entirely -- the
    // thread that has to keep 60 vblanks a second no longer stops to decode
    // audio between them.
    state.pcm.reserve(static_cast<std::size_t>(state.stream.size() / state.frame_size) *
                      2048u * 2u * sizeof(std::int16_t));
    while (state.cursor + state.frame_size <= state.stream.size()) {
        const std::size_t payload = state.cursor + kPmfFrameHeader;
        const std::size_t payload_size = state.frame_size - kPmfFrameHeader;
        state.cursor += state.frame_size;
        av_packet_unref(state.packet);
        if (av_new_packet(state.packet, static_cast<int>(payload_size)) < 0) break;
        std::copy_n(state.stream.begin() + static_cast<std::ptrdiff_t>(payload),
                    payload_size, state.packet->data);
        const int sent = avcodec_send_packet(state.codec, state.packet);
        av_packet_unref(state.packet);
        if (sent < 0) {
            if (diag) std::fprintf(stderr, "[pmf-audio] send_packet=%d\n", sent);
            continue;
        }
        if (avcodec_receive_frame(state.codec, state.frame) < 0) continue;
        const int samples = state.frame->nb_samples;
        const auto *left = reinterpret_cast<const float *>(state.frame->data[0]);
        const auto *right = state.frame->ch_layout.nb_channels > 1
            ? reinterpret_cast<const float *>(state.frame->data[1]) : left;
        const auto to_pcm = [](float value) {
            return static_cast<std::int16_t>(std::clamp(value, -1.0f, 1.0f) * 32767.0f);
        };
        for (int i = 0; i < samples; ++i) {
            const std::int16_t pair[2]{to_pcm(left[i]), to_pcm(right[i])};
            const auto *bytes = reinterpret_cast<const std::uint8_t *>(pair);
            state.pcm.insert(state.pcm.end(), bytes, bytes + sizeof(pair));
        }
        av_frame_unref(state.frame);
    }
    if (diag) std::fprintf(stderr, "[pmf-audio] pcm=%zu bytes\n", state.pcm.size());
    state.pcm_read = 0u;
    return !state.pcm.empty();
}

std::size_t PmfAudioDecoder::read(std::span<std::uint8_t> output) {
    if (!is_open()) return 0u;
    State &state = *state_;
    // A copy out of the buffer decoded at open -- no FFmpeg call happens while
    // the movie is on screen.
    //
    // The guest asks for one access unit at a time and an access unit is one
    // ATRAC3+ frame: 2048 samples, 8192 bytes, 46.4 ms, which is also what its
    // pts advances by. Serving exactly what was asked for keeps picture and
    // sound on the same clock. The earlier version decoded here instead, and
    // could hand back a short frame; from then on audio consumed the stream
    // faster than the movie played it.
    const std::size_t available = state.pcm.size() - state.pcm_read;
    const std::size_t take = std::min(available, output.size());
    if (take == 0u) return 0u;
    std::copy_n(state.pcm.begin() + static_cast<std::ptrdiff_t>(state.pcm_read), take,
                output.begin());
    state.pcm_read += take;
    return take;
}

// ---------------------------------------------------------------------------

struct VideoStreamDecoder::State {
    DecodeCommon common;
    SwsContext *scaler{};
    int width{};
    int height{};

    ~State() {
        if (scaler != nullptr) sws_freeContext(scaler);
    }
};

VideoStreamDecoder::VideoStreamDecoder() : state_(std::make_unique<State>()) {}
VideoStreamDecoder::~VideoStreamDecoder() = default;
VideoStreamDecoder::VideoStreamDecoder(VideoStreamDecoder &&) noexcept = default;
VideoStreamDecoder &VideoStreamDecoder::operator=(VideoStreamDecoder &&) noexcept = default;

bool VideoStreamDecoder::is_open() const noexcept { return state_->common.codec != nullptr; }

void VideoStreamDecoder::close() noexcept {
    if (state_->scaler != nullptr) {
        sws_freeContext(state_->scaler);
        state_->scaler = nullptr;
    }
    state_->common.release();
}

bool VideoStreamDecoder::open(const std::filesystem::path &path) {
    close();
    return state_->common.open_stream(path, AVMEDIA_TYPE_VIDEO);
}

std::size_t VideoStreamDecoder::read(std::span<std::uint8_t> output) {
    if (!is_open()) return 0u;
    State &state = *state_;
    return state.common.drain(output, [&state]() -> bool {
        return state.common.decode_one([&state](AVFrame *frame) {
            // The scaler is built on the first frame: the stream header does
            // not always carry the real dimensions, and rebuilding it per frame
            // would be pure waste since they never change afterwards.
            if (state.scaler == nullptr || state.width != frame->width ||
                state.height != frame->height) {
                if (state.scaler != nullptr) sws_freeContext(state.scaler);
                state.width = frame->width;
                state.height = frame->height;
                state.scaler = sws_getContext(
                    frame->width, frame->height, static_cast<AVPixelFormat>(frame->format),
                    frame->width, frame->height, AV_PIX_FMT_RGBA,
                    SWS_BILINEAR, nullptr, nullptr, nullptr);
            }
            if (state.scaler == nullptr) return;
            const std::size_t bytes = static_cast<std::size_t>(frame->width) * frame->height * 4u;
            state.common.pending.resize(bytes);
            std::uint8_t *destination = state.common.pending.data();
            const int stride = frame->width * 4;
            sws_scale(state.scaler, frame->data, frame->linesize, 0, frame->height,
                      &destination, &stride);
        });
    });
}

} // namespace pes6
