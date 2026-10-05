// ATRAC3 / ATRAC3plus frames decoded with libavcodec (LGPL, linked as a
// library; nothing from FFmpeg is copied into this repository).
#include "atrac_decoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/channel_layout.h>
#include <libavutil/mem.h>
}

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace pes6 {

struct AtracFrameDecoder::Impl {
    AVCodecContext *context{};
    AVPacket *packet{};
    AVFrame *frame{};

    ~Impl() {
        av_frame_free(&frame);
        av_packet_free(&packet);
        avcodec_free_context(&context);
    }
};

AtracFrameDecoder::AtracFrameDecoder() = default;
AtracFrameDecoder::~AtracFrameDecoder() = default;
AtracFrameDecoder::AtracFrameDecoder(AtracFrameDecoder &&) noexcept = default;
AtracFrameDecoder &AtracFrameDecoder::operator=(AtracFrameDecoder &&) noexcept = default;

bool AtracFrameDecoder::open(bool atrac3plus, std::uint32_t channels, std::uint32_t sample_rate,
                             std::uint32_t block_align, std::span<const std::uint8_t> codec_extradata) {
    close();
    const AVCodec *codec = avcodec_find_decoder(atrac3plus ? AV_CODEC_ID_ATRAC3P : AV_CODEC_ID_ATRAC3);
    if (codec == nullptr) {
        std::cerr << "[atrac] this FFmpeg has no " << (atrac3plus ? "atrac3plus" : "atrac3")
                  << " decoder\n";
        return false;
    }
    auto impl = std::make_unique<Impl>();
    impl->context = avcodec_alloc_context3(codec);
    impl->packet = av_packet_alloc();
    impl->frame = av_frame_alloc();
    if (impl->context == nullptr || impl->packet == nullptr || impl->frame == nullptr) return false;
    impl->context->sample_rate = static_cast<int>(sample_rate);
    impl->context->block_align = static_cast<int>(block_align);
    av_channel_layout_default(&impl->context->ch_layout, static_cast<int>(channels));
    if (!atrac3plus && !codec_extradata.empty()) {
        impl->context->extradata = static_cast<std::uint8_t *>(
            av_mallocz(codec_extradata.size() + AV_INPUT_BUFFER_PADDING_SIZE));
        if (impl->context->extradata == nullptr) return false;
        std::memcpy(impl->context->extradata, codec_extradata.data(), codec_extradata.size());
        impl->context->extradata_size = static_cast<int>(codec_extradata.size());
    }
    if (const int result = avcodec_open2(impl->context, codec, nullptr); result < 0) {
        std::cerr << "[atrac] avcodec_open2 failed (" << result << ")\n";
        return false;
    }
    impl_ = std::move(impl);
    return true;
}

bool AtracFrameDecoder::is_open() const noexcept { return impl_ != nullptr; }

std::uint32_t AtracFrameDecoder::decode(std::span<const std::uint8_t> encoded,
                                        std::span<std::int16_t> out) {
    if (impl_ == nullptr || encoded.empty()) return 0u;
    AVPacket *packet = impl_->packet;
    if (av_new_packet(packet, static_cast<int>(encoded.size())) < 0) return 0u;
    std::memcpy(packet->data, encoded.data(), encoded.size());
    const int sent = avcodec_send_packet(impl_->context, packet);
    av_packet_unref(packet);
    if (sent < 0) return 0u;

    std::uint32_t written = 0u;
    const std::size_t capacity = out.size() / 2u;
    AVFrame *frame = impl_->frame;
    while (avcodec_receive_frame(impl_->context, frame) == 0) {
        const int channels = frame->ch_layout.nb_channels;
        const auto format = static_cast<AVSampleFormat>(frame->format);
        const auto sample = [&](int channel, int index) -> float {
            const int source = std::min(channel, channels - 1);
            switch (format) {
            case AV_SAMPLE_FMT_FLTP:
                return reinterpret_cast<const float *>(frame->extended_data[source])[index];
            case AV_SAMPLE_FMT_FLT:
                return reinterpret_cast<const float *>(frame->extended_data[0])[index * channels + source];
            case AV_SAMPLE_FMT_S16P:
                return reinterpret_cast<const std::int16_t *>(frame->extended_data[source])[index] / 32768.0f;
            case AV_SAMPLE_FMT_S16:
                return reinterpret_cast<const std::int16_t *>(frame->extended_data[0])[index * channels + source] /
                    32768.0f;
            default:
                return 0.0f;
            }
        };
        for (int index = 0; index < frame->nb_samples && written < capacity; ++index, ++written) {
            for (int channel = 0; channel < 2; ++channel) {
                const float value = std::clamp(sample(channel, index) * 32768.0f, -32768.0f, 32767.0f);
                out[written * 2u + static_cast<std::size_t>(channel)] =
                    static_cast<std::int16_t>(std::lrint(value));
            }
        }
        av_frame_unref(frame);
    }
    return written;
}

void AtracFrameDecoder::reset() noexcept {
    if (impl_ != nullptr) avcodec_flush_buffers(impl_->context);
}

void AtracFrameDecoder::close() noexcept { impl_.reset(); }

} // namespace pes6
