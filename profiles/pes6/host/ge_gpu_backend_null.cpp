// Null GE GPU backend for the PES6 profile.
//
// The PES6 host renders exclusively through the software GE rasterizer
// (ge_renderer.cpp). This translation unit satisfies the ge_gpu_backend.hpp
// interface with a backend that always reports "software / not active", so
// every GPU fast path in the renderer and the HLE stays dormant.
#include "ge_gpu_backend.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace pes6 {

namespace {
struct NullGpuBackendState {
    GeGpuBackendReport report{};
    std::uint32_t display_framebuffer{};
};
NullGpuBackendState &state() {
    static NullGpuBackendState s;
    return s;
}
} // namespace

bool initialize_ge_gpu_backend(std::string &error) {
    auto &s = state(); s = {};
    s.report.requested = GeGpuBackendKind::Software;
    s.report.active = GeGpuBackendKind::Software;
    s.report.message = "PES6 host uses the software GE rasterizer only";
    error.clear(); return true;
}
void shutdown_ge_gpu_backend() noexcept { state() = {}; }
bool ge_gpu_backend_active() noexcept { return false; }
bool ge_gpu_backend_transfer_ready() noexcept { return false; }
bool ge_gpu_backend_graphics_ready() noexcept { return false; }
void ge_gpu_backend_record_draw(const GeGpuDrawDescriptor &) noexcept {}
bool ge_gpu_backend_stage_vertices(const GeGpuDrawDescriptor &, std::span<const GeGpuVertex>) noexcept { return false; }
bool ge_gpu_backend_texture_needed(const GeGpuDrawDescriptor &) noexcept { return false; }
void ge_gpu_backend_prepare_texture_keys(GeGpuDrawDescriptor &) noexcept {}
bool ge_gpu_backend_texture_signature_needed(const GeGpuDrawDescriptor &) noexcept { return false; }
bool ge_gpu_backend_is_framebuffer_feedback_texture(const GeGpuDrawDescriptor &) noexcept { return false; }
void ge_gpu_backend_note_through_extent(const GeGpuDrawDescriptor &, float, float) noexcept {}
bool ge_gpu_backend_adopt_shared_texture(const GeGpuDrawDescriptor &) noexcept { return false; }
bool ge_gpu_backend_texture_available(const GeGpuDrawDescriptor &) noexcept { return false; }
bool ge_gpu_backend_upload_decoded_texture(const GeGpuDrawDescriptor &, std::uint32_t, std::uint32_t, std::span<const std::byte>) noexcept { return false; }
bool ge_gpu_backend_upload_decoded_texture_chain(const GeGpuDrawDescriptor &, std::span<const GeGpuDecodedMipLevel>) noexcept { return false; }
bool ge_gpu_backend_upload_decoded_texture_chain_packed(const GeGpuDrawDescriptor &, std::uint32_t, std::uint32_t, std::uint32_t, std::vector<std::byte>) noexcept { return false; }
bool ge_gpu_backend_copy_last_texture_rgba(std::span<std::byte>) noexcept { return false; }
void ge_gpu_backend_accumulate_color_triangles(const GeGpuDrawDescriptor &, std::span<const GeGpuVertex>) noexcept {}
void ge_gpu_backend_accumulate_hardware_triangles(const GeGpuDrawDescriptor &, const GeGpuHardwareTransform &, std::span<const GeGpuVertex>, std::span<const std::uint32_t>) noexcept {}
void ge_gpu_backend_accumulate_hardware_primitive(const GeGpuDrawDescriptor &, const GeGpuHardwareTransform &, std::uint32_t, std::span<const GeGpuVertex>, std::span<const std::uint32_t>, std::uint32_t) noexcept {}
bool ge_gpu_backend_accumulate_hardware_packed_0115(const GeGpuDrawDescriptor &, const GeGpuHardwareTransform &, std::span<const std::byte>, std::uint32_t, std::span<const std::uint32_t>) noexcept { return false; }
void ge_gpu_backend_set_native_window(void *) noexcept {}
void ge_gpu_backend_set_display_framebuffer(std::uint32_t address) noexcept { state().display_framebuffer = address & 0x001FFFF0u; }
bool ge_gpu_backend_finish_color_frame(std::uint64_t) noexcept { return false; }
bool ge_gpu_backend_copy_game_frame_rgba(std::span<std::byte>) noexcept { return false; }
bool ge_gpu_backend_presents_directly() noexcept { return false; }
std::uint32_t ge_gpu_backend_owned_framebuffer() noexcept { return 0u; }
std::uint32_t ge_gpu_backend_display_framebuffer() noexcept { return state().display_framebuffer; }
std::span<const std::byte> ge_gpu_backend_game_frame_rgba() noexcept { return {}; }
bool ge_gpu_backend_copy_offscreen_rgba(std::span<std::byte>) noexcept { return false; }
void ge_gpu_backend_mark_window_presented() noexcept {}
GeGpuBackendReport ge_gpu_backend_report() { return state().report; }
void ge_gpu_backend_attach_memory(const psprecomp::GuestMemory *) noexcept {}
bool ge_gpu_backend_owns_framebuffer(std::uint32_t) noexcept { return false; }
void ge_gpu_backend_invalidate_framebuffer(std::uint32_t, std::uint32_t) noexcept {}
bool ge_gpu_backend_read_framebuffer_rgba(std::uint32_t, std::uint32_t, std::uint32_t, std::vector<std::byte> &) noexcept { return false; }
bool ge_gpu_backend_present_framebuffer(std::uint32_t, int, int) noexcept { return false; }
void ge_gpu_backend_present_rgba(std::span<const std::byte>, std::uint32_t, std::uint32_t, int, int) noexcept {}

const char *ge_gpu_backend_name(GeGpuBackendKind kind) noexcept {
    switch (kind) {
    case GeGpuBackendKind::Software: return "software";
    case GeGpuBackendKind::DirectX12: return "directx12";
    case GeGpuBackendKind::OpenGL: return "opengl";
    }
    return "unknown";
}

} // namespace pes6
