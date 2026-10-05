// Headless display/input backend: no window, no input, frames are only counted.
// Frame dumps (PSPRECOMP_FRAME_DUMP_DIR) are handled by the sceDisplay HLE via
// framebuffer_capture, independent of this backend.
#include "display_window.hpp"

#include <atomic>

namespace pes6 {
namespace {
std::atomic<std::uint64_t> g_presented_frames{0u};
} // namespace

bool display_window_enabled() { return false; }
void display_window_start() {}
void display_window_set_status(const char *) {}
void display_window_set_aspect_lock(bool) noexcept {}
void display_window_present(const psprecomp::GuestMemory &, const FramebufferDescription &) {
    g_presented_frames.fetch_add(1u, std::memory_order_relaxed);
}
void display_window_present_rgba(std::span<const std::byte>, std::uint32_t, std::uint32_t) {
    g_presented_frames.fetch_add(1u, std::memory_order_relaxed);
}
std::uint32_t display_window_buttons() { return 0u; }
void display_window_analog(std::uint8_t &x, std::uint8_t &y) { x = 128u; y = 128u; }
HostInputState display_window_input() { return {}; }
bool display_window_close_requested() { return false; }
std::uint64_t display_window_presented_frames() noexcept {
    return g_presented_frames.load(std::memory_order_relaxed);
}
void display_window_shutdown() {}

} // namespace pes6
