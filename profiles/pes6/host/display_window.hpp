#pragma once

#include "framebuffer_capture.hpp"
#include "psprecomp/guest_memory.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace pes6 {

// Host presentation + input backend for the PES6 profile.
//
// Exactly one implementation is linked into the host:
//   - display_window_headless.cpp: no window, no input (bring-up / CI).
//   - (planned) an SDL2 backend for desktop and a libnx backend for Switch.
//
// Threading contract: every function is called from the emulation thread
// (the thread inside psprecomp::Runtime::run). A backend that owns a separate
// UI thread must make these calls cheap and thread-safe on its side.
//
// Input contract: buttons use the PSP sceCtrl bit layout (PSP_CTRL_SELECT =
// 0x1, START = 0x8, UP = 0x10, ..., CROSS = 0x4000, SQUARE = 0x8000). The
// analog stick uses the sceCtrl range 0..255 with 128 = centred.

// True when a real window exists (or will exist after display_window_start).
[[nodiscard]] bool display_window_enabled();

// Opens the window before the guest produces its first framebuffer so the boot
// is visible from the start. No-op when the backend is headless.
void display_window_start();

// Replaces the status text shown in the title bar / over an empty frame.
void display_window_set_status(const char *status);

// Publishes one PSP display frame (called from sceDisplay vblank handling).
// The backend decodes the guest framebuffer itself, typically through
// decode_framebuffer_rgb/rgba from framebuffer_capture.hpp.
void display_window_present(const psprecomp::GuestMemory &memory,
                            const FramebufferDescription &description);

// Publishes a tightly packed RGBA8 host frame of the given dimensions.
void display_window_present_rgba(std::span<const std::byte> rgba,
                                 std::uint32_t width,
                                 std::uint32_t height);

// Pins the next frames to the PSP source aspect (black bars) regardless of the
// window shape. Intended for full-motion video.
void display_window_set_aspect_lock(bool locked) noexcept;

// Live PSP button mask sampled from the host, or 0 when there is no input.
[[nodiscard]] std::uint32_t display_window_buttons();

// Analog stick sample (128,128 when centred / no input).
void display_window_analog(std::uint8_t &x, std::uint8_t &y);

// One poll of the host input, already reduced to what the PSP pad expresses.
struct HostInputState {
    std::uint32_t buttons{};
    std::uint8_t analog_x{128u};
    std::uint8_t analog_y{128u};
};
[[nodiscard]] HostInputState display_window_input();

// True once the user asked to quit (closed the window, pressed Escape, HOME...).
[[nodiscard]] bool display_window_close_requested();

// Number of frames handed to display_window_present / _present_rgba so far.
// Useful for headless runs and diagnostics.
[[nodiscard]] std::uint64_t display_window_presented_frames() noexcept;

void display_window_shutdown();

} // namespace pes6
