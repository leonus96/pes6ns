// SDL2 display/input backend for desktop and Switch (devkitPro's switch-sdl2). Everything runs on the emulation thread, which is the process
// main thread: macOS requires window and event handling there, and presenting
// synchronously from the sceDisplay HLE keeps the guest and window in step.
//
// PES6_HEADLESS=1 disables the window (automated runs).
// PES6_WINDOW_SCALE=<n> sets the initial integer scale (default 2).
// PES6_RENDERER=gl draws the GE on the GPU (ge_gpu_backend_gl.cpp) through an
// OpenGL 3.3 core context instead of an SDL_Renderer; with PES6_HEADLESS=1
// the window is created hidden so automated runs still render on the GPU.
//
// Keyboard: arrows = D-pad, X = cross, C = circle, Z = square, V = triangle,
// Q/E = L/R, Enter = START, Space = SELECT, WASD = analog stick, Esc = quit.
// Game controllers use the standard SDL layout (south button = cross).
//
// Switch: full screen (480x272 scaled to fit, bilinear; PES6_SCALE_FILTER=
// nearest for sharp pixels) and input from libnx's PadState instead of SDL,
// mapped by position like a PlayStation pad: B = cross, A = circle,
// Y = square, X = triangle, L/R = L/R, + = START, - = SELECT, left stick =
// analog. ZL/ZR are left free. Holding + and - together for a second quits.
#include "display_window.hpp"
#include "ge_gpu_backend.hpp"

#include <SDL.h>
#if defined(__SWITCH__)
#include <switch.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace pes6 {
namespace {

// PSP_CTRL_* bits.
constexpr std::uint32_t kSelect = 0x0001u, kStart = 0x0008u, kUp = 0x0010u, kRight = 0x0020u,
                        kDown = 0x0040u, kLeft = 0x0080u, kLTrigger = 0x0100u, kRTrigger = 0x0200u,
                        kTriangle = 0x1000u, kCircle = 0x2000u, kCross = 0x4000u, kSquare = 0x8000u;

struct Window {
    bool enabled{};
    bool close_requested{};
    SDL_Window *window{};
    SDL_Renderer *renderer{};
    SDL_GLContext gl_context{};
    SDL_Texture *texture{};
    int texture_width{};
    int texture_height{};
    SDL_GameController *controller{};
    std::uint64_t presented{};
    std::string status;
#if defined(__SWITCH__)
    PadState pad{};
    unsigned quit_hold{};
#endif
} g;

#if defined(__SWITCH__)
// Presents run at ~60 Hz, so this is about one second.
constexpr unsigned kQuitHoldPolls = 60u;
#endif

void open_first_controller() {
    if (g.controller != nullptr) return;
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (!SDL_IsGameController(i)) continue;
        g.controller = SDL_GameControllerOpen(i);
        if (g.controller != nullptr) {
            std::cerr << "[window] controller: " << SDL_GameControllerName(g.controller) << "\n";
            return;
        }
    }
}

void pump_events() {
    if (!g.enabled) return;
#if defined(__SWITCH__)
    padUpdate(&g.pad);
    constexpr u64 quit_combo = HidNpadButton_Plus | HidNpadButton_Minus;
    g.quit_hold = (padGetButtons(&g.pad) & quit_combo) == quit_combo ? g.quit_hold + 1u : 0u;
    if (g.quit_hold >= kQuitHoldPolls) g.close_requested = true;
#endif
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
        case SDL_QUIT: g.close_requested = true; break;
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE) g.close_requested = true;
            break;
        case SDL_CONTROLLERDEVICEADDED: open_first_controller(); break;
        case SDL_CONTROLLERDEVICEREMOVED:
            if (g.controller != nullptr &&
                event.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(g.controller))) {
                SDL_GameControllerClose(g.controller);
                g.controller = nullptr;
                open_first_controller();
            }
            break;
        default: break;
        }
    }
}

void update_title() {
    if (g.window == nullptr) return;
    std::string title = "PES6 Native";
    if (!g.status.empty()) title += " - " + g.status;
    SDL_SetWindowTitle(g.window, title.c_str());
}

void present_rgba(const void *pixels, int width, int height, int pitch) {
    if (!g.enabled) return;
    if (g.gl_context != nullptr) {
        int drawable_width = 0, drawable_height = 0;
        SDL_GL_GetDrawableSize(g.window, &drawable_width, &drawable_height);
        ge_gpu_backend_present_rgba(
            std::span<const std::byte>(static_cast<const std::byte *>(pixels),
                                       static_cast<std::size_t>(pitch) * static_cast<std::size_t>(height)),
            static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height), drawable_width, drawable_height);
        SDL_GL_SwapWindow(g.window);
        ++g.presented;
        pump_events();
        return;
    }
    if (g.texture == nullptr || g.texture_width != width || g.texture_height != height) {
        if (g.texture != nullptr) SDL_DestroyTexture(g.texture);
        // SDL_PIXELFORMAT_ABGR8888 is R,G,B,A in memory on little-endian hosts.
        g.texture = SDL_CreateTexture(g.renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING,
                                      width, height);
        g.texture_width = width;
        g.texture_height = height;
        SDL_RenderSetLogicalSize(g.renderer, width, height);
    }
    // PSPRECOMP_PRESENT_DIAG: where the present time goes, every 120 frames.
    static const bool diag = std::getenv("PSPRECOMP_PRESENT_DIAG") != nullptr;
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    SDL_UpdateTexture(g.texture, nullptr, pixels, pitch);
    SDL_SetRenderDrawColor(g.renderer, 0, 0, 0, 255);
    SDL_RenderClear(g.renderer);
    SDL_RenderCopy(g.renderer, g.texture, nullptr, nullptr);
    const auto t1 = clock::now();
    SDL_RenderPresent(g.renderer);
    const auto t2 = clock::now();
    ++g.presented;
    pump_events();
    if (diag) {
        static std::chrono::nanoseconds upload{}, present{}, events{}, present_max{};
        const auto t3 = clock::now();
        upload += t1 - t0;
        present += t2 - t1;
        events += t3 - t2;
        present_max = std::max<std::chrono::nanoseconds>(present_max, t2 - t1);
        if (g.presented % 120u == 0u) {
            const auto us = [](std::chrono::nanoseconds ns) { return ns.count() / 120000; };
            std::cerr << "[present-diag] frames=" << g.presented << " upload_us=" << us(upload)
                      << " render_present_us=" << us(present) << " events_us=" << us(events)
                      << " present_max_us=" << present_max.count() / 1000 << "\n";
            upload = present = events = present_max = {};
        }
    }
}

std::uint32_t keyboard_buttons() {
    const Uint8 *keys = SDL_GetKeyboardState(nullptr);
    std::uint32_t buttons = 0u;
    const auto map = [&](SDL_Scancode key, std::uint32_t bit) { if (keys[key]) buttons |= bit; };
    map(SDL_SCANCODE_UP, kUp);
    map(SDL_SCANCODE_DOWN, kDown);
    map(SDL_SCANCODE_LEFT, kLeft);
    map(SDL_SCANCODE_RIGHT, kRight);
    map(SDL_SCANCODE_X, kCross);
    map(SDL_SCANCODE_C, kCircle);
    map(SDL_SCANCODE_Z, kSquare);
    map(SDL_SCANCODE_V, kTriangle);
    map(SDL_SCANCODE_Q, kLTrigger);
    map(SDL_SCANCODE_E, kRTrigger);
    map(SDL_SCANCODE_RETURN, kStart);
    map(SDL_SCANCODE_SPACE, kSelect);
    return buttons;
}

std::uint32_t controller_buttons() {
    if (g.controller == nullptr) return 0u;
    std::uint32_t buttons = 0u;
    const auto map = [&](SDL_GameControllerButton button, std::uint32_t bit) {
        if (SDL_GameControllerGetButton(g.controller, button)) buttons |= bit;
    };
    map(SDL_CONTROLLER_BUTTON_DPAD_UP, kUp);
    map(SDL_CONTROLLER_BUTTON_DPAD_DOWN, kDown);
    map(SDL_CONTROLLER_BUTTON_DPAD_LEFT, kLeft);
    map(SDL_CONTROLLER_BUTTON_DPAD_RIGHT, kRight);
    map(SDL_CONTROLLER_BUTTON_A, kCross);
    map(SDL_CONTROLLER_BUTTON_B, kCircle);
    map(SDL_CONTROLLER_BUTTON_X, kSquare);
    map(SDL_CONTROLLER_BUTTON_Y, kTriangle);
    map(SDL_CONTROLLER_BUTTON_LEFTSHOULDER, kLTrigger);
    map(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, kRTrigger);
    map(SDL_CONTROLLER_BUTTON_START, kStart);
    map(SDL_CONTROLLER_BUTTON_BACK, kSelect);
    if (SDL_GameControllerGetAxis(g.controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT) > 16000) buttons |= kLTrigger;
    if (SDL_GameControllerGetAxis(g.controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) > 16000) buttons |= kRTrigger;
    return buttons;
}

std::uint8_t axis_to_psp(int value) {
    // SDL -32768..32767 -> PSP 0..255, 128 at rest, with a small dead zone.
    if (value > -6000 && value < 6000) return 128u;
    return static_cast<std::uint8_t>(std::clamp((value + 32768) / 256, 0, 255));
}

#if defined(__SWITCH__)
std::uint32_t switch_pad_buttons() {
    const u64 held = padGetButtons(&g.pad);
    std::uint32_t buttons = 0u;
    const auto map = [&](u64 mask, std::uint32_t bit) { if (held & mask) buttons |= bit; };
    map(HidNpadButton_Up, kUp);
    map(HidNpadButton_Down, kDown);
    map(HidNpadButton_Left, kLeft);
    map(HidNpadButton_Right, kRight);
    map(HidNpadButton_B, kCross);
    map(HidNpadButton_A, kCircle);
    map(HidNpadButton_Y, kSquare);
    map(HidNpadButton_X, kTriangle);
    map(HidNpadButton_L, kLTrigger);
    map(HidNpadButton_R, kRTrigger);
    map(HidNpadButton_Plus, kStart);
    map(HidNpadButton_Minus, kSelect);
    return buttons;
}
#endif

} // namespace

bool display_window_enabled() { return g.enabled; }

bool gl_renderer_requested() {
    const char *text = std::getenv("PES6_RENDERER");
    return text != nullptr && std::string(text) == "gl";
}

// Core 3.3 context + GE backend on `window`. Returns false (and leaves no
// context behind) when either fails.
bool start_gl(bool vsync) {
    g.gl_context = SDL_GL_CreateContext(g.window);
    if (g.gl_context == nullptr) {
        std::cerr << "[window] OpenGL context: " << SDL_GetError() << "\n";
        return false;
    }
    SDL_GL_MakeCurrent(g.window, g.gl_context);
    SDL_GL_SetSwapInterval(vsync ? 1 : 0);
    std::string error;
    if (!initialize_ge_gpu_backend(error)) {
        std::cerr << "[window] GL GE backend: " << error << "\n";
        SDL_GL_DeleteContext(g.gl_context);
        g.gl_context = nullptr;
        return false;
    }
    return true;
}

void display_window_start() {
    const char *headless_text = std::getenv("PES6_HEADLESS");
    const bool headless = headless_text != nullptr && *headless_text != '\0' && std::string(headless_text) != "0";
    const bool want_gl = gl_renderer_requested();
    if (headless && !want_gl) return;
#if defined(__SWITCH__)
    // Input comes from libnx's pad, not SDL's joystick layer.
    padConfigureInput(1, HidNpadStyleSet_NpadStandard);
    padInitializeDefault(&g.pad);
    constexpr Uint32 subsystems = SDL_INIT_VIDEO;
#else
    const Uint32 subsystems = headless ? SDL_INIT_VIDEO : SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER;
#endif
    if (SDL_Init(subsystems) != 0) {
        std::cerr << "[window] SDL_Init failed: " << SDL_GetError() << " (continuing headless)\n";
        return;
    }
    int scale = 2;
    if (const char *text = std::getenv("PES6_WINDOW_SCALE"); text != nullptr && *text != '\0')
        scale = std::clamp(std::atoi(text), 1, 8);
    // No vsync by default: the HLE frame limiter already paces presents at the
    // PSP's 59.94 Hz, and a present that also waits for the display (the Metal
    // renderer did, without asking: up to a full 60 Hz refresh per frame)
    // pushes every frame past its slot, so the boot screens ran at 75-87 %
    // speed and the audio starved.  PES6_VSYNC=1 restores it.
    const char *vsync_text = std::getenv("PES6_VSYNC");
    const bool vsync = vsync_text != nullptr && *vsync_text != '\0' && std::string(vsync_text) != "0";

    // At most two attempts: the GL window, then (if GL fails) a plain one.
    for (bool gl_attempt : {want_gl, false}) {
        Uint32 flags = headless ? SDL_WINDOW_HIDDEN : 0u;
        if (gl_attempt) {
            flags |= SDL_WINDOW_OPENGL;
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
            SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
            SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
        }
#if defined(__SWITCH__)
        // The window is the whole screen; the frame is letterboxed into it.
        (void)scale;
        g.window = SDL_CreateWindow("PES6 Native", 0, 0, 1280, 720, flags);
#else
        g.window = SDL_CreateWindow("PES6 Native", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 480 * scale,
                                    272 * scale, flags | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
#endif
        if (g.window == nullptr) break;
        if (!gl_attempt || start_gl(vsync)) break;
        // GL failed: fall back to the SDL_Renderer window (or to no window).
        SDL_DestroyWindow(g.window);
        g.window = nullptr;
        if (headless) break;
    }
    if (g.window == nullptr) {
        std::cerr << "[window] could not create window: " << SDL_GetError() << " (continuing headless)\n";
        display_window_shutdown();
        return;
    }
    if (headless) return;  // GPU rendering only: no presents, no input

    if (g.gl_context == nullptr) {
        SDL_SetHint(SDL_HINT_RENDER_VSYNC, vsync ? "1" : "0");
        g.renderer = SDL_CreateRenderer(g.window, -1,
                                        SDL_RENDERER_ACCELERATED | (vsync ? SDL_RENDERER_PRESENTVSYNC : 0u));
        if (g.renderer == nullptr) {
            std::cerr << "[window] could not create renderer: " << SDL_GetError() << " (continuing headless)\n";
            display_window_shutdown();
            return;
        }
        SDL_RenderSetVSync(g.renderer, vsync ? 1 : 0);
    }
#if defined(__SWITCH__)
    // 480x272 -> 1280x720 is a non-integer 2.65x: bilinear by default.
    const char *filter = std::getenv("PES6_SCALE_FILTER");
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY,
                filter != nullptr && std::string(filter) == "nearest" ? "nearest" : "linear");
#else
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
#endif
    g.enabled = true;
    open_first_controller();
    if (g.renderer != nullptr) {
        SDL_SetRenderDrawColor(g.renderer, 0, 0, 0, 255);
        SDL_RenderClear(g.renderer);
        SDL_RenderPresent(g.renderer);
    }
    update_title();
    pump_events();
}

void display_window_set_status(const char *status) {
    g.status = status != nullptr ? status : "";
    update_title();
    pump_events();
}

void display_window_set_aspect_lock(bool) noexcept {}

void display_window_present(const psprecomp::GuestMemory &memory, const FramebufferDescription &description) {
    if (!g.enabled) { ++g.presented; return; }
    if (g.gl_context != nullptr) {
        // The GE drew this framebuffer on the GPU: show that image.
        int drawable_width = 0, drawable_height = 0;
        SDL_GL_GetDrawableSize(g.window, &drawable_width, &drawable_height);
        if (ge_gpu_backend_present_framebuffer(description.address, drawable_width, drawable_height)) {
            SDL_GL_SwapWindow(g.window);
            ++g.presented;
            pump_events();
            return;
        }
    }
    // The guest waits for vblank before its first sceDisplaySetFrameBuf.
    const std::uint32_t bytes_per_pixel = description.pixel_format == 3u ? 4u : 2u;
    if (description.address == 0u || description.width == 0u || description.height == 0u ||
        !memory.contains(description.address, static_cast<std::size_t>(description.stride) *
                                                  description.height * bytes_per_pixel)) {
        pump_events();
        return;
    }
    const std::vector<std::byte> rgba = decode_framebuffer_rgba(memory, description);
    if (rgba.empty()) { pump_events(); return; }
    present_rgba(rgba.data(), static_cast<int>(description.width), static_cast<int>(description.height),
                 static_cast<int>(description.width * 4u));
}

void display_window_present_rgba(std::span<const std::byte> rgba, std::uint32_t width, std::uint32_t height) {
    if (!g.enabled) { ++g.presented; return; }
    present_rgba(rgba.data(), static_cast<int>(width), static_cast<int>(height), static_cast<int>(width * 4u));
}

std::uint32_t display_window_buttons() {
    if (!g.enabled) return 0u;
#if defined(__SWITCH__)
    return switch_pad_buttons();
#endif
    if (!(SDL_GetWindowFlags(g.window) & SDL_WINDOW_INPUT_FOCUS)) return controller_buttons();
    return keyboard_buttons() | controller_buttons();
}

void display_window_analog(std::uint8_t &x, std::uint8_t &y) {
    const HostInputState state = display_window_input();
    x = state.analog_x;
    y = state.analog_y;
}

HostInputState display_window_input() {
    HostInputState state{};
    if (!g.enabled) return state;
    state.buttons = display_window_buttons();
#if defined(__SWITCH__)
    // libnx sticks are -32767..32767 with +y up; the PSP's y grows downward.
    const HidAnalogStickState stick = padGetStickPos(&g.pad, 0);
    state.analog_x = axis_to_psp(stick.x);
    state.analog_y = axis_to_psp(-stick.y);
    return state;
#endif
    if (g.controller != nullptr) {
        state.analog_x = axis_to_psp(SDL_GameControllerGetAxis(g.controller, SDL_CONTROLLER_AXIS_LEFTX));
        state.analog_y = axis_to_psp(SDL_GameControllerGetAxis(g.controller, SDL_CONTROLLER_AXIS_LEFTY));
    }
    if (SDL_GetWindowFlags(g.window) & SDL_WINDOW_INPUT_FOCUS) {
        const Uint8 *keys = SDL_GetKeyboardState(nullptr);
        if (keys[SDL_SCANCODE_A]) state.analog_x = 0u;
        if (keys[SDL_SCANCODE_D]) state.analog_x = 255u;
        if (keys[SDL_SCANCODE_W]) state.analog_y = 0u;
        if (keys[SDL_SCANCODE_S]) state.analog_y = 255u;
    }
    return state;
}

bool display_window_close_requested() { return g.close_requested; }

std::uint64_t display_window_presented_frames() noexcept { return g.presented; }

void display_window_shutdown() {
    if (g.controller != nullptr) SDL_GameControllerClose(g.controller);
    if (g.texture != nullptr) SDL_DestroyTexture(g.texture);
    if (g.renderer != nullptr) SDL_DestroyRenderer(g.renderer);
    if (g.gl_context != nullptr) {
        shutdown_ge_gpu_backend();
        SDL_GL_DeleteContext(g.gl_context);
    }
    if (g.window != nullptr) SDL_DestroyWindow(g.window);
    if (g.window != nullptr || g.enabled) SDL_Quit();
    g = Window{};
}

} // namespace pes6
