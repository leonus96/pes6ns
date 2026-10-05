// SDL2 display/input backend for desktop (and later Switch, via devkitPro's
// switch-sdl2). Everything runs on the emulation thread, which is the process
// main thread: macOS requires window and event handling there, and presenting
// synchronously from the sceDisplay HLE keeps the guest and window in step.
//
// PES6_HEADLESS=1 disables the window (automated runs).
// PES6_WINDOW_SCALE=<n> sets the initial integer scale (default 2).
//
// Keyboard: arrows = D-pad, X = cross, C = circle, Z = square, V = triangle,
// Q/E = L/R, Enter = START, Space = SELECT, WASD = analog stick, Esc = quit.
// Game controllers use the standard SDL layout (south button = cross).
#include "display_window.hpp"

#include <SDL.h>

#include <algorithm>
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
    SDL_Texture *texture{};
    int texture_width{};
    int texture_height{};
    SDL_GameController *controller{};
    std::uint64_t presented{};
    std::string status;
} g;

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
    if (g.texture == nullptr || g.texture_width != width || g.texture_height != height) {
        if (g.texture != nullptr) SDL_DestroyTexture(g.texture);
        // SDL_PIXELFORMAT_ABGR8888 is R,G,B,A in memory on little-endian hosts.
        g.texture = SDL_CreateTexture(g.renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING,
                                      width, height);
        g.texture_width = width;
        g.texture_height = height;
        SDL_RenderSetLogicalSize(g.renderer, width, height);
    }
    SDL_UpdateTexture(g.texture, nullptr, pixels, pitch);
    SDL_SetRenderDrawColor(g.renderer, 0, 0, 0, 255);
    SDL_RenderClear(g.renderer);
    SDL_RenderCopy(g.renderer, g.texture, nullptr, nullptr);
    SDL_RenderPresent(g.renderer);
    ++g.presented;
    pump_events();
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

} // namespace

bool display_window_enabled() { return g.enabled; }

void display_window_start() {
    if (const char *headless = std::getenv("PES6_HEADLESS"); headless != nullptr && *headless != '\0' &&
                                                              std::string(headless) != "0")
        return;
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "[window] SDL_Init failed: " << SDL_GetError() << " (continuing headless)\n";
        return;
    }
    int scale = 2;
    if (const char *text = std::getenv("PES6_WINDOW_SCALE"); text != nullptr && *text != '\0')
        scale = std::clamp(std::atoi(text), 1, 8);
    g.window = SDL_CreateWindow("PES6 Native", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                480 * scale, 272 * scale, SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (g.window != nullptr) g.renderer = SDL_CreateRenderer(g.window, -1, SDL_RENDERER_ACCELERATED);
    if (g.window == nullptr || g.renderer == nullptr) {
        std::cerr << "[window] could not create window/renderer: " << SDL_GetError() << " (continuing headless)\n";
        display_window_shutdown();
        return;
    }
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
    g.enabled = true;
    open_first_controller();
    SDL_SetRenderDrawColor(g.renderer, 0, 0, 0, 255);
    SDL_RenderClear(g.renderer);
    SDL_RenderPresent(g.renderer);
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
    if (g.window != nullptr) SDL_DestroyWindow(g.window);
    if (g.window != nullptr || g.enabled) SDL_Quit();
    g = Window{};
}

} // namespace pes6
