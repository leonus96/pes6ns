// Phase 1 smoke test for the PES6 Switch port: proves the toolchain, the
// libnx framebuffer path the port will present through, pad input and
// nxlink logging all work on real hardware.
//
// Run with `nxlink -s pes6_hello.nro` to see stdout on the host.  Press + to
// exit.

#include <switch.h>

#include <cinttypes>
#include <cstdio>
#include <unistd.h>

namespace {

constexpr u32 kWidth = 1280;
constexpr u32 kHeight = 720;
constexpr int kSquare = 96;

constexpr u32 rgba(u32 r, u32 g, u32 b) {
    // PIXEL_FORMAT_RGBA_8888 is byte order R,G,B,A; little-endian u32.
    return r | (g << 8) | (b << 16) | 0xFF000000u;
}

const char* applet_type_name(AppletType type) {
    switch (type) {
    case AppletType_Application: return "Application (title takeover)";
    case AppletType_SystemApplication: return "SystemApplication";
    case AppletType_LibraryApplet: return "LibraryApplet (applet mode, low memory)";
    case AppletType_OverlayApplet: return "OverlayApplet";
    case AppletType_SystemApplet: return "SystemApplet";
    default: return "Unknown";
    }
}

bool is_full_memory_mode(AppletType type) {
    return type == AppletType_Application || type == AppletType_SystemApplication;
}

} // namespace

int main(int, char**) {
    socketInitializeDefault();
    const int nxlink_fd = nxlinkStdio();

    const AppletType applet = appletGetAppletType();
    std::printf("pes6_hello: started\n");
    std::printf("applet type: %s\n", applet_type_name(applet));
    if (!is_full_memory_mode(applet)) {
        std::printf("WARNING: applet mode; launch via title takeover (hold R on a game)\n");
    }

    padConfigureInput(1, HidNpadStyleSet_NpadStandard);
    PadState pad;
    padInitializeDefault(&pad);

    Framebuffer fb;
    framebufferCreate(&fb, nwindowGetDefault(), kWidth, kHeight, PIXEL_FORMAT_RGBA_8888, 2);
    framebufferMakeLinear(&fb);

    u32 square_color = rgba(255, 255, 255);
    int square_x = (kWidth - kSquare) / 2;
    int square_y = (kHeight - kSquare) / 2;
    u32 frame = 0;
    u32 frames_this_second = 0;
    u64 second_start = armGetSystemTick();

    while (appletMainLoop()) {
        padUpdate(&pad);
        const u64 down = padGetButtonsDown(&pad);
        if (down & HidNpadButton_Plus) {
            break;
        }
        if (down) {
            std::printf("buttons down: 0x%016" PRIx64 "\n", down);
        }
        if (down & HidNpadButton_A) square_color = rgba(255, 64, 64);
        if (down & HidNpadButton_B) square_color = rgba(255, 220, 0);
        if (down & HidNpadButton_X) square_color = rgba(64, 128, 255);
        if (down & HidNpadButton_Y) square_color = rgba(64, 220, 64);

        // Stick range is about +/-32767; up is positive Y.
        const HidAnalogStickState stick = padGetStickPos(&pad, 0);
        square_x += stick.x / 4096;
        square_y -= stick.y / 4096;
        if (square_x < 0) square_x = 0;
        if (square_y < 0) square_y = 0;
        if (square_x > static_cast<int>(kWidth) - kSquare) square_x = kWidth - kSquare;
        if (square_y > static_cast<int>(kHeight) - kSquare) square_y = kHeight - kSquare;

        u32 stride = 0;
        auto* pixels = static_cast<u32*>(framebufferBegin(&fb, &stride));
        const u32 pitch = stride / sizeof(u32);
        for (u32 y = 0; y < kHeight; ++y) {
            u32* row = pixels + y * pitch;
            for (u32 x = 0; x < kWidth; ++x) {
                const u32 v = ((x + frame) ^ y) & 0xFF;
                row[x] = rgba(v, (v * 3) & 0xFF, 255 - v);
            }
        }
        for (int y = square_y; y < square_y + kSquare; ++y) {
            u32* row = pixels + static_cast<u32>(y) * pitch;
            for (int x = square_x; x < square_x + kSquare; ++x) {
                row[x] = square_color;
            }
        }
        framebufferEnd(&fb);

        ++frame;
        ++frames_this_second;
        const u64 now = armGetSystemTick();
        if (armTicksToNs(now - second_start) >= 1000000000ull) {
            std::printf("fps: %u\n", frames_this_second);
            frames_this_second = 0;
            second_start = now;
        }
    }

    std::printf("pes6_hello: exiting\n");
    framebufferClose(&fb);
    if (nxlink_fd >= 0) {
        close(nxlink_fd);
    }
    socketExit();
    return 0;
}
