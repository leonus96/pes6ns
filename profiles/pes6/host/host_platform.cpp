#include "host_platform.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#if defined(__SWITCH__)
#include <netinet/in.h> // struct in_addr, for nxlink.h
#include <switch.h>
#endif

namespace pes6 {

#if defined(__SWITCH__)
namespace {

constexpr const char *kDataDirectory = "sdmc:/switch/pes6";

bool g_nxlink{};
bool g_console{};
std::vector<int> g_worker_cores;

// KEY=VALUE per line; blank lines and '#' comments are skipped. Variables
// already set (none, normally) win over the file.
void load_environment_file(const std::filesystem::path &path) {
    std::ifstream input(path);
    if (!input) return;
    std::string line;
    unsigned count = 0u;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '#') continue;
        const auto equals = line.find('=', first);
        if (equals == std::string::npos || equals == first) continue;
        std::string key = line.substr(first, equals - first);
        while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
        const std::string value = line.substr(equals + 1u);
        if (setenv(key.c_str(), value.c_str(), 0) == 0) ++count;
    }
    std::cerr << "[platform] " << count << " variables from " << path.string() << "\n";
}

void console_open() {
    if (g_console) return;
    consoleInit(nullptr);
    g_console = true;
}

void console_close() {
    if (!g_console) return;
    consoleExit(nullptr);
    g_console = false;
}

// Waits for A (returns true) or + (returns false), or for the applet to exit.
bool console_wait_choice(PadState &pad, bool accept_a) {
    while (appletMainLoop()) {
        padUpdate(&pad);
        const u64 down = padGetButtonsDown(&pad);
        if (accept_a && (down & HidNpadButton_A)) return true;
        if (down & HidNpadButton_Plus) return false;
        consoleUpdate(nullptr);
    }
    return false;
}

} // namespace

bool platform_initialize() {
    // The warning console takes over stdout, so it runs before the log
    // redirection below.
    const AppletType type = appletGetAppletType();
    if (type != AppletType_Application && type != AppletType_SystemApplication) {
        // Applet mode (Album): a few hundred MiB at most. PES6 may still fit,
        // but warn instead of failing later with an opaque allocation error.
        padConfigureInput(1, HidNpadStyleSet_NpadStandard);
        PadState pad;
        padInitializeDefault(&pad);
        console_open();
        std::printf("\n  PES6 Switch\n\n"
                    "  Modo applet detectado (abierto desde el Album).\n"
                    "  Hay poca memoria: abre el Homebrew Menu en modo\n"
                    "  title takeover (manten R al abrir un juego).\n\n"
                    "  A: continuar de todos modos    +: salir\n");
        const bool proceed = console_wait_choice(pad, true);
        console_close();
        if (!proceed) return false;
    }

    // Launched with `nxlink -s`: hbloader leaves the host address behind.
    if (__nxlink_host.s_addr != 0u && R_SUCCEEDED(socketInitializeDefault()))
        g_nxlink = nxlinkStdio() >= 0;
    if (!g_nxlink) {
        const std::string log = std::string(kDataDirectory) + "/pes6.log";
        // One open file: a second fopen of the same SD file fails, which
        // silently dropped everything written to stderr. std::cerr/std::clog
        // share stdout's line-buffered FILE instead (C-level fprintf(stderr)
        // is lost; only the FFmpeg diagnostics use it). Without unitbuf, a
        // multi-part `std::cerr << a << b` line costs one SD write, not one
        // per piece.
        if (std::freopen(log.c_str(), "w", stdout) != nullptr) {
            setvbuf(stdout, nullptr, _IOLBF, 0);
            std::cerr.rdbuf(std::cout.rdbuf());
            std::clog.rdbuf(std::cout.rdbuf());
            std::cerr.unsetf(std::ios::unitbuf);
        }
    }
    std::cerr << "[platform] Switch, applet type " << static_cast<int>(type)
              << ", log=" << (g_nxlink ? "nxlink" : "pes6.log") << "\n";
    load_environment_file(std::filesystem::path(kDataDirectory) / "pes6.env");
    // Bring-up (Phase 5/6): a timing summary every 300 vblanks (~5 s) in the
    // log. Values from pes6.env win (setenv does not overwrite them).
    setenv("PSPRECOMP_FRAME_TIME_DIAG", "1", 0);
    setenv("PSPRECOMP_FRAME_TIME_INTERVAL", "300", 0);
    setenv("PSPRECOMP_GE_PHASE_DIAG", "1", 0);
    // The GE runs on the GPU (OpenGL over Mesa); the CPU rasterizer is far too
    // slow on the A57. 2x internal resolution: 960x544, upscaled to 720p.
    // PES6_RENDERER=software and PES6_RENDER_SCALE=<n> in pes6.env override.
    setenv("PES6_RENDERER", "gl", 0);
    setenv("PES6_RENDER_SCALE", "2", 0);

    // Every core the process may use except this (the emulation) thread's.
    u64 core_mask = 0u;
    if (R_FAILED(svcGetInfo(&core_mask, InfoType_CoreMask, CUR_PROCESS_HANDLE, 0))) core_mask = 0u;
    const int own_core = static_cast<int>(svcGetCurrentProcessorNumber());
    for (int core = 0; core < 4; ++core)
        if ((core_mask & (1ull << core)) != 0u && core != own_core) g_worker_cores.push_back(core);
    std::cerr << "[platform] core mask 0x" << std::hex << core_mask << std::dec << ", emulation on core "
              << own_core << ", " << g_worker_cores.size() << " worker cores\n";
    return true;
}

std::filesystem::path platform_default_data_directory(const char *) {
    // argv[0] is a device path ("sdmc:/switch/pes6/pes6.nro") that
    // std::filesystem::absolute() mangles; the data location is fixed anyway.
    return kDataDirectory;
}

void platform_show_fatal_error(std::string_view message) {
    padConfigureInput(1, HidNpadStyleSet_NpadStandard);
    PadState pad;
    padInitializeDefault(&pad);
    console_open();
    std::printf("\n  PES6 Switch: error\n\n%.*s\n\n  Datos en %s (registro: pes6.log)\n\n  +: salir\n",
                static_cast<int>(message.size()), message.data(), kDataDirectory);
    console_wait_choice(pad, false);
    console_close();
}

const std::vector<int> &platform_worker_cores() { return g_worker_cores; }

void platform_pin_current_thread(int core) {
    const Result result = svcSetThreadCoreMask(CUR_THREAD_HANDLE, core, 1u << core);
    if (R_FAILED(result))
        std::cerr << "[platform] could not pin a thread to core " << core << " (0x" << std::hex << result
                  << std::dec << ")\n";
}

void platform_shutdown() {
    std::fflush(stdout);
    std::fflush(stderr);
    if (g_nxlink) socketExit();
    g_nxlink = false;
}

#else

bool platform_initialize() { return true; }

std::filesystem::path platform_default_data_directory(const char *argv0) {
    return std::filesystem::absolute(argv0 != nullptr ? argv0 : "PES6Native").parent_path();
}

void platform_show_fatal_error(std::string_view) {}

const std::vector<int> &platform_worker_cores() {
    static const std::vector<int> none;
    return none;
}

void platform_pin_current_thread(int) {}

void platform_shutdown() {}

#endif

} // namespace pes6
