#pragma once

#include <filesystem>
#include <string_view>
#include <vector>

namespace pes6 {

// Process-level glue that differs per host OS. Follows the _WIN32 /
// __SWITCH__ / generic pattern in one translation unit (host_platform.cpp).
//
// Switch (libnx homebrew):
//   - stdout/stderr go to nxlink when the .nro was sent with `nxlink -s`,
//     otherwise to <data dir>/pes6.log.
//   - <data dir>/pes6.env (KEY=VALUE lines, '#' comments) is loaded into the
//     environment, since a .nro has no shell to set PES6_* variables.
//   - In applet mode (launched from the Album) a warning asks for title
//     takeover, which has far more memory.
// Desktop: nothing to do.

// Call first in main(). Returns false when the user chose to quit.
[[nodiscard]] bool platform_initialize();

// Directory holding the game data (decrypted EBOOT, ISO, PSP_GAME/, saves) by
// default: sdmc:/switch/pes6 on Switch, the executable's directory elsewhere.
[[nodiscard]] std::filesystem::path platform_default_data_directory(const char *argv0);

// Shows a fatal error where the user can see it (on Switch, an on-screen
// console that waits for a button) after it has been printed to stderr.
void platform_show_fatal_error(std::string_view message);

// CPU cores for host worker threads (the rasterizer pool), other than the
// emulation thread's own. Empty when the OS should place them (desktop).
// On Switch std::thread::hardware_concurrency() is 0 and a new thread starts
// on the process's default core -- the emulation thread's -- where equal
// priorities do not time-slice, so workers must be pinned explicitly.
[[nodiscard]] const std::vector<int> &platform_worker_cores();

// Pins the calling thread to one core (no-op on desktop).
void platform_pin_current_thread(int core);

void platform_shutdown();

} // namespace pes6
