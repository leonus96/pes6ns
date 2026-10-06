#pragma once

#include <filesystem>

namespace pes6 {

struct BootstrapPaths {
    std::filesystem::path psp_executable;
    std::filesystem::path game_root;
    bool discovered_from_game_dir{};
};

// argv[1] = decrypted EBOOT, argv[2] = game root (defaults to the EBOOT's
// directory when it sits directly in the root, otherwise two levels up for
// PSP_GAME/SYSDIR/<eboot>). With no arguments, the EBOOT is searched in
// <exe_dir>/game, <exe_dir>/PSP_DATA and <exe_dir> itself (first hit wins).
[[nodiscard]] BootstrapPaths resolve_bootstrap_paths(
    int argc, const char *const *argv,
    const std::filesystem::path &executable_directory);

} // namespace pes6
