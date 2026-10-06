#include "pes6_bootstrap_paths.hpp"

#include "psprecomp/common.hpp"

#include <array>
#include <sstream>

namespace pes6 {
namespace {

// A decrypted EBOOT inside PSP_GAME/SYSDIR belongs to a root two levels up;
// one placed directly in the root (profiles/pes6/game/EBOOT_DECRYPTED.BIN)
// belongs to its own directory.
std::filesystem::path default_root_for(const std::filesystem::path &executable) {
    const std::filesystem::path parent = executable.parent_path();
    if (parent.filename() == "SYSDIR" && parent.parent_path().filename() == "PSP_GAME")
        return parent.parent_path().parent_path();
    return parent;
}

} // namespace

BootstrapPaths resolve_bootstrap_paths(
    int argc, const char *const *argv,
    const std::filesystem::path &executable_directory) {
    if (argc >= 2 && argv != nullptr && argv[1] != nullptr && *argv[1] != '\0') {
        const std::filesystem::path psp_executable = argv[1];
        return {
            psp_executable,
            argc >= 3 && argv[2] != nullptr && *argv[2] != '\0'
                ? std::filesystem::path(argv[2])
                : default_root_for(psp_executable),
            false,
        };
    }

    // <dir>/game, <dir>/PSP_DATA, then <dir> itself (the Switch layout:
    // everything directly in sdmc:/switch/pes6).
    const std::array<std::filesystem::path, 3> roots{
        executable_directory / "game", executable_directory / "PSP_DATA", executable_directory};
    constexpr std::array<const char *, 8> candidates{
        "EBOOT_DECRYPTED.BIN",
        "EBOOT_DECRYPTED.ELF",
        "PSP_GAME/SYSDIR/EBOOT_DECRYPTED.BIN",
        "PSP_GAME/SYSDIR/EBOOT_DECRYPTED.ELF",
        "BOOT.BIN",
        "PSP_GAME/SYSDIR/BOOT.BIN",
        "EBOOT.BIN",
        "PSP_GAME/SYSDIR/EBOOT.BIN",
    };

    for (const std::filesystem::path &root : roots) {
        for (const char *relative : candidates) {
            const std::filesystem::path candidate = root / relative;
            std::error_code error;
            if (std::filesystem::is_regular_file(candidate, error))
                return {candidate, root, true};
        }
    }

#if defined(__SWITCH__)
    const std::filesystem::path &root = executable_directory;
#else
    const std::filesystem::path root = executable_directory / "game";
#endif
    std::ostringstream message;
    message << "No decrypted PES6 EBOOT was found.\n"
            << "Place the decrypted executable, the ISO and the extracted UMD contents at:\n  "
            << root.string() << "\n"
            << "Recommended EBOOT path:\n  "
            << (root / "EBOOT_DECRYPTED.BIN").string() << "\n"
            << "Command-line launch remains available:\n"
            << "  pes6 <decrypted EBOOT> [game_root]";
    throw psprecomp::Error(message.str());
}

} // namespace pes6
