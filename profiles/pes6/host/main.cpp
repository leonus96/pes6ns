// PES6 (PSP, ULES-00476) native host: loads the user's decrypted EBOOT into
// guest memory, registers the AOT corpus generated from it and runs the guest
// entry point on top of the pes6 HLE layer.

#include "psprecomp/common.hpp"
#include "psprecomp/elf32.hpp"
#include "psprecomp/runtime.hpp"
#include "psprecomp/sha256.hpp"
#include "audio_output.hpp"
#include "display_window.hpp"
#include "host_platform.hpp"
#include "pes6_bootstrap_paths.hpp"
#include "pes6_hle.hpp"
#include "pes6_overlays.hpp"
#include "pes6_runtime_log.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

// Decrypted EBOOT the profile was generated from (config/pes6_ules00476.toml).
constexpr const char *kExpectedSha256 =
    "361b85a6c5576fa9ada7d57b0c9b5cccb62a4c999e33ccadaccfe30726385dd7";

std::uint64_t configured_max_dispatches() {
    constexpr std::uint64_t default_limit = 4'000'000'000ull;
    const char *text = std::getenv("PSPRECOMP_MAX_DISPATCHES");
    if (text == nullptr || *text == '\0') return default_limit;

    errno = 0;
    char *end = nullptr;
    const unsigned long long parsed = std::strtoull(text, &end, 0);
    if (errno == ERANGE || end == text || *end != '\0' || parsed == 0u)
        throw psprecomp::Error(std::string("Invalid PSPRECOMP_MAX_DISPATCHES value: ") + text);
    return static_cast<std::uint64_t>(parsed);
}

std::uint32_t user_arena_start_after(const psprecomp::Elf32Image &elf) {
    std::uint64_t image_end = 0u;
    for (std::size_t index = 0; index < elf.segments().size(); ++index) {
        const auto &segment = elf.segments()[index];
        if (segment.type != 1u) continue;  // PT_LOAD
        const std::uint64_t start =
            elf.segment_runtime_address(index, psprecomp::kDefaultPspUserLoadBase);
        image_end = std::max(image_end, start + segment.memory_size);
    }
    if (image_end == 0u || image_end > 0x0A000000ull)
        throw psprecomp::Error("Invalid PSP ELF load image extent");
    return static_cast<std::uint32_t>((image_end + 0xFFu) & ~0xFFull);
}

} // namespace

int main(int argc, char **argv) {
    if (!pes6::platform_initialize()) {
        pes6::platform_shutdown();
        return 0;
    }
    try {
        const std::filesystem::path executable_directory =
            pes6::platform_default_data_directory(argc > 0 ? argv[0] : nullptr);
        const pes6::BootstrapPaths paths =
            pes6::resolve_bootstrap_paths(argc, argv, executable_directory);
        const std::filesystem::path &executable = paths.psp_executable;
        const std::filesystem::path &root = paths.game_root;
        pes6::runtime_log_initialize();
        pes6::runtime_log_line("bootstrap executable=" + executable.string());
        pes6::runtime_log_line("bootstrap root=" + root.string());

        const std::string sha256 = psprecomp::sha256_file(executable);
        if (sha256 != kExpectedSha256 && std::getenv("PES6_ALLOW_UNKNOWN_EBOOT") == nullptr) {
            throw psprecomp::Error(
                "Unexpected EBOOT SHA-256 " + sha256 +
                " (this build was generated from ULES-00476, " + kExpectedSha256 +
                "). Set PES6_ALLOW_UNKNOWN_EBOOT=1 to run anyway.");
        }

        psprecomp::Elf32Image elf = psprecomp::Elf32Image::from_file(executable);
        psprecomp::Runtime runtime(32u * 1024u * 1024u);
        runtime.set_game_root(root);
        const auto relocations =
            elf.load_and_relocate(runtime.memory(), psprecomp::kDefaultPspUserLoadBase);
        const std::uint32_t user_arena_start = user_arena_start_after(elf);

        psprecomp::register_generated_functions(runtime);
        // Profile overrides must come after the generated corpus.
        pes6::install_profile(runtime, user_arena_start);
        pes6::install_overlay_manager(runtime);

        std::cout << "PES6Native PSP bootstrap\n"
                  << "Executable: " << executable.string() << "\n"
                  << "Game root:  " << root.string() << "\n"
                  << "SHA-256:    " << sha256 << "\n"
                  << "Entry:      " << psprecomp::hex32(elf.runtime_entry()) << "\n"
                  << "Relocs:     " << relocations.total << "\n"
                  << "Functions:  " << runtime.function_count() << "\n";

        if (runtime.function_count() == 0u) {
            std::cout << "No generated functions are linked. Run scripts/regenerate.sh first.\n";
            return 3;
        }
        const auto module =
            elf.find_module_info(runtime.memory(), psprecomp::kDefaultPspUserLoadBase);
        if (!module) throw psprecomp::Error("PSP module info not found after loading");
        runtime.cpu().set_gpr(28, module->gp);
        // install_profile() establishes the module_start thread stack.
        runtime.cpu().set_gpr(31, 0u);
        runtime.cpu().set_gpr(4, 0u);
        runtime.cpu().set_gpr(5, 0u);

        const std::uint64_t max_dispatches = configured_max_dispatches();
        std::cout << "Dispatch cap: " << max_dispatches << "\n";
        pes6::display_window_start();
        pes6::install_display_heartbeat();
        pes6::install_starvation_preemption();
        try {
            runtime.run(elf.runtime_entry(), max_dispatches);
        } catch (...) {
            // PES6_CRASH_RAM_DUMP=<file>: snapshot the 32 MiB of user RAM
            // (0x08000000..) at the failure for offline inspection.
            if (const char *dump = std::getenv("PES6_CRASH_RAM_DUMP"); dump != nullptr && *dump != '\0') {
                std::vector<std::uint8_t> ram(32u * 1024u * 1024u);
                runtime.memory().copy_out(0x08000000u, ram);
                std::ofstream(dump, std::ios::binary).write(reinterpret_cast<const char *>(ram.data()),
                                                           static_cast<std::streamsize>(ram.size()));
                std::cerr << "Guest RAM written to " << dump << "\n";
            }
            pes6::report_thread_state();
            throw;
        }

        std::cout << "Runtime stopped: " << runtime.stop_reason() << "\n";
        // PES6_EXIT_RAM_DUMP=<file>: snapshot user RAM when the run ends normally.
        if (const char *dump = std::getenv("PES6_EXIT_RAM_DUMP"); dump != nullptr && *dump != '\0') {
            std::vector<std::uint8_t> ram(32u * 1024u * 1024u);
            runtime.memory().copy_out(0x08000000u, ram);
            std::ofstream(dump, std::ios::binary).write(reinterpret_cast<const char *>(ram.data()),
                                                       static_cast<std::streamsize>(ram.size()));
        }
        psprecomp::report_counted_pcs();
        runtime.report_hle_histogram(250u);
        pes6::report_disc_read_stats();
        pes6::report_present_stats();
        pes6::report_thread_state();
        pes6::audio_output_shutdown();
        pes6::display_window_shutdown();
        pes6::platform_shutdown();
        return runtime.stop_reason().empty() ? 0 : 4;
    } catch (const std::exception &e) {
        std::cerr << "PES6Native error: " << e.what() << "\n";
        pes6::audio_output_shutdown();
        pes6::display_window_shutdown();
        pes6::platform_show_fatal_error(e.what());
        pes6::platform_shutdown();
        return 1;
    }
}
