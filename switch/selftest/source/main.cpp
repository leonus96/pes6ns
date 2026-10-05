// Runs the PSPRecomp framework regression tests (tests/test_main.cpp) on real
// Switch hardware: Allegrex decoding, guest memory, ELF/PRX relocation,
// program analysis and the runtime dispatcher, all on the A57 under newlib.
//
// The result is shown on screen and, when launched with `nxlink -s`, mirrored
// to the host terminal.  Press + to exit.

#include <switch.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

int psprecomp_tests_main();

namespace {

// The SHA-256 test writes a scratch file under temp_directory_path(), which
// libstdc++ takes from TMPDIR; "/tmp" may not exist on the SD card.
constexpr const char *kTempDir = "/switch/pes6_selftest/tmp";

void emit(int nxlink_fd, const std::string &text) {
    std::fputs(text.c_str(), stdout);
    if (nxlink_fd >= 0) {
        (void)write(nxlink_fd, text.data(), text.size());
    }
}

} // namespace

int main(int, char **) {
    consoleInit(nullptr);
    padConfigureInput(1, HidNpadStyleSet_NpadStandard);
    PadState pad;
    padInitializeDefault(&pad);

    // Mirror to the host without redirecting stdout, so the console keeps
    // showing the same text on screen.
    socketInitializeDefault();
    const int nxlink_fd = nxlinkConnectToHost(false, false);

    emit(nxlink_fd, "pes6_selftest: PSPRecomp framework tests on Switch\n\n");
    consoleUpdate(nullptr);

    std::error_code ec;
    std::filesystem::create_directories(kTempDir, ec);
    setenv("TMPDIR", kTempDir, 1);

    std::ostringstream captured;
    auto *const old_out = std::cout.rdbuf(captured.rdbuf());
    auto *const old_err = std::cerr.rdbuf(captured.rdbuf());
    const auto start = std::chrono::steady_clock::now();
    int status = 1;
    try {
        status = psprecomp_tests_main();
    } catch (const std::exception &e) {
        captured << "Uncaught exception: " << e.what() << "\n";
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    std::cout.rdbuf(old_out);
    std::cerr.rdbuf(old_err);

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    emit(nxlink_fd, captured.str());
    std::ostringstream summary;
    summary << "\n" << (status == 0 ? "\x1b[32mPASS\x1b[0m" : "\x1b[31mFAIL\x1b[0m")
            << " (exit " << status << ", " << ms << " ms)\n\nPress + to exit.\n";
    emit(nxlink_fd, summary.str());

    while (appletMainLoop()) {
        padUpdate(&pad);
        if (padGetButtonsDown(&pad) & HidNpadButton_Plus) {
            break;
        }
        consoleUpdate(nullptr);
    }

    if (nxlink_fd >= 0) {
        close(nxlink_fd);
    }
    socketExit();
    consoleExit(nullptr);
    return status;
}
