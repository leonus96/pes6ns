#include "pes6_runtime_log.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

namespace pes6 {
namespace {

struct RuntimeLogState {
    std::mutex mutex;
    std::ofstream file;
    std::filesystem::path path;
    bool enabled{};
    bool to_stderr{};
};

RuntimeLogState &state() {
    static RuntimeLogState s;
    return s;
}

std::string timestamp_now() {
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const std::time_t t = clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::ostringstream out;
    out << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return out.str();
}

} // namespace

void runtime_log_initialize() {
    RuntimeLogState &s = state();
    std::lock_guard<std::mutex> guard(s.mutex);
    if (s.file.is_open()) s.file.close();
    s.path.clear();
    s.enabled = false;
    s.to_stderr = false;
    const char *value = std::getenv("PES6_LOG");
    if (value == nullptr || *value == '\0' || std::strcmp(value, "0") == 0) return;
    if (std::strcmp(value, "stderr") == 0 || std::strcmp(value, "-") == 0) {
        s.enabled = true;
        s.to_stderr = true;
        std::cerr << "PES6 runtime log started=" << timestamp_now() << '\n';
        return;
    }
    s.path = value;
    s.file.open(s.path, std::ios::out | std::ios::trunc);
    if (!s.file) {
        std::cerr << "PES6_LOG: cannot open " << s.path.string() << '\n';
        s.path.clear();
        return;
    }
    s.enabled = true;
    s.file << "PES6 runtime log\nstarted=" << timestamp_now() << "\n\n";
    s.file.flush();
}

void runtime_log_shutdown() noexcept {
    RuntimeLogState &s = state();
    std::lock_guard<std::mutex> guard(s.mutex);
    if (s.file.is_open()) {
        s.file << '\n' << '[' << timestamp_now() << "] shutdown\n";
        s.file.flush();
        s.file.close();
    }
    s.path.clear();
    s.enabled = false;
    s.to_stderr = false;
}

bool runtime_log_enabled() noexcept {
    RuntimeLogState &s = state();
    std::lock_guard<std::mutex> guard(s.mutex);
    return s.enabled && (s.to_stderr || s.file.is_open());
}

std::filesystem::path runtime_log_path() {
    RuntimeLogState &s = state();
    std::lock_guard<std::mutex> guard(s.mutex);
    return s.path;
}

void runtime_log_line(std::string_view line) {
    RuntimeLogState &s = state();
    std::lock_guard<std::mutex> guard(s.mutex);
    if (!s.enabled) return;
    if (s.to_stderr) {
        std::cerr << '[' << timestamp_now() << "] " << line << '\n';
        return;
    }
    if (!s.file.is_open()) return;
    s.file << '[' << timestamp_now() << "] " << line << '\n';
    s.file.flush();
}

void runtime_log_error(std::string_view category, std::string_view message) {
    std::ostringstream out;
    out << category << ": " << message;
    runtime_log_line(out.str());
}

} // namespace pes6
