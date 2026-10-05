#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace pes6 {

// Optional runtime log for the PES6 host.
//
// Controlled by the PES6_LOG environment variable:
//   unset / empty / "0"  -> disabled
//   "stderr" or "-"      -> lines go to stderr
//   anything else        -> treated as a file path (truncated at start)
//
// runtime_log_initialize() reads the variable; it is safe to call more than
// once (the previous sink is closed first).
void runtime_log_initialize();
void runtime_log_shutdown() noexcept;
[[nodiscard]] bool runtime_log_enabled() noexcept;
// Empty when disabled or when logging to stderr.
[[nodiscard]] std::filesystem::path runtime_log_path();
void runtime_log_line(std::string_view line);
void runtime_log_error(std::string_view category, std::string_view message);

} // namespace pes6
