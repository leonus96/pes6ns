#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace pes6 {

// PARAM.SFO ("\0PSF" v1.1): the key/value table that sits next to every PSP
// save.  Values keep their on-disk format and maximum length so a parsed
// file can be written back unchanged.
struct SfoValue {
    std::uint16_t format{};          // 0x0004 bytes, 0x0204 UTF-8 string, 0x0404 int32
    std::vector<std::uint8_t> data;  // used bytes (strings include their NUL)
    std::uint32_t max_length{};      // reserved bytes, multiple of 4
};

class ParamSfo {
public:
    void set_string(const std::string &key, const std::string &value, std::uint32_t max_length);
    void set_int(const std::string &key, std::uint32_t value);
    void set_bytes(const std::string &key, std::vector<std::uint8_t> value, std::uint32_t max_length);

    [[nodiscard]] std::optional<std::string> get_string(const std::string &key) const;
    [[nodiscard]] std::optional<std::uint32_t> get_int(const std::string &key) const;
    [[nodiscard]] const std::vector<std::uint8_t> *get_bytes(const std::string &key) const;

    [[nodiscard]] std::vector<std::uint8_t> serialize() const;
    // Empty optional on a malformed file.
    [[nodiscard]] static std::optional<ParamSfo> parse(const std::vector<std::uint8_t> &bytes);

    [[nodiscard]] bool write_file(const std::filesystem::path &path) const;
    [[nodiscard]] static std::optional<ParamSfo> read_file(const std::filesystem::path &path);

private:
    std::map<std::string, SfoValue> values_;  // keys sorted, as the format requires
};

} // namespace pes6
