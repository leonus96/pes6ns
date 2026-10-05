#include "param_sfo.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>

namespace pes6 {
namespace {

constexpr std::uint32_t kSfoMagic = 0x46535000u;  // "\0PSF"
constexpr std::uint32_t kSfoVersion = 0x00000101u;
constexpr std::uint16_t kFormatBytes = 0x0004u;
constexpr std::uint16_t kFormatString = 0x0204u;
constexpr std::uint16_t kFormatInt = 0x0404u;

std::uint32_t align4(std::uint32_t value) {
    return (value + 3u) & ~3u;
}

std::uint32_t read_le32(const std::vector<std::uint8_t> &bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1u]) << 8u) |
           (static_cast<std::uint32_t>(bytes[offset + 2u]) << 16u) |
           (static_cast<std::uint32_t>(bytes[offset + 3u]) << 24u);
}

std::uint16_t read_le16(const std::vector<std::uint8_t> &bytes, std::size_t offset) {
    return static_cast<std::uint16_t>(bytes[offset] | (bytes[offset + 1u] << 8u));
}

void write_le32(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint32_t value) {
    for (std::size_t index = 0; index < 4u; ++index)
        bytes[offset + index] = static_cast<std::uint8_t>(value >> (8u * index));
}

void write_le16(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value);
    bytes[offset + 1u] = static_cast<std::uint8_t>(value >> 8u);
}

} // namespace

void ParamSfo::set_string(const std::string &key, const std::string &value, std::uint32_t max_length) {
    std::vector<std::uint8_t> data(value.begin(), value.end());
    if (max_length != 0u && data.size() >= max_length) data.resize(max_length - 1u);
    data.push_back(0u);
    values_[key] = SfoValue{kFormatString, std::move(data), align4(max_length)};
}

void ParamSfo::set_int(const std::string &key, std::uint32_t value) {
    std::vector<std::uint8_t> data(4u);
    write_le32(data, 0u, value);
    values_[key] = SfoValue{kFormatInt, std::move(data), 4u};
}

void ParamSfo::set_bytes(const std::string &key, std::vector<std::uint8_t> value, std::uint32_t max_length) {
    if (value.size() > max_length) value.resize(max_length);
    values_[key] = SfoValue{kFormatBytes, std::move(value), align4(max_length)};
}

std::optional<std::string> ParamSfo::get_string(const std::string &key) const {
    const auto found = values_.find(key);
    if (found == values_.end() || found->second.format != kFormatString) return std::nullopt;
    std::string text(found->second.data.begin(), found->second.data.end());
    const std::size_t end = text.find('\0');
    if (end != std::string::npos) text.resize(end);
    return text;
}

std::optional<std::uint32_t> ParamSfo::get_int(const std::string &key) const {
    const auto found = values_.find(key);
    if (found == values_.end() || found->second.format != kFormatInt || found->second.data.size() < 4u)
        return std::nullopt;
    return read_le32(found->second.data, 0u);
}

const std::vector<std::uint8_t> *ParamSfo::get_bytes(const std::string &key) const {
    const auto found = values_.find(key);
    return found == values_.end() ? nullptr : &found->second.data;
}

std::vector<std::uint8_t> ParamSfo::serialize() const {
    const auto count = static_cast<std::uint32_t>(values_.size());
    std::uint32_t key_bytes = 0u;
    std::uint32_t data_bytes = 0u;
    for (const auto &[key, value] : values_) {
        key_bytes += static_cast<std::uint32_t>(key.size()) + 1u;
        data_bytes += std::max<std::uint32_t>(value.max_length, align4(static_cast<std::uint32_t>(value.data.size())));
    }
    const std::uint32_t key_table = 20u + 16u * count;
    const std::uint32_t data_table = align4(key_table + key_bytes);
    std::vector<std::uint8_t> bytes(data_table + data_bytes, 0u);
    write_le32(bytes, 0u, kSfoMagic);
    write_le32(bytes, 4u, kSfoVersion);
    write_le32(bytes, 8u, key_table);
    write_le32(bytes, 12u, data_table);
    write_le32(bytes, 16u, count);

    std::uint32_t key_offset = 0u;
    std::uint32_t data_offset = 0u;
    std::size_t index = 0u;
    for (const auto &[key, value] : values_) {
        const std::size_t entry = 20u + 16u * index++;
        const std::uint32_t reserved =
            std::max<std::uint32_t>(value.max_length, align4(static_cast<std::uint32_t>(value.data.size())));
        write_le16(bytes, entry, static_cast<std::uint16_t>(key_offset));
        write_le16(bytes, entry + 2u, value.format);
        write_le32(bytes, entry + 4u, static_cast<std::uint32_t>(value.data.size()));
        write_le32(bytes, entry + 8u, reserved);
        write_le32(bytes, entry + 12u, data_offset);
        std::copy(key.begin(), key.end(), bytes.begin() + key_table + key_offset);
        std::copy(value.data.begin(), value.data.end(), bytes.begin() + data_table + data_offset);
        key_offset += static_cast<std::uint32_t>(key.size()) + 1u;
        data_offset += reserved;
    }
    return bytes;
}

std::optional<ParamSfo> ParamSfo::parse(const std::vector<std::uint8_t> &bytes) {
    if (bytes.size() < 20u || read_le32(bytes, 0u) != kSfoMagic) return std::nullopt;
    const std::uint32_t key_table = read_le32(bytes, 8u);
    const std::uint32_t data_table = read_le32(bytes, 12u);
    const std::uint32_t count = read_le32(bytes, 16u);
    if (count > 1024u || 20u + 16ull * count > bytes.size() || key_table > bytes.size() || data_table > bytes.size())
        return std::nullopt;
    ParamSfo sfo;
    for (std::uint32_t index = 0u; index < count; ++index) {
        const std::size_t entry = 20u + 16u * index;
        const std::size_t key_start = key_table + read_le16(bytes, entry);
        const std::uint16_t format = read_le16(bytes, entry + 2u);
        const std::uint32_t length = read_le32(bytes, entry + 4u);
        const std::uint32_t max_length = read_le32(bytes, entry + 8u);
        const std::uint64_t data_start = static_cast<std::uint64_t>(data_table) + read_le32(bytes, entry + 12u);
        if (key_start >= bytes.size() || data_start + length > bytes.size()) return std::nullopt;
        std::string key;
        for (std::size_t at = key_start; at < bytes.size() && bytes[at] != 0u; ++at)
            key.push_back(static_cast<char>(bytes[at]));
        const auto first = bytes.begin() + static_cast<std::ptrdiff_t>(data_start);
        sfo.values_[key] = SfoValue{format, std::vector<std::uint8_t>(first, first + length), max_length};
    }
    return sfo;
}

bool ParamSfo::write_file(const std::filesystem::path &path) const {
    const auto bytes = serialize();
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) return false;
    output.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return output.good();
}

std::optional<ParamSfo> ParamSfo::read_file(const std::filesystem::path &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) return std::nullopt;
    const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    return parse(bytes);
}

} // namespace pes6
