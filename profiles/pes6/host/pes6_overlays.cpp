#include "pes6_overlays.hpp"

#include "pes6_runtime_log.hpp"
#include "psprecomp/common.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace pes6 {
namespace {

constexpr std::uint32_t kHeaderSize = 0x40u;

// Every guest range an overlay can occupy, keyed by load address: from the
// load address to the furthest image + bss end of the overlays sharing it.
std::map<std::uint32_t, std::uint32_t> overlay_region_ends;
// Load address -> overlay currently registered there (nullptr if none).
std::map<std::uint32_t, const OverlayDescriptor *> active_overlays;

std::uint64_t fnv1a64(const std::uint8_t *data, std::size_t size) {
    std::uint64_t value = 0xCBF29CE484222325ull;
    for (std::size_t i = 0; i < size; ++i) value = (value ^ data[i]) * 0x100000001B3ull;
    return value;
}

bool resident(psprecomp::Runtime &runtime, const OverlayDescriptor &overlay) {
    const std::uint8_t *image = runtime.memory().raw_pointer(overlay.load_address, overlay.image_size);
    if (image == nullptr || std::memcmp(image, "MWo3", 4u) != 0) return false;
    if (std::strncmp(reinterpret_cast<const char *>(image + 0x20u), overlay.name, 32u) != 0) return false;
    return fnv1a64(image, overlay.image_size) == overlay.image_hash;
}

void drop_region(psprecomp::Runtime &runtime, std::uint32_t load_address) {
    const auto active = active_overlays.find(load_address);
    if (active == active_overlays.end() || active->second == nullptr) return;
    runtime.unregister_code_range(load_address, overlay_region_ends.at(load_address));
    runtime_log_line(std::string("[overlay] unload ") + active->second->name);
    active->second = nullptr;
}

bool resolve_overlay_function(psprecomp::Runtime &runtime, std::uint32_t pc) {
    const auto start = std::chrono::steady_clock::now();
    const auto elapsed_us = [&start] {
        return std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start).count());
    };
    for (std::size_t i = 0; i < kOverlayCount; ++i) {
        const OverlayDescriptor &overlay = kOverlays[i];
        if (pc < overlay.load_address + kHeaderSize || pc >= overlay.load_address + overlay.image_size) continue;
        if (!resident(runtime, overlay)) continue;
        drop_region(runtime, overlay.load_address);
        // Clear any leftovers of a previous occupant before registering.
        runtime.unregister_code_range(overlay.load_address, overlay_region_ends.at(overlay.load_address));
        overlay.register_functions(runtime);
        active_overlays[overlay.load_address] = &overlay;
        runtime_log_line(std::string("[overlay] load ") + overlay.name + " at " +
                         psprecomp::hex32(overlay.load_address) + " (entered at " + psprecomp::hex32(pc) +
                         ", " + elapsed_us() + " us)");
        return true;
    }
    // Report what is actually in memory so a missing or stale translation is
    // obvious: either the ISO differs from the generated corpus or the guest
    // jumped somewhere unexpected.
    for (const auto &[load, end] : overlay_region_ends) {
        if (pc < load || pc >= end) continue;
        const std::uint8_t *header = runtime.memory().raw_pointer(load, kHeaderSize);
        std::string name = "(no MWo3 header)";
        if (header != nullptr && std::memcmp(header, "MWo3", 4u) == 0)
            name.assign(reinterpret_cast<const char *>(header + 0x20u), strnlen(reinterpret_cast<const char *>(header + 0x20u), 32u));
        std::cerr << "[overlay] no matching translation for pc " << psprecomp::hex32(pc)
                  << " in region " << psprecomp::hex32(load) << ", resident: " << name << "\n";
    }
    return false;
}

} // namespace

void install_overlay_manager(psprecomp::Runtime &runtime) {
    overlay_region_ends.clear();
    active_overlays.clear();
    for (std::size_t i = 0; i < kOverlayCount; ++i) {
        const OverlayDescriptor &overlay = kOverlays[i];
        auto &end = overlay_region_ends[overlay.load_address];
        end = std::max(end, overlay.load_address + overlay.image_size + overlay.bss_size);
        active_overlays.emplace(overlay.load_address, nullptr);
    }
    runtime.set_missing_function_resolver(&resolve_overlay_function);
    std::cerr << "[overlay] " << kOverlayCount << " recompiled overlays in "
              << overlay_region_ends.size() << " load regions\n";
}

void overlay_memory_written(psprecomp::Runtime &runtime, std::uint32_t address, std::uint32_t size) {
    if (size == 0u) return;
    const std::uint64_t end = static_cast<std::uint64_t>(address) + size;
    for (const auto &[load, region_end] : overlay_region_ends) {
        if (address < region_end && end > load) drop_region(runtime, load);
    }
}

} // namespace pes6
