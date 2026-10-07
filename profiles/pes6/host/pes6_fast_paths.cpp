// Native replacements for guest library routines that dominate a measured
// stall (PES6-specific addresses, so they live in the profile).
//
// The EBOOT links libgcc's generic soft-float (fp-bit) for every double
// operation -- the Allegrex FPU is single precision -- and newlib's pow on top
// of it. Before kick-off the game runs pow(c / 255, 2.2) over colour
// components (followed by a weighted sum: a luminance), three soft-float calls
// per component. That was 35 of the 37 ms of host time of the stall at
// vblank 7463 of match_route.sh on the Mac (PES6_PC_PROFILE), ~0.6 s on the
// Switch, where the audio ran dry meanwhile.
//
// The float<->double conversions are computed by the host: exact, or rounded
// to nearest even like fp-bit (verify mode: no mismatch over a match). pow is
// not: the host libm differs from the game's fdlibm in the last ulp of some
// results (verify mode on macOS), so pow is memoized instead -- a miss runs
// the game's own routine and stores its result, bit-exact by construction.
// The game feeds it few distinct arguments (c / 255, exponent 2.2).
//
// PES6_SOFT_FLOAT_FAST_PATHS=0 keeps the guest routines (A/B checks);
// =verify runs guest and host for every call, returns the guest's result and
// logs bit mismatches.
#include "pes6_fast_paths.hpp"

#include <psprecomp/runtime.hpp>

#include <bit>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>

namespace pes6 {
namespace {

constexpr std::uint32_t kExtendSfDf2 = 0x089828B0u;  // double __extendsfdf2(float)
constexpr std::uint32_t kTruncDfSf2 = 0x08983CF4u;   // float __truncdfsf2(double)
constexpr std::uint32_t kPow = 0x0898D7F0u;          // double pow(double, double)

// Soft-float ABI: a double travels in two GPRs, low word first.
double double_from(const psprecomp::AllegrexContext &ctx, std::uint32_t low_register) {
    const std::uint64_t bits = static_cast<std::uint64_t>(ctx.gpr[low_register]) |
                               (static_cast<std::uint64_t>(ctx.gpr[low_register + 1u]) << 32u);
    return std::bit_cast<double>(bits);
}

void return_double(psprecomp::AllegrexContext &ctx, double value) {
    const auto bits = std::bit_cast<std::uint64_t>(value);
    ctx.set_gpr(2, static_cast<std::uint32_t>(bits));
    ctx.set_gpr(3, static_cast<std::uint32_t>(bits >> 32u));
    ctx.pc = ctx.gpr[31];
}

void native_extendsfdf2(psprecomp::AllegrexContext &ctx) {
    return_double(ctx, static_cast<double>(ctx.fpr[12]));
}

void native_truncdfsf2(psprecomp::AllegrexContext &ctx) {
    ctx.fpr[0] = static_cast<float>(double_from(ctx, 4u));
    ctx.pc = ctx.gpr[31];
}

void native_pow(psprecomp::AllegrexContext &ctx) {
    return_double(ctx, std::pow(double_from(ctx, 4u), double_from(ctx, 6u)));
}

// --- guest routines ----------------------------------------------------------
struct Original {
    std::uint32_t address{};
    const char *name{};
    psprecomp::Runtime::RecompiledFunction function{};
    void (*native)(psprecomp::AllegrexContext &){};
    std::uint64_t calls{};
    std::uint64_t mismatches{};
};
Original originals[3];

// Never a guest function: the guest routine "returns" here.
constexpr std::uint32_t kSentinelReturn = 0x00000010u;

// Runs a wrapped guest routine to completion on `ctx`, outer-dispatch style
// (a generated function may hand control back before it returns). Calls it
// makes to other replaced routines go to their replacements.
bool run_guest(psprecomp::Runtime &rt, psprecomp::AllegrexContext &ctx, const Original &original) {
    ctx.gpr[31] = kSentinelReturn;
    ctx.pc = original.address;
    for (std::uint64_t steps = 0u; ctx.pc != kSentinelReturn; ++steps) {
        const psprecomp::Runtime::RecompiledFunction function =
            ctx.pc == original.address ? original.function : rt.registered_function(ctx.pc);
        if (function == nullptr || steps > 1'000'000u) return false;
        function(rt, ctx);
        ctx.gpr[0] = 0u;
    }
    return true;
}

// pow memo: (x, y) bits -> result bits, filled by the guest routine.
struct PowKey {
    std::uint64_t x{};
    std::uint64_t y{};
    bool operator==(const PowKey &) const = default;
};
struct PowKeyHash {
    std::size_t operator()(const PowKey &key) const noexcept {
        return static_cast<std::size_t>(key.x * 0x9E3779B97F4A7C15ull ^ (key.y + (key.x >> 29u)));
    }
};
std::unordered_map<PowKey, std::uint64_t, PowKeyHash> pow_memo;
constexpr std::size_t kPowMemoLimit = 65536u;

std::uint64_t double_bits(const psprecomp::AllegrexContext &ctx, std::uint32_t low_register) {
    return static_cast<std::uint64_t>(ctx.gpr[low_register]) |
           (static_cast<std::uint64_t>(ctx.gpr[low_register + 1u]) << 32u);
}

void memoized_pow(psprecomp::Runtime &rt, psprecomp::AllegrexContext &ctx) {
    const PowKey key{double_bits(ctx, 4u), double_bits(ctx, 6u)};
    const std::uint32_t return_address = ctx.gpr[31];
    if (const auto hit = pow_memo.find(key); hit != pow_memo.end()) {
        ctx.set_gpr(2, static_cast<std::uint32_t>(hit->second));
        ctx.set_gpr(3, static_cast<std::uint32_t>(hit->second >> 32u));
        ctx.pc = return_address;
        return;
    }
    if (!run_guest(rt, ctx, originals[2])) {
        rt.stop("pes6 pow fast path: guest pow did not return");
        return;
    }
    if (pow_memo.size() < kPowMemoLimit) pow_memo.emplace(key, double_bits(ctx, 2u));
    ctx.gpr[31] = return_address;
    ctx.pc = return_address;
}

template <std::size_t Index>
void verify_call(psprecomp::Runtime &rt, psprecomp::AllegrexContext &ctx) {
    Original &original = originals[Index];
    psprecomp::AllegrexContext native = ctx;
    original.native(native);
    const std::uint32_t return_address = ctx.gpr[31];
    if (!run_guest(rt, ctx, original)) {
        std::cerr << "[soft-float-verify] " << original.name << " guest run failed at pc=0x" << std::hex
                  << ctx.pc << std::dec << "\n";
        ctx = native;
        return;
    }
    ctx.pc = return_address;
    ctx.gpr[31] = return_address;
    ++original.calls;
    const bool same = Index == 1u
        ? std::bit_cast<std::uint32_t>(ctx.fpr[0]) == std::bit_cast<std::uint32_t>(native.fpr[0])
        : ctx.gpr[2] == native.gpr[2] && ctx.gpr[3] == native.gpr[3];
    if (!same && ++original.mismatches <= 20u) {
        std::cerr << "[soft-float-verify] " << original.name << " mismatch a0=0x" << std::hex << native.gpr[4]
                  << " a1=0x" << native.gpr[5] << " a2=0x" << native.gpr[6] << " a3=0x" << native.gpr[7]
                  << " f12=0x" << native.fpr_bits(12) << " guest=0x" << ctx.gpr[3] << ":" << ctx.gpr[2] << "/"
                  << ctx.fpr_bits(0) << " native=0x" << native.gpr[3] << ":" << native.gpr[2] << "/"
                  << native.fpr_bits(0) << std::dec << "\n";
    }
    if ((original.calls & 0x3FFu) == 0u)
        std::cerr << "[soft-float-verify] " << original.name << " calls=" << original.calls
                  << " mismatches=" << original.mismatches << "\n";
}

} // namespace

void install_soft_float_fast_paths(psprecomp::Runtime &runtime) {
    const char *text = std::getenv("PES6_SOFT_FLOAT_FAST_PATHS");
    const std::string mode = text != nullptr ? text : "";
    if (mode == "0") return;
    originals[0] = {kExtendSfDf2, "extendsfdf2", runtime.registered_function(kExtendSfDf2), &native_extendsfdf2};
    originals[1] = {kTruncDfSf2, "truncdfsf2", runtime.registered_function(kTruncDfSf2), &native_truncdfsf2};
    originals[2] = {kPow, "pow", runtime.registered_function(kPow), &native_pow};
    if (originals[2].function == nullptr) return;
    if (mode == "verify") {
        runtime.register_function(kExtendSfDf2, &verify_call<0>, "pes6_verify_extendsfdf2");
        runtime.register_function(kTruncDfSf2, &verify_call<1>, "pes6_verify_truncdfsf2");
        runtime.register_function(kPow, &verify_call<2>, "pes6_verify_pow");
        std::cerr << "[soft-float-verify] on\n";
        return;
    }
    runtime.register_function(kExtendSfDf2,
        [](psprecomp::Runtime &, psprecomp::AllegrexContext &ctx) { native_extendsfdf2(ctx); },
        "pes6_extendsfdf2");
    runtime.register_function(kTruncDfSf2,
        [](psprecomp::Runtime &, psprecomp::AllegrexContext &ctx) { native_truncdfsf2(ctx); },
        "pes6_truncdfsf2");
    runtime.register_function(kPow, &memoized_pow, "pes6_pow");
}

} // namespace pes6
