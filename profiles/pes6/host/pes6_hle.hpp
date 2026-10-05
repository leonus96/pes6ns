#pragma once
#include "psprecomp/runtime.hpp"

#include <cstdint>

namespace pes6 {

// Resets every HLE table and registers the PSP OS emulation (threads, memory,
// synchronisation, display, GE, audio, media bookkeeping, I/O, utilities) on
// `runtime`. Must run after the generated code has been registered.
// `user_arena_start` is the first guest address after the loaded ELF; the
// SysMem bump allocator grows upward from it (aligned to 256 bytes) while
// thread stacks grow downward from 0x0A000000.
void install_profile(psprecomp::Runtime &runtime, std::uint32_t user_arena_start);

// Feeds the display window a dispatch/vblank heartbeat so long synchronous
// guest phases still show progress instead of looking frozen. No-op when the
// display backend is headless.
void install_display_heartbeat();

// Installs the timer-interrupt stand-in that keeps a purely computational guest
// loop from starving the PSP threads it is waiting on
// (PSPRECOMP_TIME_TICK_DISPATCHES, default 256).
void install_starvation_preemption();

// Prints how much of the emulated UMD was served from real files and how much
// was silently zero-filled because no registered file covered the sector.
void report_disc_read_stats();

// Prints the presentation census and joins the optional async GE worker
// (PSPRECOMP_GE_ASYNC). Call once after Runtime::run() returns, while the
// Runtime object is still alive.
void report_present_stats();

} // namespace pes6
