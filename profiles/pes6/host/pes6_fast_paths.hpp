#pragma once

namespace psprecomp {
class Runtime;
}

namespace pes6 {

// Replaces the EBOOT's soft-float double conversions and pow with host code
// (see pes6_fast_paths.cpp). Call after the generated functions are registered.
void install_soft_float_fast_paths(psprecomp::Runtime &runtime);

} // namespace pes6
