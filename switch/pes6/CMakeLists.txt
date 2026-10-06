cmake_minimum_required(VERSION 3.20)
project(pes6_switch VERSION 0.1.0 LANGUAGES CXX)

# Builds the pes6 profile (PES6Native) for Switch and packages it as a .nro.
#   cmake -S switch/pes6 -B out/switch-pes6 -G Ninja \
#         -DCMAKE_TOOLCHAIN_FILE="$DEVKITPRO/cmake/Switch.cmake"
#   cmake --build out/switch-pes6
# profiles/pes6/generated/ must exist (scripts/regenerate.sh, from the user's
# own decrypted EBOOT); the game data is read from sdmc:/switch/pes6/.
# -DPES6_HOST_ONLY=ON compiles only the host sources (CI, no game data).
if(NOT NINTENDO_SWITCH)
    message(FATAL_ERROR
        "pes6_switch only targets Switch: configure with "
        "-DCMAKE_TOOLCHAIN_FILE=$DEVKITPRO/cmake/Switch.cmake")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(PSPRECOMP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(PSPRECOMP_PROFILE "pes6" CACHE STRING "" FORCE)
set(PSPRECOMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${PSPRECOMP_ROOT}" psprecomp EXCLUDE_FROM_ALL)
# gnu++20: newlib hides POSIX functions (setenv, strnlen...) under strict
# -std=c++20, which the root project selects.
if(PES6_HOST_ONLY)
    set_target_properties(pes6_host psprecomp_core PROPERTIES CXX_EXTENSIONS ON)
else()
    set_target_properties(PES6Native psprecomp_core PROPERTIES CXX_EXTENSIONS ON)
endif()
# Without it GCC keeps errno semantics for <cmath>: std::lround becomes a call
# to newlib's lroundf and every sqrt carries a libcall fallback, on the GE's
# per-vertex lighting path among others. Results are unchanged (errno only);
# Apple clang already defaults to this.
foreach(target IN ITEMS PES6Native pes6_host psprecomp_core)
    if(TARGET ${target})
        target_compile_options(${target} PRIVATE -fno-math-errno)
    endif()
endforeach()

if(PES6_HOST_ONLY)
    # CI: no game, so no generated corpus; compile-check the host only.
    add_custom_target(pes6_host_check ALL DEPENDS pes6_host)
    return()
endif()

# No Konami artwork: the title and icon are the project's own (libnx's default
# icon until there is one).
nx_generate_nacp(
    OUTPUT pes6.nacp
    NAME "PES6 Switch (unofficial)"
    AUTHOR "pes6-switch contributors"
    VERSION "${PROJECT_VERSION}")

nx_create_nro(PES6Native
    OUTPUT "${CMAKE_BINARY_DIR}/pes6.nro"
    NACP pes6.nacp)
