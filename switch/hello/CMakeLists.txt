cmake_minimum_required(VERSION 3.20)
project(pes6_hello VERSION 0.1.0 LANGUAGES CXX)

# Standalone Phase 1 homebrew; it does not pull in the PSPRecomp framework.
#   cmake -S switch/hello -B out/switch-hello -G Ninja \
#         -DCMAKE_TOOLCHAIN_FILE="$DEVKITPRO/cmake/Switch.cmake"
if(NOT NINTENDO_SWITCH)
    message(FATAL_ERROR
        "pes6_hello only targets Switch: configure with "
        "-DCMAKE_TOOLCHAIN_FILE=$DEVKITPRO/cmake/Switch.cmake")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Switch.cmake already links -lnx -lm and sets the A57 arch flags.
add_executable(pes6_hello source/main.cpp)
target_compile_options(pes6_hello PRIVATE -Wall -Wextra)

nx_generate_nacp(
    OUTPUT pes6_hello.nacp
    NAME "PES6 Switch hello"
    AUTHOR "pes6-switch contributors"
    VERSION "${PROJECT_VERSION}")

# Default libnx icon for now; Phase 7 adds our own (never Konami artwork).
nx_create_nro(pes6_hello
    NACP pes6_hello.nacp)
