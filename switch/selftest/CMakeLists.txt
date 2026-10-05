cmake_minimum_required(VERSION 3.20)
project(pes6_selftest VERSION 0.1.0 LANGUAGES CXX)

# Builds the PSPRecomp framework for Switch and runs its regression tests as a
# homebrew app.
#   cmake -S switch/selftest -B out/switch-selftest -G Ninja \
#         -DCMAKE_TOOLCHAIN_FILE="$DEVKITPRO/cmake/Switch.cmake"
if(NOT NINTENDO_SWITCH)
    message(FATAL_ERROR
        "pes6_selftest only targets Switch: configure with "
        "-DCMAKE_TOOLCHAIN_FILE=$DEVKITPRO/cmake/Switch.cmake")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# gnu++20: newlib hides POSIX setenv() under strict -std=c++20.
set(CMAKE_CXX_EXTENSIONS ON)

set(PSPRECOMP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(PSPRECOMP_PROFILE "")
set(PSPRECOMP_BUILD_TESTS OFF)
add_subdirectory("${PSPRECOMP_ROOT}" psprecomp EXCLUDE_FROM_ALL)

set(PSPRECOMP_TEST_SOURCE "${PSPRECOMP_ROOT}/tests/test_main.cpp")
add_executable(pes6_selftest source/main.cpp "${PSPRECOMP_TEST_SOURCE}")
target_link_libraries(pes6_selftest PRIVATE psprecomp_core)
target_compile_options(pes6_selftest PRIVATE -Wall -Wextra)
set_source_files_properties("${PSPRECOMP_TEST_SOURCE}" PROPERTIES COMPILE_DEFINITIONS
    "PSPRECOMP_TESTS_NO_SUBPROCESS;PSPRECOMP_TESTS_ENTRY=psprecomp_tests_main")

nx_generate_nacp(
    OUTPUT pes6_selftest.nacp
    NAME "PES6 Switch selftest"
    AUTHOR "pes6-switch contributors"
    VERSION "${PROJECT_VERSION}")

nx_create_nro(pes6_selftest
    NACP pes6_selftest.nacp)
