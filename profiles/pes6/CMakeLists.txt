# PES6 (PSP, ULES-00476) profile.
#
# generated/ is produced locally from the user's own decrypted EBOOT by
# scripts/regenerate.sh and is never committed (the repository is public).
set(PES6_PROFILE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# The EBOOT corpus plus one sub-directory per recompiled overlay.
file(GLOB_RECURSE PES6_GENERATED CONFIGURE_DEPENDS "${PES6_PROFILE_DIR}/generated/*.cpp")
if(NOT PES6_GENERATED)
    message(FATAL_ERROR
        "profiles/pes6/generated is empty. Put your decrypted EBOOT at "
        "profiles/pes6/game/EBOOT_DECRYPTED.BIN and run profiles/pes6/scripts/regenerate.sh")
endif()

if(NOT MSVC)
    # Each generated unit is one very large function; -g0 keeps debug info from
    # dominating build time and memory.
    set_source_files_properties(${PES6_GENERATED} PROPERTIES COMPILE_OPTIONS
        "-O${PSPRECOMP_GENERATED_OPT_LEVEL};-g0")
endif()

file(GLOB PES6_HOST_SOURCES CONFIGURE_DEPENDS "${PES6_PROFILE_DIR}/host/*.cpp")
list(FILTER PES6_HOST_SOURCES EXCLUDE REGEX "(display_window|audio_device|atrac_decoder)_[a-z]+\\.cpp$")

# Display/input and audio backends: an SDL2 window and audio device when SDL2 is
# available (desktop, and devkitPro's switch-sdl2), otherwise headless/silent.
option(PES6_USE_SDL2 "Build the SDL2 window/input backend when SDL2 is found" ON)
if(PES6_USE_SDL2)
    find_package(SDL2 CONFIG QUIET)
endif()
if(PES6_USE_SDL2 AND SDL2_FOUND)
    list(APPEND PES6_HOST_SOURCES
        "${PES6_PROFILE_DIR}/host/display_window_sdl.cpp"
        "${PES6_PROFILE_DIR}/host/audio_device_sdl.cpp")
    message(STATUS "PES6: SDL2 display and audio backend")
else()
    list(APPEND PES6_HOST_SOURCES
        "${PES6_PROFILE_DIR}/host/display_window_headless.cpp"
        "${PES6_PROFILE_DIR}/host/audio_device_null.cpp")
    message(STATUS "PES6: headless display backend, no audio device")
endif()

# ATRAC3/ATRAC3plus music: libavcodec + libavutil (LGPL, linked, not vendored)
# when pkg-config finds them -- Homebrew's ffmpeg on macOS, devkitPro's
# switch-ffmpeg on Switch.  Without them ATRAC streams play as silence.
option(PES6_USE_FFMPEG "Decode ATRAC music with FFmpeg's libavcodec when found" ON)
if(PES6_USE_FFMPEG)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(PES6_FFMPEG IMPORTED_TARGET libavcodec libavutil)
    endif()
endif()
if(PES6_USE_FFMPEG AND PES6_FFMPEG_FOUND)
    list(APPEND PES6_HOST_SOURCES "${PES6_PROFILE_DIR}/host/atrac_decoder_ffmpeg.cpp")
    message(STATUS "PES6: ATRAC decoding with FFmpeg ${PES6_FFMPEG_libavcodec_VERSION}")
else()
    list(APPEND PES6_HOST_SOURCES "${PES6_PROFILE_DIR}/host/atrac_decoder_null.cpp")
    message(STATUS "PES6: no FFmpeg, ATRAC music will be silent")
endif()

add_executable(PES6Native ${PES6_HOST_SOURCES} ${PES6_GENERATED})
target_include_directories(PES6Native PRIVATE
    "${PES6_PROFILE_DIR}/host"
    "${PES6_PROFILE_DIR}/generated")
target_link_libraries(PES6Native PRIVATE psprecomp_core)
if(PES6_USE_SDL2 AND SDL2_FOUND)
    target_link_libraries(PES6Native PRIVATE SDL2::SDL2)
endif()
if(PES6_USE_FFMPEG AND PES6_FFMPEG_FOUND)
    target_link_libraries(PES6Native PRIVATE PkgConfig::PES6_FFMPEG)
endif()
if(NOT NINTENDO_SWITCH)
    target_link_libraries(PES6Native PRIVATE ${CMAKE_DL_LIBS})
endif()
set_target_properties(PES6Native PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
