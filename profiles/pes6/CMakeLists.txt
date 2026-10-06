# PES6 (PSP, ULES-00476) profile.
#
# generated/ is produced locally from the user's own decrypted EBOOT by
# scripts/regenerate.sh and is never committed (the repository is public).
set(PES6_PROFILE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# The EBOOT corpus plus one sub-directory per recompiled overlay.
file(GLOB_RECURSE PES6_GENERATED CONFIGURE_DEPENDS "${PES6_PROFILE_DIR}/generated/*.cpp")
# PES6_HOST_ONLY: without generated/ (CI, which never has the game), compile
# just the host sources as the pes6_host object library.
option(PES6_HOST_ONLY "Build only the host sources (no generated corpus, no executable)" OFF)
if(NOT PES6_GENERATED AND NOT PES6_HOST_ONLY)
    message(FATAL_ERROR
        "profiles/pes6/generated is empty. Put your decrypted EBOOT at "
        "profiles/pes6/game/EBOOT_DECRYPTED.BIN and run profiles/pes6/scripts/regenerate.sh "
        "(or configure with -DPES6_HOST_ONLY=ON to compile only the host)")
endif()

if(NOT MSVC AND PES6_GENERATED)
    # Each generated unit is one very large function; -g0 keeps debug info from
    # dominating build time and memory.
    set_source_files_properties(${PES6_GENERATED} PROPERTIES COMPILE_OPTIONS
        "-O${PSPRECOMP_GENERATED_OPT_LEVEL};-g0")
endif()

file(GLOB PES6_HOST_SOURCES CONFIGURE_DEPENDS "${PES6_PROFILE_DIR}/host/*.cpp")
list(FILTER PES6_HOST_SOURCES EXCLUDE REGEX "(display_window|audio_device|atrac_decoder|media_decoder|ge_gpu_backend)_[a-z]+\\.cpp$")

# Display/input and audio backends: an SDL2 window and audio device when SDL2 is
# available (desktop, and devkitPro's switch-sdl2), otherwise headless/silent.
option(PES6_USE_SDL2 "Build the SDL2 window/input backend when SDL2 is found" ON)
if(PES6_USE_SDL2)
    find_package(SDL2 CONFIG QUIET)
endif()
if(PES6_USE_SDL2 AND SDL2_FOUND)
    # The GL GE backend needs SDL's GL context; PES6_RENDERER=gl turns it on.
    list(APPEND PES6_HOST_SOURCES
        "${PES6_PROFILE_DIR}/host/display_window_sdl.cpp"
        "${PES6_PROFILE_DIR}/host/audio_device_sdl.cpp"
        "${PES6_PROFILE_DIR}/host/ge_gpu_backend_gl.cpp")
    message(STATUS "PES6: SDL2 display and audio backend, OpenGL GE backend")
else()
    list(APPEND PES6_HOST_SOURCES
        "${PES6_PROFILE_DIR}/host/display_window_headless.cpp"
        "${PES6_PROFILE_DIR}/host/audio_device_null.cpp"
        "${PES6_PROFILE_DIR}/host/ge_gpu_backend_null.cpp")
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

# PMF movies (the intro): additionally libavformat (MPEG-PS demuxer) and
# libswscale (YUV -> RGBA).  Without them the intro is skipped.
if(PES6_USE_FFMPEG AND PES6_FFMPEG_FOUND)
    pkg_check_modules(PES6_FFMPEG_VIDEO IMPORTED_TARGET libavformat libswscale)
endif()
if(PES6_USE_FFMPEG AND PES6_FFMPEG_FOUND AND PES6_FFMPEG_VIDEO_FOUND)
    list(APPEND PES6_HOST_SOURCES "${PES6_PROFILE_DIR}/host/media_decoder_ffmpeg.cpp")
    message(STATUS "PES6: PMF movie decoding with FFmpeg")
else()
    list(APPEND PES6_HOST_SOURCES "${PES6_PROFILE_DIR}/host/media_decoder_null.cpp")
    message(STATUS "PES6: no libavformat/libswscale, PMF movies are skipped")
endif()

if(PES6_HOST_ONLY)
    set(PES6_TARGET pes6_host)
    add_library(pes6_host OBJECT ${PES6_HOST_SOURCES})
    message(STATUS "PES6: host-only build (pes6_host), no generated corpus")
else()
    set(PES6_TARGET PES6Native)
    add_executable(PES6Native ${PES6_HOST_SOURCES} ${PES6_GENERATED})
    set_target_properties(PES6Native PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()
target_include_directories(${PES6_TARGET} PRIVATE
    "${PES6_PROFILE_DIR}/host"
    "${PES6_PROFILE_DIR}/generated")
target_link_libraries(${PES6_TARGET} PRIVATE psprecomp_core)
if(PES6_USE_SDL2 AND SDL2_FOUND)
    target_link_libraries(${PES6_TARGET} PRIVATE SDL2::SDL2)
endif()
if(PES6_USE_FFMPEG AND PES6_FFMPEG_FOUND)
    target_link_libraries(${PES6_TARGET} PRIVATE PkgConfig::PES6_FFMPEG)
endif()
if(PES6_USE_FFMPEG AND PES6_FFMPEG_FOUND AND PES6_FFMPEG_VIDEO_FOUND)
    target_link_libraries(${PES6_TARGET} PRIVATE PkgConfig::PES6_FFMPEG_VIDEO)
endif()
if(NOT NINTENDO_SWITCH)
    target_link_libraries(${PES6_TARGET} PRIVATE ${CMAKE_DL_LIBS})
endif()
