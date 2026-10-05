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

add_executable(PES6Native ${PES6_HOST_SOURCES} ${PES6_GENERATED})
target_include_directories(PES6Native PRIVATE
    "${PES6_PROFILE_DIR}/host"
    "${PES6_PROFILE_DIR}/generated")
target_link_libraries(PES6Native PRIVATE psprecomp_core)
if(NOT NINTENDO_SWITCH)
    target_link_libraries(PES6Native PRIVATE ${CMAKE_DL_LIBS})
endif()
set_target_properties(PES6Native PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
