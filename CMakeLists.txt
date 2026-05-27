cmake_minimum_required(VERSION 3.15)
project(racer3-rmp-basic-motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED RSI_SDK_ROOT)
    set(RSI_SDK_ROOT "C:/RSI/11.0.5" CACHE PATH "Root directory of the RSI RapidCode SDK")
endif()

option(RACER3_BUILD_RTTASKS "Build experimental Racer3 custom RTTaskFunctions library" OFF)
option(RACER3_CONFIGURE_INTIME_RTTASKS "Also configure/build the INtime .rsl variant of RTTaskFunctions, matching the laser-demo RTTaskManager deployment model" OFF)
set(RACER3_RTTASK_RUNTIME_DIR "${RSI_SDK_ROOT}" CACHE PATH "Directory where the RMP RTTask manager loads RTTaskFunctions.dll/.rsl")

# Keep the RSI SDK root and the laser-demo-style RapidSoftware helper aligned.
# RapidSoftware.cmake uses RMP_DIR as the RMP/RTTask runtime install path.
set(RMP_DIR "${RSI_SDK_ROOT}" CACHE PATH "RMP install directory used by RapidSoftware/RTTask builds")

# The companion INtime build is only for the RTTaskFunctions library. Do not try
# to build the normal Windows session-server executable inside that child build.
# This mirrors the laser demo, where the RTTask library is the artifact deployed
# to the RMP/RTTask runtime directory.
set(RACER3_INTIME_ONLY_BUILD OFF)
if(DEFINED __INTIME__)
    set(RACER3_INTIME_ONLY_BUILD ON)
endif()

find_path(RSI_INCLUDE_DIR rsi.h HINTS "${RSI_SDK_ROOT}/include")
if(NOT RSI_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find RSI RapidCode SDK headers. Set RSI_SDK_ROOT to the SDK install root.")
endif()

if(NOT RACER3_INTIME_ONLY_BUILD)
    find_library(RSI_LIBRARY NAMES RapidCode64 RapidCode HINTS "${RSI_SDK_ROOT}/lib/x64" "${RSI_SDK_ROOT}/lib")
    if(NOT RSI_LIBRARY)
        message(FATAL_ERROR "Could not find RapidCode library in ${RSI_SDK_ROOT}.")
    endif()

    add_executable(racer3-basic-motion
        src/main.cpp
        src/Racer3BasicMotion.cpp
    )

    target_include_directories(racer3-basic-motion PRIVATE ${RSI_INCLUDE_DIR})
    target_link_libraries(racer3-basic-motion PRIVATE ${RSI_LIBRARY})

    if(WIN32)
        target_link_libraries(racer3-basic-motion PRIVATE ws2_32 Xinput)
    endif()
endif()

if(RACER3_BUILD_RTTASKS)
    # The laser demo does not treat RTTasks as ordinary host-only DLLs. It uses
    # RapidSoftware.cmake/setup_rmp_target so the task library is built/deployed
    # into the RMP runtime directory and, when requested, a companion INtime .rsl
    # build is produced. Racer3 follows that same structure for RTTask work.
    include(cmake/RapidSoftware.cmake)

    # Match the laser demo: the project-specific RTTask source is built into
    # the standard RTTaskFunctions library name. On Windows/INtime this produces
    # RTTaskFunctions.rsl in the RSI runtime directory. The contents are Racer3
    # specific even though the deployed basename stays RTTaskFunctions.
    add_library(RTTaskFunctions SHARED
        rttasks/rttaskfunctions.cpp
        rttasks/rttaskglobals.h
    )

    
target_include_directories(RTTaskFunctions PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/rttasks
    )
    target_compile_definitions(RTTaskFunctions PRIVATE RACER3_RTTASKS_EXPERIMENTAL=1)

    if(COMMAND setup_rmp_target)
        setup_rmp_target(RTTaskFunctions)
    else()
        target_include_directories(RTTaskFunctions PRIVATE ${RSI_INCLUDE_DIR})
        target_link_libraries(RTTaskFunctions PRIVATE ${RSI_LIBRARY})
    endif()

    # Match the proven laser-demo RTTask deployment pattern: the RTTask library
    # must be available from the RMP/RTTask runtime directory, not only from the
    # Visual Studio build tree. In a normal Windows build this is a DLL; in the
    # companion INtime build this becomes an .rsl.
    set_target_properties(RTTaskFunctions PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${RACER3_RTTASK_RUNTIME_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${RACER3_RTTASK_RUNTIME_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${RACER3_RTTASK_RUNTIME_DIR}"
        LIBRARY_OUTPUT_DIRECTORY "${RACER3_RTTASK_RUNTIME_DIR}"
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${RACER3_RTTASK_RUNTIME_DIR}"
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${RACER3_RTTASK_RUNTIME_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/rttasks"
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/rttasks"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/rttasks"
    )

    if(RACER3_CONFIGURE_INTIME_RTTASKS AND WIN32 AND NOT DEFINED __INTIME__ AND COMMAND configure_cmake_for_intime)
        message(STATUS "Configuring companion INtime RTTaskFunctions build for Racer3.")
        configure_cmake_for_intime(${CMAKE_SOURCE_DIR})
        configure_intime_target(RTTaskFunctions)
    endif()

    message(STATUS "RTTaskFunctions runtime output: ${RACER3_RTTASK_RUNTIME_DIR}")
endif()


if(TARGET RTTaskFunctions)
    target_compile_features(RTTaskFunctions PRIVATE cxx_std_20)
    set_target_properties(RTTaskFunctions PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
endif()