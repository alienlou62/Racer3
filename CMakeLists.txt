cmake_minimum_required(VERSION 3.15)
project(racer3-rmp-basic-motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED RSI_SDK_ROOT)
    set(RSI_SDK_ROOT "C:/RSI/11.0.0" CACHE PATH "Root directory of the RSI RapidCode SDK")
endif()

find_path(RSI_INCLUDE_DIR rsi.h HINTS "${RSI_SDK_ROOT}/include")
find_library(RSI_LIBRARY NAMES RapidCode64 RapidCode HINTS "${RSI_SDK_ROOT}/lib/x64" "${RSI_SDK_ROOT}/lib")

if(NOT RSI_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find RSI RapidCode SDK headers. Set RSI_SDK_ROOT to the SDK install root.")
endif()
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
    target_link_libraries(racer3-basic-motion PRIVATE ws2_32)
endif()
