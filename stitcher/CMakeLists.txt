cmake_minimum_required(VERSION 3.6)

project("airmap-panorama-stitcher")

string(COMPARE EQUAL "${CMAKE_BUILD_TYPE}" "Coverage" IS_COVERAGE_BUILD)
if (IS_COVERAGE_BUILD)
    message(STATUS "Enabling coverage build")
    set(CMAKE_BUILD_TYPE 'Debug')
    #
    # Assumes GCOV-compatible compiler such as GCC or Clang.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

find_package(OpenCV 4.2 REQUIRED)

# Boost is a development dependency and this binary has very
# little to ask from Boost, so linking statically
set(Boost_USE_STATIC_LIBS ON)
find_package(
    Boost REQUIRED
    program_options
    filesystem
)

include_directories(
    include
    3rdParty
    ${OpenCV_INCLUDE_DIRS}
)

add_library(
    airmap_stitching
    src/camera.cpp
    src/camera_models.cpp
    src/cubemap.cpp
    src/cropper.cpp
    src/distortion.cpp
    src/gimbal.cpp
    src/images.cpp
    src/monitor/estimator.cpp
    src/monitor/monitor.cpp
    src/monitor/timer.cpp
    src/opencv/forward.cpp
    src/opencv/matchers.cpp
    src/opencv/seam_finders.cpp
    src/panorama.cpp
    src/stitcher.cpp
    src/stitcher_configuration.cpp
    3rdParty/TinyEXIF/TinyEXIF.cpp
    3rdParty/TinyEXIF/tinyxml2.cpp
)

add_executable (
    airmap_stitcher
    main.cpp
)

target_link_libraries(
    airmap_stitching
    ${OpenCV_LIBS}
)

target_link_libraries(
    airmap_stitcher
    airmap_stitching
    Boost::program_options
    Boost::filesystem
)

#
# Installation
#
install(TARGETS airmap_stitcher
        RUNTIME DESTINATION bin)

#
# Testing
#
enable_testing()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test/bats)

include(${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/CMakeLists.txt)
