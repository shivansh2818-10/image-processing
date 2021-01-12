cmake_minimum_required(VERSION 3.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/CMakeLists-googletest.txt)
googletest_setup()

add_executable(cameraTests test/gtest/camera.cpp)
add_executable(gimbalTests test/gtest/gimbal.cpp)
add_executable(imagesTests test/gtest/images.cpp)

target_link_libraries(cameraTests gtest gtest_main airmap_stitching)
target_link_libraries(gimbalTests gtest gtest_main airmap_stitching)
target_link_libraries(imagesTests gtest gtest_main airmap_stitching Boost::filesystem)

add_test(cameraTests cameraTests)
add_test(gimbalTests gimbalTests)
add_test(imagesTests imagesTests)
