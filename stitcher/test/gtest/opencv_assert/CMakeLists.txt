cmake_minimum_required(VERSION 3.6)

add_library(
    opencv_assert INTERFACE
)

target_link_libraries(
    opencv_assert INTERFACE
    ${OpenCV_LIBS}
    gtest
)

target_include_directories(
    opencv_assert INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/opencv_assert
)

add_executable(matCompareTests ${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/opencv_assert/test/mat_compare.cpp)
target_link_libraries(matCompareTests gtest gtest_main opencv_assert)
add_test(matCompareTests matCompareTests)
