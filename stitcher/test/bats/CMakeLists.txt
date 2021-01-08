execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/install_bats.sh)

add_test (
    NAME test_airmap_stitcher
    COMMAND ./bats/bin/bats ${CMAKE_CURRENT_SOURCE_DIR}/airmap_stitcher.test.bats
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../..
)

add_test (
    NAME test_airmap_stitcher_failures
    COMMAND ../bats/bin/bats ${CMAKE_CURRENT_SOURCE_DIR}/airmap_stitcher_failures.test.bats
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/..
)