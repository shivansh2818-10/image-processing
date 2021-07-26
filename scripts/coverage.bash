#!/bin/bash

set -xeuo pipefail

HERE="$(dirname "$(readlink -f "${0}")")"
cd "${HERE}/.."

mkdir -p build
cd build
BUILD_DIR="$(pwd)"

cmake \
    -DCMAKE_BUILD_TYPE='Coverage' \
    ../stitcher
make "-j$(nproc)"
ctest \
    "-j$(nproc)" \
    -E 'test_airmap_stitcher' # TODO: BOSS-812

python3 \
    -m pip \
        install \
           --user \
        gcovr

mkdir -p "${BUILD_DIR}/coverage"
python3 \
    -m gcovr \
    -r "${BUILD_DIR}/../stitcher" \
    -j "$(nproc)"\
    --html --html-details "${BUILD_DIR}/coverage/coverage.html" \
    -d \
    "${BUILD_DIR}"

tar -cJf coverage.tar.xz ./coverage

