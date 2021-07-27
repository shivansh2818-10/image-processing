#!/bin/bash

set -xeuo pipefail

HERE="$(dirname "$(readlink -f "${0}")")"
cd "${HERE}/.."

COVERAGE_TARBALL="$(pwd)/coverage.tar.xz"
if [[ -e "${COVERAGE_TARBALL}" ]]; then
    >&2 echo "Coverage tarball \"${COVERAGE_TARBALL}\" exists already. Refusing to overwrite."
    exit 1
fi
COVERAGE_REPORT_DIR="$(pwd)/coverage"
if [[ -e "${COVERAGE_REPORT_DIR}" ]]; then
    >&2 echo "Coverage report directory \"${COVERAGE_REPORT_DIR}\" exists already. Refusing to overwrite."
    exit 1
fi

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

mkdir -p "${COVERAGE_REPORT_DIR}"
python3 \
    -m gcovr \
    -r "${BUILD_DIR}/../stitcher" \
    -j "$(nproc)"\
    --html --html-details "${COVERAGE_REPORT_DIR}/coverage.html" \
    -d \
    "${BUILD_DIR}"

tar -cJf coverage.tar.xz ./coverage

