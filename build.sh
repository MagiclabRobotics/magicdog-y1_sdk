#!/bin/bash
set -e

if [ -d ./build/install ]; then
    rm -rf ./build/install
fi

cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DSDK_INSTALL=ON \
    $@

cmake --build build --parallel $(nproc)

cmake --install build
