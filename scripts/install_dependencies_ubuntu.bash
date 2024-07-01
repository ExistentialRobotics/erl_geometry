#!/usr/bin/env bash

set -e
set -x

wget https://github.com/qhull/qhull/archive/refs/tags/v8.1-alpha1.tar.gz
tar -xf v8.1-alpha1.tar.gz
rm v8.1-alpha1.tar.gz
cd qhull-8.1-alpha1
mkdir -p build && cd build
cmake .. -DCMAKE_C_FLAGS="$CFLAGS -ffat-lto-objects" -DCMAKE_CXX_FLAGS="$CXXFLAGS -ffat-lto-objects" -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
sudo make install
cd ../.. && rm -rf qhull-8.1-alpha1

sudo apt install -y \
    xorg-dev \
    libxcb-shm0 \
    libglu1-mesa-dev \
    python3-dev \
    libc++-dev \
    libc++abi-dev \
    libsdl2-dev \
    ninja-build \
    libxi-dev
git clone --recursive https://github.com/isl-org/Open3D.git
cd Open3D
git checkout 0f06a14
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
make install
cd ../..
rm -rf Open3D
