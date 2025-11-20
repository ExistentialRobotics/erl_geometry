#!/usr/bin/env zsh

if [ "$(uname)" != "Darwin" ]; then
    echo "This script is only for macOS."
    exit 1
fi

set -e

SCRIPT_DIR="${0:A:h}"

if [ ! -d "Open3D" ]; then
    git clone --recursive https://github.com/isl-org/Open3D.git
fi

cd Open3D
git checkout 02674268f706be4b004bbbf3d39b95fa9de35f74
patch -p1 < ${SCRIPT_DIR}/open3d-macos.patch
mkdir -p build
cd build
LAPACKE_DIR=/opt/homebrew/opt/lapack cmake .. \
-DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
-DCMAKE_CXX_FLAGS="-I/opt/homebrew/opt/minizip/include/minizip/" \
-DBUILD_SHARED_LIBS=ON \
-DCMAKE_BUILD_TYPE=Release \
-DBUILD_CUDA_MODULE=OFF \
-DUSE_SYSTEM_ASSIMP=ON \
-DUSE_SYSTEM_BLAS=OFF \
-DUSE_SYSTEM_CURL=OFF \
-DUSE_SYSTEM_EIGEN3=ON \
-DUSE_SYSTEM_FMT=ON \
-DUSE_SYSTEM_GLEW=ON \
-DUSE_SYSTEM_GLFW=ON \
-DUSE_SYSTEM_GOOGLETEST=ON \
-DUSE_SYSTEM_JPEG=ON \
-DUSE_SYSTEM_JSONCPP=ON \
-DUSE_SYSTEM_NANOFLANN=ON \
-DUSE_SYSTEM_PYBIND11=ON \
-DUSE_SYSTEM_PNG=ON \
-DUSE_SYSTEM_QHULLCPP=ON \
-DUSE_SYSTEM_VTK=ON \
-DUSE_SYSTEM_ZEROMQ=OFF \
-DGLIBCXX_USE_CXX11_ABI=ON \
-DBUILD_PYTHON_MODULE=OFF \
-DBUILD_EXAMPLES=OFF \
-DBUILD_UNIT_TESTS=OFF \
-DBUILD_BENCHMARKS=OFF
make -j$(sysctl -n hw.ncpu)
sudo make install