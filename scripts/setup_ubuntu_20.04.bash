#!/usr/bin/bash

mkdir tmp
cd tmp

# install nanoflann
git clone https://github.com/jlblancoc/nanoflann && \
cd nanoflann && git checkout v1.7.1 && \
mkdir build && cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release && \
make install && \
cd ../.. && \
rm -rf nanoflann

# install abseil-cpp
git clone --recursive https://github.com/abseil/abseil-cpp.git && cd abseil-cpp && \
git checkout 20240722.1 && mkdir -p build && cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="${CXXFLAGS} -DNDEBUG" \
   -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON -DABSL_PROPAGATE_CXX_STD=ON && \
make -j`nproc` && make install && \
cd ../.. && \
rm -rf abseil-cpp

# install qhull
git clone --recursive https://github.com/qhull/qhull.git && cd qhull && \
git checkout 2020.2 && mkdir -p my_build && cd my_build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fPIC -ffat-lto-objects" \
  -DCMAKE_C_FLAGS="-fPIC -ffat-lto-objects" -DCMAKE_SKIP_RPATH=ON && \
make -j`nproc` && make install && \
cd ../.. && \
rm -rf qhull

# Open3D 0.19.0 requires at least CMake 3.24
wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0-linux-x86_64.sh &&\
chmod +x cmake-3.24.0-linux-x86_64.sh && \
./cmake-3.24.0-linux-x86_64.sh --skip-license --prefix=/usr/local && \
rm cmake-3.24.0-linux-x86_64.sh

# install Open3D
git clone --recursive https://github.com/isl-org/Open3D.git && cd Open3D && \
git checkout 1e7b174 && echo y | util/install_deps_ubuntu.sh && mkdir -p build && \
cd build && \
PATH="/usr/local/bin:${PATH}" cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_SYSTEM_FMT=ON \
    -DUSE_SYSTEM_GOOGLETEST=ON \
    -DUSE_SYSTEM_JSONCPP=ON \
    -DUSE_SYSTEM_NANOFLANN=ON \
    -DUSE_SYSTEM_PYBIND11=ON \
    -DUSE_SYSTEM_QHULLCPP=ON \
    -DGLIBCXX_USE_CXX11_ABI=ON \
    -DBUILD_PYTHON_MODULE=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_BENCHMARKS=OFF && \
make -j`nproc` && make install && \
cd ../.. && \
rm -rf Open3D

# Clean up
cd ..
rm -rf tmp
