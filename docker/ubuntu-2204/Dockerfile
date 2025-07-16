ARG BASE_IMAGE=erl/common:22.04
FROM ${BASE_IMAGE}

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y \
    libabsl-dev \
    libqhull-dev

# libnanoflann-dev and libopen3d-dev are too old in Ubuntu 22.04

WORKDIR /root

# install nanoflann
RUN git clone https://github.com/jlblancoc/nanoflann && \
    cd nanoflann && git checkout v1.7.1 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make install && \
    cd /root && rm -rf nanoflann

# Open3D 0.19.0 requires at least CMake 3.24
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0-linux-x86_64.sh &&\
    chmod +x cmake-3.24.0-linux-x86_64.sh && \
    ./cmake-3.24.0-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.24.0-linux-x86_64.sh
# install Open3D
RUN git clone --recursive https://github.com/isl-org/Open3D.git && cd Open3D && \
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
    cd /root && rm -rf Open3D
