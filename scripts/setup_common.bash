#!/usr/bin/env bash
#
# Shared dependency-installation helpers for erl_geometry.
# This file is *sourced* by the per-release setup_ubuntu_<version>.bash and
# setup_archlinux.bash scripts; it is not meant to be executed directly.
#
# erl_geometry depends on erl_common, so run erl_common's setup first (it installs
# the base toolchain, Intel MKL / LAPACK, Eigen, plplot, yaml-cpp, boost, OpenCV,
# abseil, ...). The functions here install only erl_geometry's *extra* dependencies
# on top of that base: nanoflann, qhull, a recent CMake and Open3D. All source
# builds are idempotent (they check for an existing install first), so re-running
# a setup script — or running it after erl_common's — is cheap.

set -euo pipefail

# Use sudo only when we are not already root (works both in bare containers and
# on a normal user machine).
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo"; fi
export DEBIAN_FRONTEND=noninteractive

NPROC="${SETUP_NPROC:-$(nproc)}"   # override with SETUP_NPROC to cap build parallelism

# Pinned third-party versions.
ABSEIL_TAG="20240722.1"
NANOFLANN_TAG="v1.7.1"
QHULL_TAG="2020.2"
OPEN3D_COMMIT="1e7b174"
CMAKE_MIN="3.24.0"   # minimum CMake required by Open3D 0.19

log() { echo -e "\n\033[1;34m==> $*\033[0m"; }

apt_update() { $SUDO apt-get update; }

apt_install() { $SUDO apt-get install -y "$@"; }

# Open3D 0.19 (our pinned commit) does not compile with gcc >= 15: its SizeVector /
# Tensor headers fail under the stricter template parsing. Because downstream code
# #includes those headers, both Open3D and erl_geometry must use the same older gcc.
# Install and select gcc/g++-<v> as the default toolchain (used on Ubuntu 26.04).
use_gcc_version() {
    local v="$1"
    log "Selecting gcc/g++-${v} as default (Open3D 0.19 needs gcc < 15)"
    apt_install "gcc-${v}" "g++-${v}"
    $SUDO update-alternatives --install /usr/bin/gcc gcc "/usr/bin/gcc-${v}" 100 \
                              --slave   /usr/bin/g++ g++ "/usr/bin/g++-${v}"
    $SUDO update-alternatives --set gcc "/usr/bin/gcc-${v}"
    hash -r
    gcc --version | head -1
}

# Install CMake >= ${CMAKE_MIN} into /usr/local, but only when the system cmake is
# older (avoids downgrading a newer distro cmake on 24.04+).
ensure_cmake() {
    local have=""
    if command -v cmake >/dev/null 2>&1; then
        have="$(cmake --version | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')"
    fi
    if [ -n "${have}" ] && [ "$(printf '%s\n%s\n' "${CMAKE_MIN}" "${have}" | sort -V | head -1)" = "${CMAKE_MIN}" ]; then
        log "System CMake ${have} >= ${CMAKE_MIN}, keeping it"
        return
    fi
    log "Installing CMake ${CMAKE_MIN} (system has: ${have:-none})"
    local sh="cmake-${CMAKE_MIN}-linux-x86_64.sh"
    wget -q "https://github.com/Kitware/CMake/releases/download/v${CMAKE_MIN}/${sh}"
    chmod +x "${sh}"
    $SUDO ./"${sh}" --skip-license --prefix=/usr/local
    rm -f "${sh}"
    hash -r
}

# abseil-cpp is not packaged on Ubuntu 20.04 (no libabsl-dev). erl_common builds it
# from source there; re-check here so erl_geometry can also be set up standalone.
install_abseil_from_source() {
    if [ -d /usr/local/include/absl ] || [ -f /usr/local/lib/cmake/absl/abslConfig.cmake ]; then
        log "abseil-cpp already installed under /usr/local, skipping"; return
    fi
    log "Building abseil-cpp ${ABSEIL_TAG} (no libabsl-dev on this release)"
    local d; d="$(mktemp -d)"
    (
        cd "${d}"
        git clone --depth 1 --branch "${ABSEIL_TAG}" https://github.com/abseil/abseil-cpp.git
        cmake -S abseil-cpp -B abseil-cpp/build -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON -DABSL_PROPAGATE_CXX_STD=ON \
            -DCMAKE_CXX_FLAGS="-DNDEBUG"
        cmake --build abseil-cpp/build -j"${NPROC}"
        $SUDO cmake --install abseil-cpp/build
    )
    rm -rf "${d}"
}

install_nanoflann_from_source() {
    if [ -f /usr/local/include/nanoflann.hpp ]; then
        log "nanoflann already installed, skipping"; return
    fi
    log "Building nanoflann ${NANOFLANN_TAG}"
    local d; d="$(mktemp -d)"
    (
        cd "${d}"
        git clone --depth 1 --branch "${NANOFLANN_TAG}" https://github.com/jlblancoc/nanoflann.git
        cmake -S nanoflann -B nanoflann/build -DCMAKE_BUILD_TYPE=Release \
            -DNANOFLANN_BUILD_EXAMPLES=OFF -DNANOFLANN_BUILD_TESTS=OFF
        $SUDO cmake --install nanoflann/build
    )
    rm -rf "${d}"
}

install_qhull_from_source() {
    if ls /usr/local/include/libqhullcpp/Qhull.h >/dev/null 2>&1; then
        log "qhull (with qhullcpp) already installed, skipping"; return
    fi
    log "Building qhull ${QHULL_TAG} (no suitable libqhull-dev on this release)"
    local d; d="$(mktemp -d)"
    (
        cd "${d}"
        git clone --depth 1 --branch "${QHULL_TAG}" https://github.com/qhull/qhull.git
        cmake -S qhull -B qhull/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_SKIP_RPATH=ON \
            -DCMAKE_C_FLAGS="-fPIC -ffat-lto-objects" -DCMAKE_CXX_FLAGS="-fPIC -ffat-lto-objects"
        cmake --build qhull/build -j"${NPROC}"
        $SUDO cmake --install qhull/build
    )
    rm -rf "${d}"
}

# Ubuntu Open3D build: pulls Open3D's own system deps via util/install_deps_ubuntu.sh,
# then builds against the system fmt/gtest/jsoncpp/nanoflann/pybind11/qhullcpp.
install_open3d_from_source() {
    if [ -d /usr/local/lib/cmake/Open3D ]; then
        log "Open3D already installed under /usr/local, skipping"; return
    fi
    log "Building Open3D @ ${OPEN3D_COMMIT} (this is the long step)"
    local d; d="$(mktemp -d)"
    (
        cd "${d}"
        git clone https://github.com/isl-org/Open3D.git
        cd Open3D
        git checkout "${OPEN3D_COMMIT}"
        # CMake >= 4.0 (Ubuntu 26.04) rejects the pre-3.5 cmake_minimum_required in
        # some of Open3D's bundled 3rd-party projects (e.g. turbojpeg). This env var
        # (honored by CMake >= 3.31, ignored by older) supplies a floor for the main
        # configure and every ExternalProject sub-build. Harmless on 20.04–24.04.
        export CMAKE_POLICY_VERSION_MINIMUM=3.5
        # Use 'echo y' (one line) rather than 'yes' — 'yes' keeps writing after the
        # script exits and raises SIGPIPE (141), which aborts under pipefail/errexit.
        echo y | ./util/install_deps_ubuntu.sh
        # OPEN3D_EXTRA_CMAKE_ARGS lets a release add flags, e.g. -DUSE_SYSTEM_EIGEN3=ON
        # on 22.04+ (20.04 omits it: focal ships Eigen 3.3 < the 3.4 Open3D needs).
        cmake -S . -B build \
            -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release \
            -DUSE_SYSTEM_FMT=ON -DUSE_SYSTEM_GOOGLETEST=ON -DUSE_SYSTEM_JSONCPP=ON \
            -DUSE_SYSTEM_NANOFLANN=ON -DUSE_SYSTEM_PYBIND11=ON -DUSE_SYSTEM_QHULLCPP=ON \
            -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DBUILD_EXAMPLES=OFF \
            -DBUILD_UNIT_TESTS=OFF -DBUILD_BENCHMARKS=OFF ${OPEN3D_EXTRA_CMAKE_ARGS:-}
        cmake --build build -j"${NPROC}"
        $SUDO cmake --install build
    )
    rm -rf "${d}"
}
