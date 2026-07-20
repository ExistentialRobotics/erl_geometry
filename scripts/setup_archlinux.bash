#!/usr/bin/env bash
#
# Install erl_geometry's extra build dependencies on Arch Linux (rolling):
# qhull, nanoflann (AUR) and Open3D (built from source). Run erl_common's setup
# first (base toolchain, Intel MKL, abseil, ...); this layers the geometry-specific
# dependencies on top. Run from anywhere after cloning the repository:
#
#     bash scripts/setup_archlinux.bash
#
# Uses sudo for pacman / system installs when not run as root. Set SETUP_NPROC to
# cap build parallelism. Set SETUP_PACMAN_ONLY=1 to install only the pacman
# packages and exit (skips the AUR + Open3D source builds).
#
# Arch specifics:
#   * qhull ships the qhullcpp / qhull_r CMake targets erl needs, so it comes from
#     pacman (no source build). nanoflann is header-only and comes from the AUR.
#   * Open3D 0.19 needs three tweaks to build with gcc >= 15 (applied in
#     install_open3d_archlinux, so no older gcc toolchain is needed): add <cstdint>
#     to SmallVector.h and <memory>/<functional> to rendering/Renderer.h (libstdc++ 15
#     no longer pulls these in transitively), and drop Open3D's default -Werror (gcc
#     15+ raises new false-positive warnings, e.g. -Warray-bounds).

set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Reuse the shared helpers: log(), NPROC, SUDO, OPEN3D_COMMIT.
# shellcheck source=setup_common.bash
source "${SCRIPT_DIR}/setup_common.bash"

# --------------------------------------------------------------------------- #
# pacman helpers
# --------------------------------------------------------------------------- #

pacman_sync() {
    log "Synchronizing and upgrading the system (pacman -Syu)"
    $SUDO pacman -Syu --noconfirm --needed
}

pacman_install() { $SUDO pacman -S --needed --noconfirm "$@"; }

# Official-repo dependencies for qhull + the Open3D from-source build.
install_geometry_pacman() {
    log "Installing erl_geometry pacman dependencies (qhull + Open3D build deps)"
    pacman_install \
        qhull \
        onetbb assimp glew glfw vtk libpng libjpeg-turbo zlib \
        mesa glu libx11 libxrandr libxinerama libxcursor libxi \
        libc++ libc++abi
}

# --------------------------------------------------------------------------- #
# AUR helpers (makepkg cannot run as root, so build under a throwaway user when needed)
# --------------------------------------------------------------------------- #

AUR_BUILD_USER="${SETUP_AUR_USER:-aurbuild}"
AUR_RUNAS=()   # command prefix used to drop to a non-root build user (empty => run as self)

ensure_aur_build_user() {
    if [ "$(id -u)" -ne 0 ]; then
        AUR_RUNAS=()
        return
    fi
    if ! id "${AUR_BUILD_USER}" >/dev/null 2>&1; then
        log "Creating AUR build user '${AUR_BUILD_USER}' (makepkg refuses to run as root)"
        useradd -m -s /bin/bash "${AUR_BUILD_USER}"
    fi
    echo "${AUR_BUILD_USER} ALL=(ALL) NOPASSWD: /usr/bin/pacman" \
        | $SUDO tee /etc/sudoers.d/erl-aurbuild >/dev/null
    $SUDO chmod 0440 /etc/sudoers.d/erl-aurbuild
    AUR_RUNAS=(sudo -u "${AUR_BUILD_USER}")
}

detect_aur_helper() {
    local h
    for h in paru yay; do
        command -v "${h}" >/dev/null 2>&1 && { echo "${h}"; return; }
    done
}

# aur_install <pkg> [pkg...] : ensure each AUR package is installed. Skips packages
# already present, prefers an existing paru/yay helper, and otherwise falls back to
# a git-clone + makepkg build (under a throwaway non-root user when running as root).
aur_install() {
    local pkg missing=()
    for pkg in "$@"; do
        if pacman -Qi "${pkg}" >/dev/null 2>&1; then
            log "AUR package ${pkg} already installed, skipping"
        else
            missing+=("${pkg}")
        fi
    done
    [ "${#missing[@]}" -eq 0 ] && return 0

    local helper=""; [ "$(id -u)" -ne 0 ] && helper="$(detect_aur_helper)"
    if [ -n "${helper}" ]; then
        log "Installing AUR packages with ${helper}: ${missing[*]}"
        "${helper}" -S --needed --noconfirm "${missing[@]}"
        return
    fi

    ensure_aur_build_user
    local d
    for pkg in "${missing[@]}"; do
        log "Building AUR package from source: ${pkg}"
        d="$(mktemp -d)"
        git clone --depth 1 "https://aur.archlinux.org/${pkg}.git" "${d}/${pkg}"
        [ "$(id -u)" -eq 0 ] && $SUDO chown -R "${AUR_BUILD_USER}:${AUR_BUILD_USER}" "${d}"
        ( cd "${d}/${pkg}" && "${AUR_RUNAS[@]}" makepkg -si --noconfirm --needed )
        rm -rf "${d}"
    done
}

# Arch-specific Open3D build. Unlike the Ubuntu path we cannot run
# util/install_deps_ubuntu.sh; the required system deps are installed via pacman
# above. Builds with the system gcc (>= 15) after the gcc >= 15 source fixes below.
install_open3d_archlinux() {
    # Skip if Open3D is already available to CMake, whether from a distro/AUR package
    # (installs Open3DConfig.cmake under /usr) or a previous from-source build (/usr/local).
    if pacman -Qq open3d >/dev/null 2>&1 \
       || ls /usr/lib/cmake/Open3D*/Open3DConfig.cmake >/dev/null 2>&1 \
       || ls /usr/local/lib/cmake/Open3D*/Open3DConfig.cmake >/dev/null 2>&1; then
        log "Open3D already installed (system/AUR package or /usr[/local]); skipping source build"
        return
    fi
    log "Building Open3D @ ${OPEN3D_COMMIT} with $(gcc -dumpversion) (this is the long step)"
    local d; d="$(mktemp -d)"
    (
        cd "${d}"
        git clone https://github.com/isl-org/Open3D.git
        cd Open3D
        git checkout "${OPEN3D_COMMIT}"
        # gcc >= 15 fix: SmallVector.h uses uint32_t/uint64_t but libstdc++ 15 no longer
        # transitively includes <cstdint>, so core/Tensor/SizeVector fails to compile.
        grep -q '#include <cstdint>' cpp/open3d/core/SmallVector.h || \
            sed -i 's|#include <cstring>|#include <cstring>\n#include <cstdint>|' \
                cpp/open3d/core/SmallVector.h
        # gcc >= 15 emits new -Wall/-Wextra diagnostics (e.g. false-positive -Warray-bounds)
        # that Open3D's default -Werror turns fatal. Drop -Werror.
        sed -i 's/ -Werror//g' cmake/Open3DShowAndAbortOnWarning.cmake
        # gcc >= 15 fix: Renderer.h uses std::shared_ptr/std::function without including
        # <memory>/<functional> (libstdc++ 15 no longer pulls them in transitively).
        grep -q '#include <memory>' cpp/open3d/visualization/rendering/Renderer.h || \
            sed -i 's|#include "open3d/visualization/rendering/RendererHandle.h"|&\n\n#include <functional>\n#include <memory>|' \
                cpp/open3d/visualization/rendering/Renderer.h
        # CMake >= 4.0 rejects the pre-3.5 cmake_minimum_required in some bundled
        # 3rd-party projects; this env var supplies a floor (honored by CMake >= 3.31).
        export CMAKE_POLICY_VERSION_MINIMUM=3.5
        cmake -S . -B build \
            -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release \
            -DUSE_SYSTEM_FMT=ON -DUSE_SYSTEM_GOOGLETEST=ON -DUSE_SYSTEM_JSONCPP=ON \
            -DUSE_SYSTEM_NANOFLANN=ON -DUSE_SYSTEM_PYBIND11=ON -DUSE_SYSTEM_QHULLCPP=ON \
            -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DBUILD_EXAMPLES=OFF \
            -DBUILD_UNIT_TESTS=OFF -DBUILD_BENCHMARKS=OFF
        cmake --build build -j"${NPROC}"
        $SUDO cmake --install build
    )
    rm -rf "${d}"
}

# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

pacman_sync
install_geometry_pacman

if [ -n "${SETUP_PACMAN_ONLY:-}" ]; then
    log "SETUP_PACMAN_ONLY set — pacman packages installed, skipping AUR + Open3D builds."
    exit 0
fi

aur_install nanoflann          # header-only; from the AUR (not in the official repos)
install_open3d_archlinux       # deps from pacman (qhull/abseil/MKL) + nanoflann from AUR

log "erl_geometry Arch Linux setup complete."
