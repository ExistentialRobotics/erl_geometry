#!/usr/bin/env bash
#
# Install erl_geometry's extra build dependencies on Ubuntu: nanoflann, qhull, a
# recent CMake and Open3D. The target release is auto-detected from
# /etc/os-release, so this one script covers 20.04, 22.04, 24.04 and 26.04. Run
# erl_common's setup first (base toolchain); this layers the geometry-specific
# dependencies on top. Run from anywhere after cloning the repository:
#
#     bash scripts/setup_ubuntu.bash
#
# Uses sudo for apt / system installs when not run as root. Set SETUP_NPROC to
# cap build parallelism.
#
# Release-specific handling:
#   * 20.04 (Focal): abseil + qhull are not packaged suitably -> built from source,
#     and Open3D keeps its bundled Eigen (focal Eigen 3.3 < the 3.4 Open3D wants).
#   * 26.04: defaults to gcc-15, which cannot compile Open3D 0.19's headers, so
#     gcc-14 is pinned as the default toolchain.

set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# shellcheck source=setup_common.bash
source "${SCRIPT_DIR}/setup_common.bash"

# Detect the Ubuntu release (e.g. 20.04, 22.04, 24.04, 26.04).
UBUNTU_VERSION=""
[ -r /etc/os-release ] && UBUNTU_VERSION="$( . /etc/os-release && echo "${VERSION_ID:-}" )"
log "Detected Ubuntu release: ${UBUNTU_VERSION:-unknown}"

apt_update
if [ "${UBUNTU_VERSION}" = "20.04" ]; then
    # 20.04 (Focal): abseil + qhull are not packaged suitably -> build from source.
    install_abseil_from_source     # no libabsl-dev on 20.04 (idempotent if erl_common built it)
    install_nanoflann_from_source
    install_qhull_from_source      # no libqhull-dev with qhullcpp on 20.04
    ensure_cmake                   # 20.04 ships CMake 3.16, Open3D needs >= 3.24
    install_open3d_from_source     # keep Open3D's bundled Eigen on focal
else
    # 22.04+ : qhull (libqhull-dev, provides qhullcpp) and abseil (libabsl-dev) are
    # packaged. abseil is also pulled in by erl_common; kept here for standalone use.
    apt_install libabsl-dev libqhull-dev
    case "${UBUNTU_VERSION}" in
        22.04 | 24.04) : ;;
        26.04) use_gcc_version 14 ;;   # gcc-15 default can't build Open3D 0.19
        *) log "Untested Ubuntu '${UBUNTU_VERSION:-unknown}'; proceeding with the 24.04+ path (no gcc pin)." ;;
    esac
    install_nanoflann_from_source
    ensure_cmake                   # 22.04 ships 3.22 (upgraded); 24.04+ ship >= 3.24 (kept)
    # 22.04+ ship Eigen >= 3.4, so build Open3D against the system Eigen.
    OPEN3D_EXTRA_CMAKE_ARGS="-DUSE_SYSTEM_EIGEN3=ON" install_open3d_from_source
fi

log "erl_geometry Ubuntu ${UBUNTU_VERSION:-} setup complete."
