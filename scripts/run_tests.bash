#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(dirname "$(realpath "$0")")
BUILD_DIR=${SCRIPT_DIR}/../../../build
BIN_DIR=${BUILD_DIR}/src/erl_geometry

cd ${BUILD_DIR}

function build_target() {
    cd ${BUILD_DIR}
    local target=$1
    if ! make -j8 ${target}; then
        echo "Failed to build target: ${target}"
        exit 1
    fi
}

function run_target() {
    local target=$1
    if [ ! -x "${BIN_DIR}/${target}" ]; then
        echo "${target} does not exist"
        exit 1
    fi
    cd "${BIN_DIR}"
    "./${target}" "${@:2}"
}

function run_test() {
    local target=$1
    build_target ${target}
    run_target $@
}

tests=(
"test_bayesian_hilbert_map_2d --gtest_filter=BayesianHilbertMap.2Df --config ${SCRIPT_DIR}/../config/test_bayesian_hilbert_map_2d.yaml"
"test_bayesian_hilbert_map_torch_2d --gtest_filter=BayesianHilbertMap.Torch2Df --config ${SCRIPT_DIR}/../config/test_bayesian_hilbert_map_2d.yaml"
"test_city_street_map"
"test_convex_hull_3d_impls"
"test_convex_hull"
"test_cow_and_lady --max_wp_idx 10"
"test_depth_frame_3d_basic"
"test_depth_frame_3d_serialization"
"test_find_voxel_indices_torch"
"test_gazebo_room_2d"
"test_hidden_point_removal_2d"
"test_hidden_point_removal_3d"
"test_house_expo_map"
"test_intersection_line_ellipse"
"test_libmorton_torch"
"test_lidar_frame_2d_basic"
"test_lidar_frame_2d_serialization"
"test_lidar_frame_3d_basic"
"test_lidar_frame_3d_serialization"
"test_marching_cubes"
"test_mesh_sdf"
"test_newer_college --max_wp_idx 10"
"test_occupancy_octree_basic"
"test_occupancy_octree_build_cow_and_lady --max_wp_idx 10"
"test_occupancy_octree_build_newer_college --max_wp_idx 10"
"test_occupancy_octree_build_profiling"
"test_occupancy_octree_build"
"test_occupancy_octree_impls"
"test_occupancy_octree_ray_casting"
"test_occupancy_quadtree_basic"
"test_occupancy_quadtree_batch_ray_casting"
"test_occupancy_quadtree_build_from_image"
"test_occupancy_quadtree_build"
"test_occupancy_quadtree_find_neighbors"
"test_occupancy_quadtree_house_expo"
"test_occupancy_quadtree_leaf_in_aabb"
"test_occupancy_quadtree_leaf_of_node"
"test_occupancy_quadtree_leaf_on_ray"
"test_occupancy_quadtree_leaf_vertices"
"test_occupancy_quadtree_ray_casting"
"test_semi_sparse_octree"
"test_semi_sparse_quadtree"
)

for spec in "${tests[@]}"; do
    run_test $spec
done