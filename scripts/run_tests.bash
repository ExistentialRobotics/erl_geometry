#!/usr/bin/env bash

set -e
SCRIPT_DIR=$(dirname "$(realpath "$0")")
BUILD_DIR=${SCRIPT_DIR}/../../../build
BIN_DIR=${BUILD_DIR}/src/erl_geometry

cd ${BUILD_DIR}

function build_target() {
    local target=$1
    if ! make -j8 ${target}; then
        echo "Failed to build target: ${target}"
        exit 1
    fi
}

build_target "test_bayesian_hilbert_map_2d"
${BIN_DIR}/test_bayesian_hilbert_map_2d --gtest_filter=BayesianHilbertMap.2Df --config ${SCRIPT_DIR}/../config/test_bayesian_hilbert_map_2d.yaml

build_target "test_bayesian_hilbert_map_torch_2d"
${BIN_DIR}/test_bayesian_hilbert_map_torch_2d --gtest_filter=BayesianHilbertMap.Torch2Df --config ${SCRIPT_DIR}/../config/test_bayesian_hilbert_map_2d.yaml

build_target "test_city_street_map"
${BIN_DIR}/test_city_street_map

build_target "test_convex_hull_3d_impls"
${BIN_DIR}/test_convex_hull_3d_impls

build_target "test_convex_hull"
${BIN_DIR}/test_convex_hull

build_target "test_cow_and_lady"
${BIN_DIR}/test_cow_and_lady --max_wp_idx 10

build_target "test_depth_frame_3d_basic"
${BIN_DIR}/test_depth_frame_3d_basic

build_target "test_depth_frame_3d_serialization"
${BIN_DIR}/test_depth_frame_3d_serialization

build_target "test_find_voxel_indices_torch"
${BIN_DIR}/test_find_voxel_indices_torch

build_target "test_gazebo_room_2d"
${BIN_DIR}/test_gazebo_room_2d

build_target "test_hidden_point_removal_2d"
${BIN_DIR}/test_hidden_point_removal_2d

build_target "test_hidden_point_removal_3d"
${BIN_DIR}/test_hidden_point_removal_3d

build_target "test_house_expo_map"
${BIN_DIR}/test_house_expo_map

build_target "test_intersection_line_ellipse"
${BIN_DIR}/test_intersection_line_ellipse

build_target "test_libmorton_torch"
${BIN_DIR}/test_libmorton_torch

build_target "test_lidar_frame_2d_basic"
${BIN_DIR}/test_lidar_frame_2d_basic

build_target "test_lidar_frame_2d_serialization"
${BIN_DIR}/test_lidar_frame_2d_serialization

build_target "test_lidar_frame_3d_basic"
${BIN_DIR}/test_lidar_frame_3d_basic

build_target "test_lidar_frame_3d_serialization"
${BIN_DIR}/test_lidar_frame_3d_serialization

build_target "test_marching_cubes"
${BIN_DIR}/test_marching_cubes

build_target "test_mesh_sdf"
${BIN_DIR}/test_mesh_sdf

build_target "test_newer_college"
${BIN_DIR}/test_newer_college --max_wp_idx 10

build_target "test_occupancy_octree_basic"
${BIN_DIR}/test_occupancy_octree_basic

build_target "test_occupancy_octree_build_cow_and_lady"
${BIN_DIR}/test_occupancy_octree_build_cow_and_lady --max_wp_idx 10

build_target "test_occupancy_octree_build_newer_college"
${BIN_DIR}/test_occupancy_octree_build_newer_college --max_wp_idx 10

build_target "test_occupancy_octree_build_profiling"
${BIN_DIR}/test_occupancy_octree_build_profiling

build_target "test_occupancy_octree_build"
${BIN_DIR}/test_occupancy_octree_build

build_target "test_occupancy_octree_impls"
${BIN_DIR}/test_occupancy_octree_impls

build_target "test_occupancy_octree_ray_casting"
${BIN_DIR}/test_occupancy_octree_ray_casting

build_target "test_occupancy_quadtree_basic"
${BIN_DIR}/test_occupancy_quadtree_basic

build_target "test_occupancy_quadtree_batch_ray_casting"
${BIN_DIR}/test_occupancy_quadtree_batch_ray_casting

build_target "test_occupancy_quadtree_build_from_image"
${BIN_DIR}/test_occupancy_quadtree_build_from_image

build_target "test_occupancy_quadtree_build"
${BIN_DIR}/test_occupancy_quadtree_build

build_target "test_occupancy_quadtree_find_neighbors"
${BIN_DIR}/test_occupancy_quadtree_find_neighbors

build_target "test_occupancy_quadtree_house_expo"
${BIN_DIR}/test_occupancy_quadtree_house_expo

build_target "test_occupancy_quadtree_leaf_in_aabb"
${BIN_DIR}/test_occupancy_quadtree_leaf_in_aabb

build_target "test_occupancy_quadtree_leaf_of_node"
${BIN_DIR}/test_occupancy_quadtree_leaf_of_node

build_target "test_occupancy_quadtree_leaf_on_ray"
${BIN_DIR}/test_occupancy_quadtree_leaf_on_ray

build_target "test_occupancy_quadtree_leaf_vertices"
${BIN_DIR}/test_occupancy_quadtree_leaf_vertices

build_target "test_occupancy_quadtree_ray_casting"
${BIN_DIR}/test_occupancy_quadtree_ray_casting

build_target "test_semi_sparse_octree"
${BIN_DIR}/test_semi_sparse_octree

build_target "test_semi_sparse_quadtree"
${BIN_DIR}/test_semi_sparse_quadtree