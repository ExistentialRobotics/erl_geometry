# `erl_geometry`

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS1](https://img.shields.io/badge/ROS1-noetic-blue)](http://wiki.ros.org/)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue)](https://docs.ros.org/)

**`erl_geometry` is a C++ library for geometry processing.**

## Available Functionalities

- [Axis Aligned Bounding Box (Aabb)](include/erl_geometry/aabb.hpp): Axis Aligned Bounding Box (
  Aabb), derived
  from `Eigen::AlignedBox`.
- [KdTree](include/erl_geometry/kdtree_eigen_adaptor.hpp): create kd-tree from Eigen matrix.
- Occupancy Maps
    - [Occupancy Map](include/erl_geometry/occupancy_map.hpp): base class for occupancy mapping.
    - [LogOdd](include/erl_geometry/logodd.hpp): implementation of log odd.
    - [Log Odd Map](include/erl_geometry/log_odd_map.hpp): occupancy grid map based on log odd.
    - [Log Odd Map 2D](include/erl_geometry/log_odd_map_2d.hpp): 2D occupancy grid map based on log odd.
    - [Bayesian Hilbert Map](include/erl_geometry/bayesian_hilbert_map.hpp): probabilistic occupancy mapping using Hilbert space embeddings.
    - [Bayesian Hilbert Map Torch](include/erl_geometry/bayesian_hilbert_map_torch.hpp): PyTorch-based implementation of Bayesian Hilbert Map.
- Occupancy Quadtree: developed based on [Octomap](https://octomap.github.io/)
    - [Occupancy Quadtree](include/erl_geometry/occupancy_quadtree.hpp): implementation of occupancy
      quadtree.
    - [Colored Occupancy Quadtree](include/erl_geometry/colored_occupancy_quadtree.hpp): occupancy quadtree with color information.
    - [PyObject Occupancy Quadtree](python/binding/pyobject_occupancy_quadtree.hpp): occupancy
      quadtree that supports
      tagging Python object to quadtree nodes.
    - [Abstract Quadtree](include/erl_geometry/abstract_quadtree.hpp): abstract class for quadtree.
    - [Quadtree Implementation](include/erl_geometry/quadtree_impl.hpp): implementation of quadtree,
      unlike incremental
      quadtree, this one always inserts nodes to the deepest level.
    - [Abstract Occupancy Quadtree](include/erl_geometry/abstract_occupancy_quadtree.hpp): abstract
      class for occupancy
      quadtree.
    - [QuadtreeKey](include/erl_geometry/quadtree_key.hpp): key for quadtree node to achieve fast
      lookup.
    - [Abstract Quadtree Node](include/erl_geometry/abstract_quadtree_node.hpp): abstract class for
      quadtree node.
    - [Quadtree Data Node](include/erl_geometry/quadtree_data_node.hpp): template implementation of
      quadtree node with
      data type T.
    - [Occupancy Quadtree Node](include/erl_geometry/occupancy_quadtree_node.hpp): implementation of
      quadtree node with
      occupancy data.
    - [Colored Occupancy Quadtree Node](include/erl_geometry/colored_occupancy_quadtree_node.hpp): occupancy quadtree node with color data.
    - [PyObject Occupancy Quadtree Node](python/binding/pyobject_occupancy_quadtree_node.hpp): occupancy quadtree node that supports tagging Python object.
    - [Abstract Quadtree Drawer](include/erl_geometry/abstract_quadtree_drawer.hpp): abstract class
      for quadtree drawer.
    - [Occupancy Quadtree Drawer](include/erl_geometry/occupancy_quadtree_drawer.hpp): visualization
      of occupancy
      quadtree.
    - [Occupancy ND Tree Batch Ray Caster](include/erl_geometry/occupancy_nd_tree_batch_ray_caster.hpp): batch ray casting for N-dimensional occupancy trees.
- Occupancy Octree: developed based on [Octomap](https://octomap.github.io/)
    - [Occupancy Octree](include/erl_geometry/occupancy_octree.hpp): implementation of occupancy
      octree.
    - [Colored Occupancy Octree](include/erl_geometry/colored_occupancy_octree.hpp): occupancy octree with color information.
    - [PyObject Occupancy Octree](python/binding/pyobject_occupancy_octree.hpp): occupancy octree
      that supports
      tagging Python object to octree nodes.
    - [Abstract Octree](include/erl_geometry/abstract_octree.hpp): abstract class for octree.
    - [Octree Implementation](include/erl_geometry/octree_impl.hpp): implementation of octree,
      unlike incremental
      quadtree, this one always inserts nodes to the deepest level.
    - [Abstract Occupancy Octree](include/erl_geometry/abstract_occupancy_octree.hpp): abstract
      class for occupancy
      octree.
    - [OctreeKey](include/erl_geometry/octree_key.hpp): key for octree node to achieve fast lookup.
    - [Abstract Octree Node](include/erl_geometry/abstract_octree_node.hpp): abstract class for
      octree node.
    - [Octree Data Node](include/erl_geometry/octree_data_node.hpp): template implementation of
      octree node with data
      type T.
    - [Occupancy Octree Node](include/erl_geometry/occupancy_octree_node.hpp): implementation of
      octree node with
      occupancy data.
    - [Colored Occupancy Octree Node](include/erl_geometry/colored_occupancy_octree_node.hpp): occupancy octree node with color data.
    - [PyObject Occupancy Octree Node](python/binding/pyobject_occupancy_octree_node.hpp): occupancy octree node that supports tagging Python object.
    - [Abstract Octree Drawer](include/erl_geometry/abstract_octree_drawer.hpp): abstract class for
      octree drawer.
    - [Occupancy Octree Drawer](include/erl_geometry/occupancy_octree_drawer.hpp): visualization of
      occupancy octree.
- Collision Detection
    - [Winding Number](include/erl_geometry/winding_number.hpp): check if a point is in a polygon.
- Geometric Primitives
    - [Primitives 2D](include/erl_geometry/primitives_2d.hpp): 2D geometric primitives including lines, segments, rays, rectangles, and ellipses.
    - [Primitives 3D](include/erl_geometry/primitives_3d.hpp): 3D geometric primitives including planes, triangles, boxes, and ellipsoids.
    - [Intersection](include/erl_geometry/intersection.hpp): compute intersections between geometric primitives.
- Signed Distance Functions (SDF)
    - [Mesh SDF](include/erl_geometry/mesh_sdf.hpp): compute signed distance function from triangle mesh.
    - [SDF Utilities](include/erl_geometry/sdf/sdf_util.hpp): utility functions for SDF computation.
- Surface Extraction
    - [Marching Squares](include/erl_geometry/marching_squares.hpp): extract surface from 2D scalar
      field.
    - [Marching Cubes](include/erl_geometry/marching_cubes.hpp): extract surface from 3D scalar field using marching cubes algorithm.
- Polygon Triangulation
    - [EarCut](include/erl_geometry/earcut.hpp): triangulate a polygon even with holes.
    - [Convert Polygon to Triangle Mesh](include/erl_geometry/polygon_to_mesh.hpp): triangulate a
      polygon.
- Ray Casting
    - [Bresenham 2D](include/erl_geometry/bresenham_2d.hpp): 2D ray tracing.
    - [Ray Marching](include/erl_geometry/ray_marching.hpp): 2D ray marching with SDF.
    - Also available in Quadtree and Octree.
- Camera Models and Sensors
    - [Camera Intrinsic](include/erl_geometry/camera_intrinsic.hpp): camera intrinsic parameters and projection models.
    - [Camera Base 3D](include/erl_geometry/camera_base_3d.hpp): base class for 3D camera models.
    - [Range Sensor 3D](include/erl_geometry/range_sensor_3d.hpp): 3D range sensor simulation.
    - [Range Sensor Frame 3D](include/erl_geometry/range_sensor_frame_3d.hpp): data structure for 3D range sensor measurements.
- 2D Point Cloud Processing
    - [Surface 2D](include/erl_geometry/surface_2d.hpp): data structure to store 2D surface
      consisting of points and
      normals.
    - [Space 2D](include/erl_geometry/space_2d.hpp): algorithm to compute SDF, SDDF and normals.
    - [LiDAR 2D](include/erl_geometry/lidar_2d.hpp): generate 2D LiDAR scan with a given space.
    - [LiDAR Frame 2D](include/erl_geometry/lidar_frame_2d.hpp): data structure to store 2D LiDAR
      scan and sample from
      it.
- 3D Point Cloud Processing
    - [LiDAR 3D](include/erl_geometry/lidar_3d.hpp): generate 3D LiDAR scan with a given scene.
    - [LiDAR Frame 3D](include/erl_geometry/lidar_frame_3d.hpp): data structure to store 3D LiDAR
      scan and sample from
      it.
    - [Depth Camera 3D](include/erl_geometry/depth_camera_3d.hpp): generate 3D depth image with a
      given scene.
    - [Depth Frame 3D](include/erl_geometry/depth_frame_3d.hpp): data structure to store a depth
      image and sample from
      it.
    - [RGBD Camera 3D](include/erl_geometry/rgbd_camera_3d.hpp): generate 3D RGBD image with a given
      scene.
    - [RGBD Frame 3D](include/erl_geometry/rgbd_frame_3d.hpp): data structure to store a RGBD image
      and sample from it.
- Visualization and Rendering
    - [Open3D Helper](include/erl_geometry/open3d_helper.hpp): helper functions for Open3D integration.
    - [Open3D Visualizer Wrapper](include/erl_geometry/open3d_visualizer_wrapper.hpp): wrapper for Open3D visualization.
- Motion and Trajectory
    - [Trajectory](include/erl_geometry/trajectory.hpp): data structures and utilities for 2D/3D trajectories and SE(2)/SE(3) poses.
- Datasets
    - [HouseExpoMap](include/erl_geometry/house_expo_map.hpp): load 2D map or 3D mesh from HouseExpo
      dataset.
    - [HouseExpoMapLidar2D](include/erl_geometry/house_expo_map_lidar_2d.hpp): generate 2D LiDAR
      scan sequence from a
      HouseExpo map.
    - [GazeboRoom2D](include/erl_geometry/gazebo_room_2d.hpp): generate 2D LiDAR scan sequence from
      a Gazebo room.
    - [CityStreetMaps](include/erl_geometry/city_street_map.hpp): load 2D map from CityStreet
      dataset.
    - [UcsdFah2D](include/erl_geometry/ucsd_fah_2d.hpp): a real 2D LiDAR scan sequence collected
      from FAH building in
      UCSD.
    - [CowAndLady](include/erl_geometry/cow_and_lady.hpp): a real 3D RGBD sequence provided
      by [ETH Zurich ASL](https://projects.asl.ethz.ch/datasets/doku.php?id=iros2017).
- Others
    - [Compute Intersections](include/erl_geometry/utils.hpp): utility functions for geometric computations.
    - [Euler Angle](include/erl_geometry/euler_angle.hpp): Euler angle conversions and utilities.
    - [Compute ConvexHull](include/erl_geometry/convex_hull.hpp): convex hull computation algorithms.
    - [Hidden Point Removal](include/erl_geometry/hidden_point_removal.hpp): remove hidden points from 3D point clouds.
    - [ND Tree Setting](include/erl_geometry/nd_tree_setting.hpp): configuration settings for N-dimensional trees.
    - [Occupancy ND Tree Setting](include/erl_geometry/occupancy_nd_tree_setting.hpp): configuration settings for occupancy N-dimensional trees.

## Getting Started

### Prerequisites

- CMake 3.24 or higher
- A C++17 compatible compiler

### Create Workspace

```bash
mkdir -p <your_workspace>/src && \
vcs import --input https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry/refs/heads/main/erl_geometry.repos <your_workspace>/src
```

### Dependencies

- [erl_cmake_tools](https://github.com/ExistentialRobotics/erl_cmake_tools)
- [erl_common](https://github.com/ExistentialRobotics/erl_common)
- [erl_covariance](https://github.com/ExistentialRobotics/erl_covariance)

```bash
# Ubuntu 20.04
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_common/refs/heads/main/scripts/setup_ubuntu_20.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry/refs/heads/main/scripts/setup_ubuntu_20.04.bash | bash
# Ubuntu 22.04, 24.04
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_common/refs/heads/main/scripts/setup_ubuntu_22.04_24.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry/refs/heads/main/scripts/setup_ubuntu_22.04_24.04.bash | bash
```

### Docker Option

The easiest way to get started is to use the provided [Docker files](./docker), which contains all dependencies.

### Use as a standard CMake package

```bash
cd <your_workspace>
touch CMakeLists.txt
```

Add the following lines to your `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.24)
project(<your_project_name>)
add_subdirectory(src/erl_cmake_tools)
add_subdirectory(src/erl_common)
add_subdirectory(src/erl_covariance)
add_subdirectory(src/erl_geometry)
```

Then run the following commands:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
```

### Use as a ROS Package

```bash
cd <your_workspace>
source /opt/ros/<distro>/setup.bash
# for ROS1
catkin build erl_geometry
source devel/setup.bash
# for ROS2
colcon build --packages-up-to erl_geometry
source install/setup.bash
```
See also 🚪[erl_geometry_ros](https://github.com/ExistentialRobotics/erl_geometry_ros) for additional ROS tools.

### Install As Python Package

- Make sure you have installed all dependencies.
- Make sure you have the correct Python environment activated, `pipenv` is recommended.

```bash
cd <your_workspace>
for package in erl_cmake_tools erl_common erl_covariance erl_geometry; do
    cd src/$package
    pip install . --verbose
    cd ../..
done
```
