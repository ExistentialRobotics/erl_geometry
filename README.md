`erl_geometry`
==============
This CMake project provides `erl_geometry` which is a C++ library for geometry processing.

- [Axis Aligned Bounding Box (Aabb)](include/erl_geometry/aabb.hpp): Axis Aligned Bounding Box (Aabb), derived
  from `Eigen::AlignedBox`.
- [KdTree](include/erl_geometry/kdtree_eigen_adaptor.hpp): create kd-tree from Eigen matrix.
- Occupancy Quadtree: developed based on [Octomap](https://octomap.github.io/)
    - [Occupancy Quadtree](include/erl_geometry/occupancy_quadtree.hpp): implementation of occupancy quadtree.
    - [PyObject Occupancy Quadtree](python/binding/pyobject_occupancy_quadtree.hpp): occupancy quadtree that supports
      tagging Python object to quadtree nodes.
    - [Abstract Quadtree](include/erl_geometry/abstract_quadtree.hpp): abstract class for quadtree.
    - [Quadtree Implementation](include/erl_geometry/quadtree_impl.hpp): implementation of quadtree, unlike incremental
      quadtree, this one always inserts nodes to the deepest level.
    - [Abstract Occupancy Quadtree](include/erl_geometry/abstract_occupancy_quadtree.hpp): abstract class for occupancy
      quadtree.
    - [QuadtreeKey](include/erl_geometry/quadtree_key.hpp): key for quadtree node to achieve fast lookup.
    - [Abstract Quadtree Node](include/erl_geometry/abstract_quadtree_node.hpp): abstract class for quadtree node.
    - [Quadtree Data Node](include/erl_geometry/quadtree_data_node.hpp): template implementation of quadtree node with
      data type T.
    - [Occupancy Quadtree Node](include/erl_geometry/occupancy_quadtree_node.hpp): implementation of quadtree node with
      occupancy data.
    - [LogOdd](include/erl_geometry/logodd.hpp): implementation of log odd.
    - [Abstract Quadtree Drawer](include/erl_geometry/abstract_quadtree_drawer.hpp): abstract class for quadtree drawer.
    - [Occupancy Quadtree Drawer](include/erl_geometry/occupancy_quadtree_drawer.hpp): visualization of occupancy
      quadtree.
- Occupancy Octree: developed based on [Octomap](https://octomap.github.io/)
    - [Occupancy Octree](include/erl_geometry/occupancy_octree.hpp): implementation of occupancy octree.
    - [PyObject Occupancy Octree](python/binding/pyobject_occupancy_octree.hpp): occupancy octree that supports
      tagging Python object to octree nodes.
    - [Abstract Octree](include/erl_geometry/abstract_octree.hpp): abstract class for octree.
    - [Octree Implementation](include/erl_geometry/octree_impl.hpp): implementation of octree, unlike incremental
      quadtree, this one always inserts nodes to the deepest level.
    - [Abstract Occupancy Octree](include/erl_geometry/abstract_occupancy_octree.hpp): abstract class for occupancy
      octree.
    - [OctreeKey](include/erl_geometry/octree_key.hpp): key for octree node to achieve fast lookup.
    - [Abstract Octree Node](include/erl_geometry/abstract_octree_node.hpp): abstract class for octree node.
    - [Octree Data Node](include/erl_geometry/octree_data_node.hpp): template implementation of octree node with data
      type T.
    - [Occupancy Octree Node](include/erl_geometry/occupancy_octree_node.hpp): implementation of octree node with
      occupancy data.
    - [Abstract Octree Drawer](include/erl_geometry/abstract_octree_drawer.hpp): abstract class for octree drawer.
    - [Occupancy Octree Drawer](include/erl_geometry/occupancy_octree_drawer.hpp): visualization of occupancy octree.
- [Log Odd Map 2D](include/erl_geometry/log_odd_map_2d.hpp): 2D occupancy grid map based on log odd.
- Collision Detection
    - [Winding Number](include/erl_geometry/winding_number.hpp): check if a point is in a polygon.
- Surface Extraction
    - [Marching Square](include/erl_geometry/marching_square.hpp): extract surface from 2D scalar field.
- Polygon Triangulation
    - [EarCut](include/erl_geometry/earcut.hpp): triangulate a polygon even with holes.
    - [Convert Polygon to Triangle Mesh](include/erl_geometry/polygon_to_mesh.hpp): triangulate a polygon.
- Ray Casting
    - [Bresenham 2D](include/erl_geometry/bresenham_2d.hpp): 2D ray tracing.
    - [Ray Marching](include/erl_geometry/ray_marching.hpp): 2D ray marching with SDF.
    - Also available in Quadtree and Octree.
- 2D Point Cloud Processing
    - [Surface 2D](include/erl_geometry/surface_2d.hpp): data structure to store 2D surface consisting of points and
      normals.
    - [Space 2D](include/erl_geometry/space_2d.hpp): algorithm to compute SDF, SDDF and normals.
    - [LiDAR 2D](include/erl_geometry/lidar_2d.hpp): generate 2D LiDAR scan with a given space.
    - [LiDAR Frame 2D](include/erl_geometry/lidar_frame_2d.hpp): data structure to store 2D LiDAR scan and sample from
      it.
- 3D Point Cloud Processing
    - [LiDAR 3D](include/erl_geometry/lidar_3d.hpp): generate 3D LiDAR scan with a given scene.
    - [LiDAR Frame 3D](include/erl_geometry/lidar_frame_3d.hpp): data structure to store 3D LiDAR scan and sample from
      it.
    - [Depth Camera 3D](include/erl_geometry/depth_camera_3d.hpp): generate 3D depth image with a given scene.
    - [Depth Frame 3D](include/erl_geometry/depth_frame_3d.hpp): data structure to store a depth image and sample from
      it.
    - [RGBD Camera 3D](include/erl_geometry/rgbd_camera_3d.hpp): generate 3D RGBD image with a given scene.
    - [RGBD Frame 3D](include/erl_geometry/rgbd_frame_3d.hpp): data structure to store a RGBD image and sample from it.
- Datasets
    - [HouseExpoMap](include/erl_geometry/house_expo_map.hpp): load 2D map or 3D mesh from HouseExpo dataset.
    - [HouseExpoMapLidar2D](include/erl_geometry/house_expo_map_lidar_2d.hpp): generate 2D LiDAR scan sequence from a
      HouseExpo map.
    - [GazeboRoom2D](include/erl_geometry/gazebo_room_2d.hpp): generate 2D LiDAR scan sequence from a Gazebo room.
    - [CityStreetMaps](include/erl_geometry/city_street_map.hpp): load 2D map from CityStreet dataset.
    - [UcsdFah2D](include/erl_geometry/ucsd_fah_2d.hpp): a real 2D LiDAR scan sequence collected from FAH building in
      UCSD.
    - [CowAndLady](include/erl_geometry/cow_and_lady.hpp): a real 3D RGBD sequence provided
      by [ETH Zurich ASL](https://projects.asl.ethz.ch/datasets/doku.php?id=iros2017).
- Others
    - [Compute Intersections](include/erl_geometry/utils.hpp)
    - [Euler Angle](include/erl_geometry/euler_angle.hpp)
    - [Compute ConvexHull](include/erl_geometry/convex_hull.hpp)
    - [Hidden Point Removal](include/erl_geometry/hidden_point_removal.hpp)

# Install Dependencies

## Ubuntu

```bash
sudo apt install libqhull-dev libopen3d-dev liboctomap-dev
```

## Arch Linux

```bash
sudo pacman -S qhull
paru -S open3d octomap
```

`erl_geometry` also depends on [erl_common](https://github.com/ExistentialRobotics/erl_common).

# Getting Started

## Build C++ Library

- Make sure you have [erl_common](https://github.com/ExistentialRobotics/erl_common) placed in the same directory
  where `erl_geometry` is placed.
- Make sure you have installed all dependencies required by `erl_common` and `erl_geometry`.
- Create a `CMakeLists.txt` file in the same directory where `erl_geometry` is placed, with the following content:

```cmake
cmake_minimum_required(VERSION 3.24)
project(
        <YOUR_PROJECT_NAME>
        LANGUAGES CXX
)

add_subdirectory(erl_common)
add_subdirectory(erl_geometry)
```

Then run the following commands:

```bash
mkdir build
cd build
cmake ..  # default build type is Release
make -j`nproc`
make install  # if you want to install the library, sudo may be required
```

## Use as a Catkin Package

```bash
cd <your_workspace>/src
git clone https://github.com/ExistentialRobotics/erl_geometry.git
cd ..
catkin build
```

## Install As Python Package

- Make sure you have [erl_common](https://github.com/ExistentialRobotics/erl_common) placed in the same directory
  where `erl_geometry` is placed.
- Make sure you have installed all dependencies required by `erl_common` and `erl_geometry`.
- Make sure you have the correct Python environment activated, `pipenv` is recommended.
- Then run the following commands:

```bash
cd erl_geometry
pip install . --verbose
```
