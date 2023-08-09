`erl_geometry`
==============
This CMake project provides `erl_geometry` which is a C++ library for geometry processing.

- [Axis Aligned Bounding Box (AABB)](include/erl_geometry/aabb.hpp): Axis Aligned Bounding Box (AABB), derived from `Eigen::AlignedBox`.
- [KdTree](include/erl_geometry/kdtree_eigen_adaptor.hpp): create kd-tree from Eigen matrix.
- [Incremental Quadtree](include/erl_geometry/incremental_quadtree.hpp): support incremental construction, keep the 
  tree depth as shallow as possible, i.e. deeper depth is created only when an inserted node is very close to existing 
  nodes in the tree. Allow custom data storage:
  - [Node](include/erl_geometry/node.hpp): node in the quadtree.
  - [Node Container](include/erl_geometry/node_container.hpp): abstract class of container for nodes.
  - [Node Container of Single Type](include/erl_geometry/node_container_single_type.hpp): container for nodes of single type.
  - [Node Container of Multiple Types](include/erl_geometry/node_container_multi_types.hpp): container for nodes of multiple types.
- Occupancy Quadtree: developed based on [Octomap](https://octomap.github.io/)
  - [Occupancy Quadtree](include/erl_geometry/occupancy_quadtree.hpp): implementation of occupancy quadtree.
  - [Abstract Quadtree](include/erl_geometry/abstract_quadtree.hpp): abstract class for quadtree.
  - [Quadtree Implementation](include/erl_geometry/quadtree_impl.hpp): implementation of quadtree, unlike incremental
    quadtree, this one always inserts nodes to the deepest level.
  - [Abstract Occupancy Quadtree](include/erl_geometry/abstract_occupancy_quadtree.hpp): abstract class for occupancy quadtree.
  - [QuadtreeKey](include/erl_geometry/quadtree_key.hpp): key for quadtree node to achieve fast lookup.
  - [Abstract Quadtree Node](include/erl_geometry/abstract_quadtree_node.hpp): abstract class for quadtree node.
  - [Quadtree Node](include/erl_geometry/quadtree_node.hpp): template implementation of quadtree node with data type T.
  - [Occupancy Quadtree Node](include/erl_geometry/occupancy_quadtree_node.hpp): implementation of quadtree node with occupancy data.
  - [LogOdd](include/erl_geometry/logodd.hpp): implementation of log odd.
  - [Abstract Quadtree Drawer](include/erl_geometry/abstract_quadtree_drawer.hpp): abstract class for quadtree drawer.
  - [Occupancy Quadtree Drawer](include/erl_geometry/occupancy_quadtree_drawer.hpp): visualization of occupancy quadtree.
- [Log Odd Map 2D](include/erl_geometry/log_odd_map_2d.hpp): 2D occupancy grid map based on log odd.
- Collision Detection
  - [Winding Number](include/erl_geometry/winding_number.hpp): check if a point is in a polygon.
  - [Grid Collision Check 2D](include/erl_geometry/grid_collision_checker_se2.hpp): check if a shape hits obstacles.
  - [Grid Collision Check 3D](include/erl_geometry/grid_collision_checker_3d.hpp): check if a shape hits obstacles.
  - [Point Collision Checker](include/erl_geometry/point_collision_checker.hpp): check if a point is in obstacle.
- Surface Extraction
  - [Marching Square](include/erl_geometry/marching_square.hpp): extract surface from 2D scalar field.
- Ray Tracing
  - [Bresenham 2D](include/erl_geometry/bresenham_2d.hpp): 2D ray tracing.
  - [Ray Marching](include/erl_geometry/ray_marching.hpp): 2D ray marching with SDF.
  - Also available in quadtree.
- 2D Point Cloud Processing
  - [Surface 2D](include/erl_geometry/surface_2d.hpp): data structure to store 2D surface consisting of points and normals.
  - [Space 2D](include/erl_geometry/space_2d.hpp): algorithm to compute SDF, SDDF and normals.
  - [Lidar 2D](include/erl_geometry/lidar_2d.hpp): generate 2D lidar scan with a given space.
  - [Lidar 2D Frame](include/erl_geometry/lidar_2d_frame.hpp): data structure to store 2D lidar scan.
- Datasets
  - [HouseExpo](python/erl_geometry/house_expo/README.md)
  - [GazeboSequence](python/erl_geometry/gazebo/sequence.py)
- Others
  - [Compute Intersections](include/erl_geometry/utils.hpp)
  - [Euler Angle](include/erl_geometry/euler_angle.hpp)
  - [Azimuth Elevation](include/erl_geometry/azimuth_elevation.hpp)
