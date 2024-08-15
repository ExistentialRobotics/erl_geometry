# 2024-08-15

- Fix: a bug in factory pattern of Quadtree and Octree that causes loading tree from file failed.
- Implement: intersection computation between various geometric primitives.
- Implement: helper function to create an ellipsoid with Open3D.
- Update: Python binding.

# 2024-07-29

- Add `SearchNode`, `GetLeafIterator`, `GetLeafInAabbIterator`, `GetTreeIterator`, `GetTreeInAabbIterator`
  and `GetNodeOnRayIterator` to `AbstractQuadtree` and `AbstractOctree`.

# 2024-07-25

- Update python binding: `SurfaceMapping[Quadtree|Octree]Node` should be inherited
  from `Occupancy[Quadtree|Octree]Node`.

# 2024-07-22

- Use Tracy to profile memory allocation and deallocation in Quadtree and Octree
- Add dataset CowAndLady
- Add dataset UcsdFah2D
- Add dataset GazeboRoom2D
- Add dataset HouseExpoMapLidar2D
- Make Open3dVisualizerWrapper support reopening the window
- Add IO support to DepthFrame3D, LidarFrame2D and LidarFrame3D
- Add SurfaceMapping interface
- Add SurfaceMapping Quadtree & Octree
- Update Python binding
