# octomap library is used by test only
add_library(octomap SHARED
        src/math/Pose6D.cpp
        src/math/Quaternion.cpp
        src/math/Vector3.cpp
        src/AbstractOccupancyOcTree.cpp
        src/AbstractOcTree.cpp
        src/ColorOcTree.cpp
        src/CountingOcTree.cpp
        src/OcTree.cpp
        src/OcTreeNode.cpp
        src/OcTreeStamped.cpp
        src/Pointcloud.cpp
        src/ScanGraph.cpp
)
target_include_directories(octomap PUBLIC include)
