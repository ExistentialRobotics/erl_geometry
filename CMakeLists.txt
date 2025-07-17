cmake_minimum_required(VERSION 3.16)

project(erl_geometry
        LANGUAGES CXX C
        VERSION 0.1.0
        DESCRIPTION "erl_geometry is a C++ library of geometry data structures, algorithms and datasets.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_common REQUIRED)
endif ()

erl_project_setup(
        ERL_PACKAGES erl_common erl_covariance
)
include(cmake/erl_geometry_dependencies.cmake)
erl_setup_ros(
        CATKIN_COMPONENTS
        roscpp std_msgs nav_msgs geometry_msgs sensor_msgs message_generation rviz tf2_ros

        CATKIN_DEPENDS
        roscpp std_msgs nav_msgs geometry_msgs sensor_msgs message_runtime rviz tf2_ros

        LIBRARIES
        ${PROJECT_NAME}

        MSG_FILES
        OccupancyTreeMsg.msg

        MSG_DEPENDENCIES
        std_msgs

        CFG_EXTRAS
        erl_geometry_dependencies.cmake

        ROS2_COMPONENTS
        rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros pluginlib
        rviz_common rviz_rendering rviz_default_plugins
)

# ###################################################################################################
# BUILD
# ###################################################################################################

# #######################
# define the libraries #
# #######################
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} src/sdf/mesh_sdf.cpp src/sdf/sdf_util.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
erl_collect_targets(LIBRARIES ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
erl_target_dependencies(
        ${PROJECT_NAME}
        PUBLIC nanoflann::nanoflann
        PUBLIC Qhull::qhullcpp Qhull::qhull_r Open3D::Open3D
        PUBLIC absl::flat_hash_map absl::flat_hash_set
)
if (ERL_USE_LIBTORCH)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${TORCH_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${TORCH_LIBRARIES})
endif ()

include(src/ros/ros.cmake)

# ##################
# Python bindings #
# ##################
erl_add_pybind_module(
        PYBIND_MODULE_NAME py${PROJECT_NAME}
        PYBIND_SRC_DIR ${${PROJECT_NAME}_PYTHON_BINDING_DIR}
        LIBRARIES ${PROJECT_NAME}
)
erl_add_python_package()

# ###################################################################################################
# INSTALL
# ###################################################################################################
erl_install(
        EXECUTABLES ${${PROJECT_NAME}_COLLECTED_EXECUTABLES}
        LIBRARIES ${${PROJECT_NAME}_COLLECTED_LIBRARIES}
        PYBIND_MODULES py${PROJECT_NAME}
        OTHER_FILES plugin_description.xml)

# ###################################################################################################
# TEST
# ###################################################################################################
erl_add_tests(
        LIBRARIES ${PROJECT_NAME}
        EXCLUDE_FROM_ALL
        test_depth_frame_3d_basic
        test_convex_hull
        test_convex_hull_3d_impls
        test_depth_frame_3d_basic
        test_hidden_point_removal
        test_lidar_frame_2d_basic
        test_lidar_frame_3d_basic
        test_occupancy_octree_build
        test_occupancy_octree_ray_casting
        test_occupancy_quadtree_batch_ray_casting
        test_occupancy_quadtree_build
        test_occupancy_quadtree_build_from_image
        test_occupancy_quadtree_find_neighbors
        test_occupancy_quadtree_house_expo
        test_occupancy_quadtree_leaf_in_aabb
        test_occupancy_quadtree_leaf_of_node
        test_occupancy_quadtree_leaf_on_ray
        test_occupancy_quadtree_ray_casting
)

# ###################################################################################################
# export variables
# ###################################################################################################
erl_mark_project_found()
