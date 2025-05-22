cmake_minimum_required(VERSION 3.16)

project(erl_geometry
        LANGUAGES CXX
        VERSION 0.1.0
        DESCRIPTION "erl_geometry is a C++ library of geometry data structures, algorithms and datasets.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_common REQUIRED)
endif ()
erl_project_setup(
        ${PROJECT_NAME}
        ERL_PACKAGES erl_common erl_covariance
        CATKIN_COMPONENTS roscpp std_msgs nav_msgs message_generation
        CATKIN_DEPENDS roscpp std_msgs nav_msgs message_runtime rviz
)

erl_config_qhull()
erl_config_absl()
erl_config_open3d()
if (ROS1_ACTIVATED)
    erl_config_qt5(Core Widgets)
    erl_config_ogre()
endif ()

if (ERL_BUILD_TEST_${PROJECT_NAME})
    add_subdirectory(deps/octomap)  # test_occupancy_octree_impls depends on octomap
    erl_config_cgal()
    if (NOT CGAL_FOUND)
        list(APPEND ignore_tests test_convex_hull_3d_impls.cpp)  # CGAL is not found, ignore the test
        message(WARNING "CGAL not found, ignoring test_convex_hull_3d_impls")
    endif ()
endif ()

erl_setup_ros(
        CATKIN_COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
        MSG_FILES OccupancyTreeMsg.msg
        MSG_DEPENDENCIES std_msgs
)
erl_catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} py${PROJECT_NAME}
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPENDS}
        DEPENDS ${${PROJECT_NAME}_DEPENDS})   # non-catkin dependencies are collected via erl_find_package

####################################################################################################
# BUILD
####################################################################################################

########################
# define the libraries #
########################

file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} src/sdf/mesh_sdf.cpp src/sdf/sdf_util.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
target_link_libraries(${PROJECT_NAME}
        PUBLIC nanoflann::nanoflann
        PUBLIC Qhull::qhullcpp Qhull::qhull_r Open3D::Open3D
        PUBLIC absl::flat_hash_map absl::flat_hash_set
)
set(libs ${PROJECT_NAME})
if (ROS1_ACTIVATED)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
    # library: erl_geometry_rviz_plugin
    add_library(${PROJECT_NAME}_rviz_plugin MODULE
            include/erl_geometry/rviz_plugin/occupancy_tree_grid_display.hpp
            include/erl_geometry/rviz_plugin/occupancy_tree_map_display.hpp
            src/rviz_plugin/occupancy_tree_grid_display.cpp
            src/rviz_plugin/occupancy_tree_map_display.cpp)
    target_include_directories(${PROJECT_NAME}_rviz_plugin
            PRIVATE ${Qt5_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_rviz_plugin
            ${PROJECT_NAME} ${Qt5_LIBRARIES} ${OGRE_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME}_rviz_plugin PRIVATE QT_NO_KEYWORDS)
    set_target_properties(${PROJECT_NAME}_rviz_plugin PROPERTIES AUTOMOC ON)
    # Avoid OGRE deprecation warnings under C++17
    target_compile_options(${PROJECT_NAME}_rviz_plugin PUBLIC "-Wno-register")
    list(APPEND libs ${PROJECT_NAME}_rviz_plugin)
else ()
    target_link_libraries(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_ERL_PACKAGES})
endif ()

###################
# Python bindings #
###################
erl_add_pybind_module(
        PYBIND_MODULE_NAME py${PROJECT_NAME}
        PYBIND_SRC_DIR ${${PROJECT_NAME}_PYTHON_BINDING_DIR}
        PYTHON_PKG_DIR ${${PROJECT_NAME}_PYTHON_DIR}/${PROJECT_NAME}
        LIBRARIES ${PROJECT_NAME}
)
erl_add_python_package(
        PYTHON_PKG_DIR ${${PROJECT_NAME}_PYTHON_DIR}/${PROJECT_NAME}
        DEPENDS_PYTHON_PKGS ${${PROJECT_NAME}_ERL_PACKAGES})

####################################################################################################
# INSTALL
####################################################################################################
erl_install(
        LIBRARIES ${libs}
        PYBIND_MODULES py${PROJECT_NAME}
        OTHER_FILES plugin_description.xml)

####################################################################################################
# TEST
####################################################################################################
erl_set_gtest_extra_libraries(test_convex_hull_3d_impls CGAL::CGAL)
erl_set_gtest_extra_libraries(test_occupancy_octree_impls octomap)
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
        IGNORE_FILES ${ignore_tests}
)
if (ROS1_ACTIVATED)
    add_executable(test_rviz_plugin_occupancy_tree_display
            test/gtest/ros1/test_rviz_plugin_occupancy_tree_display.cpp)
    target_link_libraries(test_rviz_plugin_occupancy_tree_display
            PUBLIC ${PROJECT_NAME}
            PUBLIC ${catkin_LIBRARIES})
endif ()

####################################################################################################
# export variables
####################################################################################################
erl_mark_project_found(${PROJECT_NAME})
