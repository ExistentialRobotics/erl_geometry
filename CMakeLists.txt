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
        ERL_PACKAGES erl_common erl_covariance)

erl_config_qhull()
erl_config_absl()
erl_config_open3d()

if (ERL_BUILD_TEST_${PROJECT_NAME})
    add_subdirectory(deps/octomap)  # test_occupancy_octree_impls depends on octomap
    erl_config_cgal()
    if (NOT CGAL_FOUND)
        list(APPEND ignore_tests test_convex_hull_3d_impls.cpp)  # CGAL is not found, ignore the test
        message(WARNING "CGAL not found, ignoring test_convex_hull_3d_impls")
    endif ()
endif ()

erl_setup_ros()
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

file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
if (catkin_LIBRARIES)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
else ()
    target_link_libraries(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_ERL_PACKAGES})
endif ()
target_link_libraries(${PROJECT_NAME}
        PUBLIC nanoflann::nanoflann
        PUBLIC Qhull::qhullcpp Qhull::qhull_r Open3D::Open3D
        PUBLIC absl::flat_hash_map absl::flat_hash_set
)

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
        LIBRARIES ${PROJECT_NAME}
        PYBIND_MODULES py${PROJECT_NAME})

####################################################################################################
# TEST
####################################################################################################
erl_add_tests(
        test_convex_hull_3d_impls_LIBRARIES CGAL::CGAL
        test_occupancy_octree_impls_LIBRARIES octomap
        LIBRARIES ${PROJECT_NAME}
        EXCLUDE_FROM_ALL test_depth_frame_3d_basic
        test_convex_hull
        test_convex_hull_3d_impls
        test_depth_frame_3d_basic
        test_hidden_point_removal
        test_lidar_frame_2d_basic
        test_lidar_frame_3d_basic
        test_occupancy_octree_build
        test_occupancy_octree_ray_casting
        test_occupancy_quadtree_batch_ray_casting
        test_occupancy_quadtree_build_from_image
        test_occupancy_quadtree_find_neighbors
        test_occupancy_quadtree_house_expo
        test_occupancy_quadtree_leaf_in_aabb
        test_occupancy_quadtree_leaf_of_node
        test_occupancy_quadtree_leaf_of_ray
        test_occupancy_quadtree_ray_casting
        IGNORE_FILES ${ignore_tests}
)

####################################################################################################
# export variables
####################################################################################################
erl_mark_project_found(${PROJECT_NAME})
