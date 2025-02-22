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
        ERL_PACKAGES erl_common)

erl_find_package(
        PACKAGE Qhull
        REQUIRED
        COMMANDS UBUNTU_LINUX "try `sudo apt-get install libqhull-dev`"
        COMMANDS ARCH_LINUX "try `sudo pacman -S qhull`")
get_target_property(Qhull_INCLUDE_DIRS Qhull::qhullcpp INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(qhull_r_location Qhull::qhull_r LOCATION)
get_target_property(qhullcpp_location Qhull::qhullcpp LOCATION)
list(APPEND Qhull_LIBRARIES ${qhull_r_location} ${qhullcpp_location})

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.24)
set(_GLOBAL_STRING "GLOBAL")
else()
# _GLOBAL_STRING is nothing
message(STATUS "CMake version is too low at ${CMAKE_VERSION}. Will still try, but may not work correctly.")
endif()

erl_find_package(
        PACKAGE absl
        REQUIRED ${_GLOBAL_STRING}
        COMMANDS APPLE "try `brew install abseil`"
        COMMANDS UBUNTU_LINUX "try `sudo apt install libabseil-dev`"
        COMMANDS ARCH_LINUX "try `sudo pacman -S abseil-cpp`")

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.24)
get_target_property(absl::flat_hash_map_location absl::flat_hash_map LOCATION)
get_target_property(absl::flat_hash_set_location absl::flat_hash_set LOCATION)
list(APPEND absl_LIBRARIES ${absl_flat_hash_map_location} ${absl_flat_hash_set_location})
endif()

erl_find_package(
        PACKAGE Open3D
        REQUIRED
        COMMANDS GENERAL "visit http://www.open3d.org/"
        COMMANDS ARCH_LINUX "try `paru -S open3d`"
)
get_target_property(Open3D_LIBRARIES Open3D::Open3D LOCATION)

if (ERL_BUILD_TEST_${PROJECT_NAME})

    add_library(octomap SHARED
            test/gtest/octomap/src/math/Pose6D.cpp
            test/gtest/octomap/src/math/Quaternion.cpp
            test/gtest/octomap/src/math/Vector3.cpp
            test/gtest/octomap/src/AbstractOccupancyOcTree.cpp
            test/gtest/octomap/src/AbstractOcTree.cpp
            test/gtest/octomap/src/ColorOcTree.cpp
            test/gtest/octomap/src/CountingOcTree.cpp
            test/gtest/octomap/src/OcTree.cpp
            test/gtest/octomap/src/OcTreeNode.cpp
            test/gtest/octomap/src/OcTreeStamped.cpp
            test/gtest/octomap/src/Pointcloud.cpp
            test/gtest/octomap/src/ScanGraph.cpp
    )
    target_include_directories(octomap PUBLIC test/gtest/octomap/include)

    set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
    erl_find_package(
            PACKAGE CGAL
            REQUIRED COMPONENTS Core
            COMMANDS UBUNTU_LINUX "try `sudo apt-get install libcgal-dev`"
            COMMANDS ARCH_LINUX "try `sudo pacman -S cgal`"
    )
endif ()

erl_setup_ros()
erl_catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} py${PROJECT_NAME}
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPENDS}
        DEPENDS ${${PROJECT_NAME}_DEPENDS})   # non-catkin dependencies are collected via erl_find_package

#######################################################################################################################
# BUILD
#######################################################################################################################

########################
# define the libraries #
########################
set(LIBMORTON_DIR ${CMAKE_CURRENT_LIST_DIR}/deps/libmorton)
if (EXISTS "${LIBMORTON_DIR}/CMakeLists.txt")
    message(STATUS "libmorton found in ${LIBMORTON_DIR}")
else ()
    message(FATAL_ERROR "libmorton not found, run `git submodule update --init --recursive`")
endif ()
add_subdirectory(${LIBMORTON_DIR})  # libmorton is a header-only library
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
        PUBLIC libmorton
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

#######################################################################################################################
# INSTALL
#######################################################################################################################
erl_install(
        LIBRARIES ${PROJECT_NAME}
        PYBIND_MODULES py${PROJECT_NAME})

#######################################################################################################################
# TEST
#######################################################################################################################
erl_add_tests(
        LIBRARIES ${PROJECT_NAME} CGAL::CGAL octomap
)

#######################################################################################################################
# export variables
#######################################################################################################################
erl_mark_project_found(${PROJECT_NAME})
