cmake_minimum_required(VERSION 3.20)

project(erl_geometry
        LANGUAGES CXX
        VERSION 0.1.0
        DESCRIPTION "erl_geometry is a C++ library of geometry data structures and algorithms")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_common REQUIRED)
endif ()
set(${PROJECT_NAME}_CATKIN_COMPONENTS erl_common)
erl_project_setup()
erl_setup_ros()
erl_catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} py${PROJECT_NAME}
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
        DEPENDS ${${PROJECT_NAME}_DEPENDS})   # dependencies are collected via erl_find_package

#######################################################################################################################
# BUILD
#######################################################################################################################

########################
# define the libraries #
########################
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC erl_common ${catkin_LIBRARIES})

###################
# Python bindings #
###################
erl_add_python_package(
        PYTHON_PKG_DIR ${${PROJECT_NAME}_PYTHON_DIR}/${PROJECT_NAME}
        PYBIND_MODULE_NAME py${PROJECT_NAME}
        DEPENDS_PYTHON_PKGS erl_common)

#######################################################################################################################
# INSTALL
#######################################################################################################################
erl_install(
        LIBRARIES ${PROJECT_NAME}
        PYBIND_MODULES py${PROJECT_NAME})

#######################################################################################################################
# TEST
#######################################################################################################################
erl_add_tests()

#######################################################################################################################
# export variables
#######################################################################################################################
erl_mark_project_found()
