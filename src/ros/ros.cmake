if (ROS1_ACTIVATED)
    message(STATUS "ROS1 activated, building ROS1 stuff")

    # library: erl_geometry_rviz_plugin
    file(GLOB RVIZ_PLUGIN_INCLUDE_FILES include/erl_geometry/ros/ros1/rviz_plugin/*.hpp)
    file(GLOB RVIZ_PLUGIN_SRC_FILES src/ros/ros1/rviz_plugin/*.cpp)
    add_library(${PROJECT_NAME}_rviz_plugin MODULE
            ${RVIZ_PLUGIN_INCLUDE_FILES} ${RVIZ_PLUGIN_SRC_FILES})
    erl_collect_targets(LIBRARIES ${PROJECT_NAME}_rviz_plugin)
    target_include_directories(${PROJECT_NAME}_rviz_plugin
            PRIVATE ${Qt5_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_rviz_plugin
            ${PROJECT_NAME} ${Qt5_LIBRARIES} ${OGRE_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME}_rviz_plugin PRIVATE QT_NO_KEYWORDS)
    set_target_properties(${PROJECT_NAME}_rviz_plugin PROPERTIES AUTOMOC ON)

    # Avoid OGRE deprecation warnings under C++17
    target_compile_options(${PROJECT_NAME}_rviz_plugin PUBLIC "-Wno-register")

    # node: gazebo_room_2d_node
    add_executable(gazebo_room_2d_node src/ros/ros1/gazebo_room_2d_node.cpp)
    erl_target_dependencies(gazebo_room_2d_node PRIVATE ${PROJECT_NAME})
    erl_collect_targets(EXECUTABLES gazebo_room_2d_node)

    # node: point_cloud_node
    add_executable(point_cloud_node src/ros/ros1/point_cloud_node.cpp)
    erl_target_dependencies(point_cloud_node PRIVATE ${PROJECT_NAME})
    erl_collect_targets(EXECUTABLES point_cloud_node)
elseif (ROS2_ACTIVATED)
    message(STATUS "ROS2 activated, building ROS2 stuff")

    # library: erl_geometry_rviz_plugin
    file(GLOB RVIZ_PLUGIN_INCLUDE_FILES include/erl_geometry/ros/ros2/rviz_plugin/*.hpp)
    file(GLOB RVIZ_PLUGIN_SRC_FILES src/ros/ros2/rviz_plugin/*.cpp)
    add_library(${PROJECT_NAME}_rviz_plugin MODULE
            ${RVIZ_PLUGIN_INCLUDE_FILES} ${RVIZ_PLUGIN_SRC_FILES})
    erl_collect_targets(LIBRARIES ${PROJECT_NAME}_rviz_plugin)
    erl_target_dependencies(${PROJECT_NAME}_rviz_plugin PRIVATE
        ${PROJECT_NAME} ${${PROJECT_NAME}_ros_interfaces_LIBRARIES} ${QT_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME}_rviz_plugin PRIVATE QT_NO_KEYWORDS)
    set_target_properties(${PROJECT_NAME}_rviz_plugin PROPERTIES AUTOMOC ON)

    # Avoid OGRE deprecation warnings under C++17
    target_compile_options(${PROJECT_NAME}_rviz_plugin PUBLIC "-Wno-register")

    # node: gazebo_room_2d_node
    add_executable(gazebo_room_2d_node src/ros/ros2/gazebo_room_2d_node.cpp)
    erl_target_dependencies(gazebo_room_2d_node PRIVATE ${PROJECT_NAME})
    erl_collect_targets(EXECUTABLES gazebo_room_2d_node)

    # node: point_cloud_node
    add_executable(point_cloud_node src/ros/ros2/point_cloud_node.cpp)
    erl_target_dependencies(point_cloud_node PRIVATE ${PROJECT_NAME})
    erl_collect_targets(EXECUTABLES point_cloud_node)
else ()
    message(STATUS "No ROS version activated, skipping ROS stuff")
endif ()
