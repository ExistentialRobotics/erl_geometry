if (ROS1_ACTIVATED)
    # library: erl_geometry_rviz_plugin
    file(GLOB RVIZ_PLUGIN_INCLUDE_FILES ../../include/erl_geometry/rviz_plugin/*.hpp)
    file(GLOB RVIZ_PLUGIN_SRC_FILES rviz_plugin/*.cpp)
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
    add_executable(gazebo_room_2d_node ros1/gazebo_room_2d_node.cpp)
    target_include_directories(gazebo_room_2d_node PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(gazebo_room_2d_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})

    # node: point_cloud_node
    add_executable(point_cloud_node ros1/point_cloud_node.cpp)
    target_include_directories(point_cloud_node PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(point_cloud_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})

elseif (ROS2_ACTIVATED)

endif ()
