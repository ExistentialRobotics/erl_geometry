cmake_minimum_required(VERSION 3.24)

link_libraries(${PROJECT_NAME}::${PROJECT_NAME} GTest::gtest_main)

add_executable(test_incremental_quadtree test_incremental_quadtree.cpp)
add_test(NAME test_incremental_quadtree
        COMMAND test_incremental_quadtree)

add_executable(test_marching_square test_marching_square.cpp)
add_test(NAME test_marching_square
        COMMAND test_marching_square)

add_executable(test_euler_angle test_euler_angle.cpp)
add_test(NAME test_euler_angle
        COMMAND test_euler_angle)

add_executable(test_intersection_between_ray_and_aabb2d test_intersection_between_ray_and_aabb2d.cpp)
add_test(NAME test_intersection_between_ray_and_aabb2d
        COMMAND test_intersection_between_ray_and_aabb2d)

add_executable(test_lidar_area test_lidar_area.cpp)
add_test(NAME test_lidar_area
        COMMAND test_lidar_area)

add_executable(test_quadtree gtest/test_occupancy_quadtree.cpp)
add_test(NAME test_quadtree
        COMMAND test_quadtree
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
gtest_discover_tests(test_quadtree)

add_executable(test_quadtree_find_neighbors gtest/test_occupancy_quadtree_find_neighbors.cpp)
add_test(NAME test_quadtree_find_neighbors
        COMMAND test_quadtree_find_neighbors
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

add_executable(test_quadtree_ray_casting gtest/test_occupancy_quadtree_ray_casting.cpp)
add_test(NAME test_quadtree_ray_casting
        COMMAND test_quadtree_ray_casting
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

add_executable(test_quadtree_ray_tracing gtest/test_occupancy_quadtree_occupied_leaf_on_ray.cpp)
add_test(NAME test_quadtree_ray_tracing
        COMMAND test_quadtree_ray_tracing --tree-bt-file gtest/house_expo_room_1451.bt --resolution 0.01
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

add_executable(test_quadtree_leaf_of_node gtest/test_occupancy_quadtree_leaf_of_node.cpp)
add_test(NAME test_quadtree_leaf_of_node
        COMMAND test_quadtree_leaf_of_node
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
