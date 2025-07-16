erl_config_nanoflann()
erl_config_qhull()
erl_config_absl()
erl_config_open3d()
if (ROS1_ACTIVATED)
    erl_config_qt5(Core Widgets)
    erl_config_ogre()
endif ()

if (ERL_BUILD_TEST_${PROJECT_NAME})
    add_subdirectory(deps/octomap) # test_occupancy_octree_impls depends on octomap
    erl_config_cgal()

    if (NOT CGAL_FOUND)
        erl_ignore_gtest(test_convex_hull_3d_impls.cpp)
        message(WARNING "CGAL not found, ignoring test_convex_hull_3d_impls")
    endif ()

    erl_set_gtest_extra_libraries(test_convex_hull_3d_impls CGAL::CGAL)
    erl_set_gtest_extra_libraries(test_occupancy_octree_impls octomap)
endif ()
