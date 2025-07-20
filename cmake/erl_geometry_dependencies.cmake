# if (NOT COMMAND erl_project_setup) find_package(erl_cmake_tools REQUIRED) endif ()

erl_config_nanoflann()
erl_config_qhull()
erl_config_absl()
erl_config_open3d()
if (ROS1_ACTIVATED)
    erl_config_qt5(Core Widgets)
    erl_config_ogre()
elseif (ROS2_ACTIVATED)
    erl_config_qt(Core Widgets) # Qt5 or Qt6
endif ()
