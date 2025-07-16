#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
BUILD_UBUNTU_2004=${BUILD_UBUNTU_2004:-true}
BUILD_ROS_NOETIC=${BUILD_ROS_NOETIC:-false}
BUILD_UBUNTU_2204=${BUILD_UBUNTU_2204:-true}
BUILD_ROS_HUMBLE=${BUILD_ROS_HUMBLE:-false}

if [ "${BUILD_UBUNTU_2004}" = "true" ]; then
  echo "Building Ubuntu 20.04 image..."
  cd $SCRIPT_DIR/ubuntu-2004
  ./build.bash $@
fi

if [ "${BUILD_ROS_NOETIC}" = "true" ]; then
  echo "Building ROS image..."
  cd $SCRIPT_DIR/ros-noetic
  ./build.bash $@
fi

if [ "${BUILD_UBUNTU_2204}" = "true" ]; then
  echo "Building Ubuntu 22.04 image..."
  cd $SCRIPT_DIR/ubuntu-2204
  ./build.bash $@
fi

if [ "${BUILD_ROS_HUMBLE}" = "true" ]; then
  echo "Building ROS Humble image..."
  cd $SCRIPT_DIR/ros-humble
  ./build.bash $@
fi
