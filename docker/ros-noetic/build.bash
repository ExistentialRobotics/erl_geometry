#! /usr/bin/bash

docker build --rm -t erl/ros-noetic:cpu-geometry . \
  --build-arg BASE_IMAGE=erl/geometry:20.04 $@
