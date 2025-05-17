#!/usr/bin/env bash

set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
docker build --rm -t erl/geometry:20.04 . \
  --build-arg BASE_IMAGE=erl/common:20.04

docker build --rm -t erl/geometry:20.04-cu12.9 . \
  --build-arg BASE_IMAGE=erl/common:20.04-cu12.9 $@
