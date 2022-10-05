#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
docker build --build-arg UID="$uid" --build-arg GID="$gid" -t coordinator/ros:foxy .

echo "Run Container"
xhost + local:root
docker run --name coordinator -v $PWD/src:/home/docker/ros2_ws/src/behavior-tree-coordinator -it --privileged --net host -e DISPLAY=$DISPLAY --rm coordinator/ros:foxy
