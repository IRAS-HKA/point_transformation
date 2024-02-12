#!/bin/sh

xhost + local:root

docker run \
       --name point_transformation \
       -it \
       -e DISPLAY=$DISPLAY \
       --env-file .env \
       --net host \
       --rm \
       point_transformation/ros:humble
