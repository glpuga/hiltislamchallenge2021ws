#!/usr/bin/env bash

IMAGE_NAME="hiltislamchallenge2021"

xhost +

CONTAINER_ID=$(docker ps -aqf "ancestor=${IMAGE_NAME}")

docker exec \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LINES=$(tput lines) \
    -it ${CONTAINER_ID} bash

xhost -
