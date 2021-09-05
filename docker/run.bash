#!/usr/bin/env bash

IMAGE_NAME="hiltislamchallenge2021"

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

LOCAL_REPO_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/../" >/dev/null 2>&1 && pwd)"

USERID=$(id -u)
GROUPID=$(id -g)

docker run -it \
    --mount type=bind,source=${LOCAL_REPO_PATH}/src,target=/home/developer/ws/src/project \
    --mount type=bind,source=${LOCAL_REPO_PATH}/datasets,target=/home/developer/datasets \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    --network host \
    --privileged \
    --rm \
    --security-opt seccomp=unconfined \
    -u $USERID:$GROUPID \
    $IMAGE_NAME
