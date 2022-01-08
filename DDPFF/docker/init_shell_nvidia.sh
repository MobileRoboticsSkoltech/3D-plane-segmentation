#!/bin/bash

source args.sh

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

xhost +
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --runtime=nvidia \
    --name $CONTAINER_NVIDIA_SHELL \
    --net=host \
    -u $USER:$USER \
    -e LANG=C.UTF-8 \
    -e LC_ALL=C.UTF-8 \
    -e QT_X11_NO_MITSHM=1 \
    --volume="$SOURCELOC:/home/$USER/$REPONAME" \
    $IMAGE_NVIDIA