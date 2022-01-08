#!/bin/bash

source args.sh

xhost +
docker run -it \
	--name $CONTAINER_INTEGRATED_SHELL \
	--net=host \
    -u $USER:$USER \
    -e DISPLAY=unix$DISPLAY \
    -e LANG=C.UTF-8 \
    -e LC_ALL=C.UTF-8 \
    --device=/dev/dri:/dev/dri \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$SOURCELOC:/home/$USER/$REPONAME" \
    $IMAGE_INTEGRATED