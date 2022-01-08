#!/bin/bash

source args.sh

XAUTH=/home/arc/.docker.xauth
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
    --volume="/dev:/dev" \
    --volume="$SOURCELOC:/home/$USER/$REPONAME" \
    --volume="$DATA:/home/$USER/data" \
    --device-cgroup-rule='c 81:* rmw' \
    --runtime="nvidia" \
    --name="$CONTAINER_NVIDIA_QTCREATOR" \
    --net=host \
    --user="$USER:$USER" \
    --env="LANG=C.UTF-8" \
    --env="LC_ALL=C.UTF-8" \
    --env="QT_PLUGIN_PATH=/opt/Qt/5.15.2/gcc_64/plugins" \
    --env="PATH=/opt/Qt/5.15.2/gcc_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    --env="QML_IMPORT_PATH=/opt/Qt/5.15.2/gcc_64/qml" \
    --env="QML2_IMPORT_PATH=/opt/Qt/5.15.2/gcc_64/qml" \
    --env="PATH=/opt/Qt/Tools/QtCreator/bin/:/opt/Qt/5.15.2/gcc_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    --privileged \
    $IMAGE_NVIDIA \
    qtcreator