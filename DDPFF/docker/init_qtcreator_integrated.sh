#!/bin/bash

source args.sh

xhost +
docker run -it \
	--name $CONTAINER_INTEGRATED_QTCREATOR \
	--privileged \
	--net=host \
    -u $USER:$USER \
    -e DISPLAY=unix$DISPLAY \
    -e LANG=C.UTF-8 \
    -e LC_ALL=C.UTF-8 \
	-e QT_X11_NO_MITSHM=1 \
    -e QT_PLUGIN_PATH=/opt/Qt/5.15.2/gcc_64/plugins \
    -e PATH=/opt/Qt/5.15.2/gcc_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    -e QML_IMPORT_PATH=/opt/Qt/5.15.2/gcc_64/qml \
    -e QML2_IMPORT_PATH=/opt/Qt/5.15.2/gcc_64/qml \
    -e PATH=/opt/Qt/Tools/QtCreator/bin/:/opt/Qt/5.15.2/gcc_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
	--device=/dev/dri:/dev/dri \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$SOURCELOC:/home/$USER/$REPONAME" \
    --volume="$DATA:/home/$USER/data" \
    $IMAGE_INTEGRATED \
    qtcreator