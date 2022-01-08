#!/bin/bash

source args.sh

xhost +

docker start $CONTAINER_INTEGRATED_QTCREATOR --attach --interactive