#!/bin/bash

source args.sh

xhost +

docker start $CONTAINER_NVIDIA_SHELL --attach --interactive