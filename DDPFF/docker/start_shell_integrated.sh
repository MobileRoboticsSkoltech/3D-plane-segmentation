#!/bin/bash

source args.sh

xhost +

docker start $CONTAINER_INTEGRATED_SHELL --attach --interactive