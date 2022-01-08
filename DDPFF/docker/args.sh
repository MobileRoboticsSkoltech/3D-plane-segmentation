#!/bin/bash

REPONAME=DDPFF
IMAGE_NVIDIA=dockedqt-nvidia2
CONTAINER_NVIDIA_QTCREATOR="qtcreator-nvidia-$REPONAME"
CONTAINER_NVIDIA_SHELL="shell-nvidia-$REPONAME"
IMAGE_INTEGRATED=arindamrc/ddpffenv-integrated
CONTAINER_INTEGRATED_QTCREATOR="qtcreator-integrated-$REPONAME"
CONTAINER_INTEGRATED_SHELL="shell-integrated-$REPONAME"
SOURCELOC="/home/arc/Sources/$REPONAME"
DATA=/home/arc/Datasets/pointclouds

