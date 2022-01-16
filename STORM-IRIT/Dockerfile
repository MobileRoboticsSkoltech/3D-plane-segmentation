FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev \
libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev \
libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev unzip libgtk-3-dev libeigen3-dev \
x11-xserver-utils libcanberra-gtk3-module graphviz libpcl-dev libcgal-dev

ADD ./CGAL /root/CGAL

WORKDIR /root/CGAL/build

RUN cmake .. && make

ADD ./Plane-Detection-Point-Cloud /app/src

WORKDIR /app/build

RUN cmake -DCMAKE_BUILD_TYPE=Release ../src && make