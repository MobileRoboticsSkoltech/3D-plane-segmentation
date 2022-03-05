FROM ubuntu:latest

ARG RELISE_VERSION=1.12.1

MAINTAINER pavelmokeev

RUN apt-get update && \
    apt-get install -y \
      gcc \
      mono-mcs \
      libboost-all-dev \
      libeigen3-dev \
      libflann-dev \
      libvtk6-dev \
      python3-vtk7 \
      libusb-1.0-0-dev \
      wget \
      cmake \
      python3 \
      python3-pip \
      libglu1-mesa-dev freeglut3-dev mesa-common-dev

WORKDIR /src/build

RUN wget -qO- https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${RELISE_VERSION}.tar.gz | tar xz \
 && cd pcl-pcl-${RELISE_VERSION} \
 && mkdir build \
 && cd build \
 && cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make -j2 install \
 && cd ../.. /