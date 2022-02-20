FROM gcc:latest as build

ARG RELISE_VERSION=1.12.1

RUN apt-get update && \
    apt-get install -y \
      libboost-all-dev \
      libeigen3-dev \
      libflann-dev \
      python3-vtk7 \
      libusb-1.0-0-dev \
      wget \
      cmake \
      python3-pip \
      python3.8

RUN pip3 install open3d

ADD . /app/build
WORKDIR /app/build

RUN wget -qO- https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-${RELISE_VERSION}.tar.gz | tar xz \
 && cd pcl-pcl-${RELISE_VERSION} \
 && mkdir build \
 && cd build \
 && cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make -j2 install \
 && cd ..\.. /

RUN mkdir build && cd build && cmake .. && cd ..

ENTRYPOINT ["python", "scripts/run.py"]