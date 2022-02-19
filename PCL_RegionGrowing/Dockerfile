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

ADD . /app/src

WORKDIR /app/build

RUN cmake ../src && make

RUN wget -qO- https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-${RELISE_VERSION}.tar.gz | tar xz \
 && cd pcl-pcl-${RELISE_VERSION} \
 && mkdir build \
 && cd build \
 && cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make -j2 install \
 && cd ../

ENTRYPOINT ["bash", "run.sh"]