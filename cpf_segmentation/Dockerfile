FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y cmake pkg-config libeigen3-dev libopencv-dev libpcl-dev

WORKDIR /app/src/src

ADD ./src .

WORKDIR /app/src/gco-v3.0

ADD ./gco-v3.0 .

WORKDIR /app/src/include

ADD ./include .

ADD CMakeLists.txt /app/src

WORKDIR /app/build

RUN cmake ../src && make && make install

ENTRYPOINT ["./segmentation_test"]