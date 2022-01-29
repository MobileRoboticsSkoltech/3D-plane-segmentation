FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y cmake pkg-config libeigen3-dev libopencv-dev libpcl-dev

WORKDIR /app/src/cpp

ADD ./cpp .

WORKDIR /app/src/include

ADD ./include .

WORKDIR /app/build

RUN cmake ../src/cpp && make

ENTRYPOINT ["./plane_fitter_pcd"]