FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y cmake pkg-config libeigen3-dev libopencv-dev libpcl-dev

ADD ./cpf_segmentation /app/src

WORKDIR /app/build

RUN cmake ../src && make && make install