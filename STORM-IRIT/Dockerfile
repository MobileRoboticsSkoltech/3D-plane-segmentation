FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y cmake pkg-config  libcgal-dev

ADD ./CGAL /root/CGAL

WORKDIR /root/CGAL/build

RUN cmake .. && make

ADD ./Plane-Detection-Point-Cloud /app/src

WORKDIR /app/build

RUN cmake -DCMAKE_BUILD_TYPE=Release ../src && make