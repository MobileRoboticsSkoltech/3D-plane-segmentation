FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y cmake pkg-config libeigen3-dev libopencv-dev libpcl-dev

RUN apt-get update && apt-get install -y python3-pip git

RUN python3 -m pip install --user git+https://github.com/DanielPollithy/pypcd.git

ADD ./cpp /app/src/cpp

ADD ./include /app/src/include

WORKDIR /app/build

RUN cmake ../src/cpp && make

ADD scripts/ .

ENTRYPOINT ["bash", "run.sh"]