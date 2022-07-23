FROM gcc:latest as build

RUN apt-get update && \
    apt-get install -y \
      libeigen3-dev \
      cmake

ADD . /app/src

WORKDIR /app/build

RUN cmake ../src && make

ENTRYPOINT ["./DDPFFAdoptation"]
