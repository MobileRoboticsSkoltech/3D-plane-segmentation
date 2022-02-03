FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y cmake pkg-config  wget unzip

#CGAL installation:
RUN apt-get install -y libgmp-dev libmpfr-dev libboost-all-dev

WORKDIR /root

RUN wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0.2/CGAL-5.0.2.zip \
    &&\
    unzip CGAL-5.0.2.zip \
    &&\
    rm CGAL-5.0.2.zip \
    &&\
    mv CGAL-5.0.2 CGAL

WORKDIR /root/CGAL/build

RUN cmake .. && make

ENV CGAL_DIR "/root/CGAL"

#Building STORM IRIT package
ADD ./src /app/src/src

ADD ./app /app/src/app

ADD CMakeLists.txt /app/src

WORKDIR /app/build

RUN cmake -DCMAKE_BUILD_TYPE=Release ../src && make

ADD run.sh .

ENTRYPOINT ["bash", "run.sh"]