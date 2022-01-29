FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y cmake pkg-config  wget unzip


#CGAL installation:
RUN apt-get install -y libgmp-dev libmpfr-dev libboost-all-dev

WORKDIR /root

RUN wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0.2/CGAL-5.0.2.zip

RUN unzip CGAL-5.0.2.zip

RUN rm CGAL-5.0.2.zip

RUN mv CGAL-5.0.2 CGAL

WORKDIR /root/CGAL/build

RUN cmake .. && make

ENV CGAL_DIR "/root/CGAL"


WORKDIR /app/src/src

ADD ./src .

WORKDIR /app/src/app

ADD ./app .

ADD CMakeLists.txt /app/src

WORKDIR /app/build

RUN cmake -DCMAKE_BUILD_TYPE=Release ../src && make

ADD run.sh .

ENTRYPOINT ["bash", "run.sh"]