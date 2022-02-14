FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

# Install Open3D system dependencies and pip
RUN apt-get update && apt-get install -y \
    libjpeg-dev \
    zlib1g-dev \
    libgl1 \
    libgomp1 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Open3D from the pypi repositories
RUN python3 -m pip install --no-cache-dir --upgrade open3d

#CGAL installation:
RUN apt-get update && apt-get install -y cmake pkg-config  wget unzip libgmp-dev libmpfr-dev libboost-all-dev

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

ADD scripts/ .

ENTRYPOINT ["bash", "run.sh"]