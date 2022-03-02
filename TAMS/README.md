# Instruction

## Building the docker image of repo:

You can build docker image using current `Dockerfile` in this directory:
```shell
docker build -t tams:1.0 .
```

Building the PCL library:
```shell
wget -qO- https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.12.1.tar.gz | tar xz \
cd pcl-pcl-1.12.1
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2 install
```