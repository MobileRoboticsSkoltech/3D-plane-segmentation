# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory:
```
docker build -t cape:1.0 .
```
2) Or you can run ready docker image `tojiboyevf/cape:1.0`

Then finally after building docker image run following command:

```
sudo docker run --rm  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind cape:1.0
```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. For example:

```
sudo docker run --rm   --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 
```

You can use depth image with `.png` format as a dataset. But images should start with `depth_` and in `calib_params.xml` file you have to write the calibration of IR(infrared) and RGB cameras. Here is the example [link](https://github.com/tojiboyevf/CAPE/tree/master/input). 

The original repo: https://github.com/pedropro/CAPE

