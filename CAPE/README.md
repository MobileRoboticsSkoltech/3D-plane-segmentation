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

If you want to see visualization then add parameter `--vis` at the end. You have to give permission so run commad first `xhost +` and add other parameters in docker run command:
```
xhost +

sudo docker run --rm  --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 --vis
```

If you want to change pixel size and read files in another folder in input directory then you add pixel size and folder name after file name. For example:

```
sudo docker run --rm   --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 16 icm-nuim
```

If you want to see visualization in with another pixel size and another data in folder then run following command:

```
xhost +

sudo docker run --rm  --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 16 icm-nuim --vis
```


You can use depth image with `.png` format as a dataset. But images should start with `depth_` and in `calib_params.xml` file you have to write the calibration of IR(infrared) and RGB cameras. Here is the example [link](https://github.com/tojiboyevf/CAPE/tree/master/input). 

The original repo: https://github.com/pedropro/CAPE

