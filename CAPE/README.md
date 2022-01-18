# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory:
```
docker build -t cape:1.0 .
```

Then finally after creating docker image run following command:

```sudo docker run --rm -ti --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind cape:1.0```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. For example:
```
sudo docker run --rm -ti --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/home/adminlinux/cpf_segmentation/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cpf_segmentation/output,target=/app/build/output,type=bind cape:1.0
```
2) Or you can run ready docker image `tojiboyevf/cape:1.0`

## Testing repo:

If you are using docker image by default the path of working directory will be in `build` folder.
You can use depth image with `.png` format as a dataset. But images should start with `depth_` and in `calib_params.xml` file you have to write the calibration of IR(infrared) and RGB cameras. Here is the example [link](https://github.com/tojiboyevf/CAPE/tree/master/input). To test the library, use following general command:

```./cape_offline <cell_size> <sequence_name>```

where the first argument is the cell size in pixels (recommended 20)
and the second argument is a folder stored in:``./input`` that you indicated before runnig docker image.
that contains the image and calibration files. 
For example, if the target sequence is the 'icl-nuim' (assuming this was downloaded as specified above),run:

```./cape_offline 16 icl-nuim```
If you run just ``./cape_offline `` by default cell size in pixels is 16 and reads data directly from `input` folder that you indicated before runnig docker image.

The result will be stored in `output` folder that you indicated before runnig docker image.

The original repo: https://github.com/pedropro/CAPE
