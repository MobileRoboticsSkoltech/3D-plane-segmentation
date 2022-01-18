# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory. But before running dockerfile you have to change the location of the some folders and files. The script `runbeforedocker.sh` does everything for you. Then build the docker image:
```
docker build -t peac:1.0 .
```

Then finally after creating docker image run following command:

```sudo docker run --rm -ti --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind peac:1.0```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. For example:
```
sudo docker run --rm -ti --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/home/adminlinux/peac/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/peac/output,target=/app/build/output,type=bind peac:1.0
```
2) Or you can run ready docker image `tojiboyevf/peac:1.0`

## Testing repo:

If you are using docker image by default the path of working directory will be in `build` folder.
You can use  `.pcd` format as a dataset format. First you have to write all the `.pcd` files into `lists.txt` and put it with `.pcd` files into `input` folder that you indicated before runnig docker image. Here is the example [link](https://github.com/ai4ce/peac/tree/master/data/andreashaus). To test the library, use following general command:

```
./plane_fitter_pcd
```
The result will be stored in `output` folder that you indicated before runnig docker image.

The original repo: https://github.com/ai4ce/peac
