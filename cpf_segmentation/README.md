# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory. But before running dockerfile you have to change the location of the some folders and files. The script `runbeforedocker.sh` does everything for you. Then build the docker image:
```
docker build -t cpf_segmentation:1.0 .
```

Then finally after creating docker image run following command:

```sudo docker run --rm -ti --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind cpf_segmentation:1.0```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. For example:
```
sudo docker run --rm -ti --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/home/adminlinux/cpf_segmentation/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cpf_segmentation/output,target=/app/build/output,type=bind cpf_segmentation:1.0
```
2) Or you can run ready docker image `tojiboyevf/cpf_segmentation:1.0`

If there is a problem with displaying result in docker image like `Gtk-WARNING **: 21:16:20.033: cannot open display: :0` or other errors that related with displaying, then try to run command `xhost +` before running docker image. After finishing all stuff don't forget to run `xhost -`.

## Testing repo:

If you are using docker image by default the path of working directory will be in `build` folder.
You can use `.ply` or `.pcd` formats as a dataset format. To test the library, use following general command:

```
pathOfBuildDirectory/segmentation_test pathOfDataset/data.pcd -o outputFolderPath/nameOfOutputfile
```
For example, in docker image you can run:
```
./segmentation_test input/0000.pcd -o output/0000_result
```
`-o` is response to save the result to indicated path after it. You can omit it if you are not going to save the result.  
The name of output file starts with `0000_result` and you can find it in `output` folder. Be sure you pasted dataset in `input` folder.
There is an additonal parameter `-novis`. If you use it then it doesn't visualize the result. For example:
```
./segmentation_test input/0000.pcd -o output/result -novis
```

The original repo: https://github.com/MarkusEich/cpf_segmentation
