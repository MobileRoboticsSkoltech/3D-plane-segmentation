# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory:
```
docker build -t peac:1.0 .
```
2) Or you can run ready docker image `tojiboyevf/peac:1.0`

Then finally after building docker image run following command:

```
sudo docker run --rm --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind peac:1.0 filename
```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. `filename ` is the file you are going to test. For example:
```
sudo docker run --rm --mount src=/home/adminlinux/peac/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/peac/output,target=/app/build/output,type=bind peac:1.0 0000_s.pcd
```

[`plane_flitter_pcd.ini`](https://github.com/tojiboyevf/point-cloud-plane-extraction/blob/peac/peac/cpp/plane_fitter_pcd.ini) file contains paramters about camera and so on and when you run docker it automatically reads this file. If you want to use another `.ini` file then after filename show it:

```
sudo docker run --rm --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind peac:1.0 filename iniFileName
```

For example:

```
sudo docker run --rm --mount src=/home/adminlinux/peac/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/peac/output,target=/app/build/output,type=bind peac:1.0 0000_s.pcd another_plane_flitter_pcd.ini
```


<span style="color:red"> **Note that an organized (structured) data is required** </span>. You can use  `.pcd` format as a dataset format.

The original repo: https://github.com/ai4ce/peac

