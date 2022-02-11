# Instruction
## Building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory:
```
docker build -t ddpff:1.0 .
```

2) Or you can run ready docker image `tojiboyevf/ddpff:1.0`

Then finally after building docker image run following command:

```
sudo docker run --rm  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind ddpff:1.0 filename
```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. `filename ` is the file you are going to test. For example:
```
sudo docker run --rm --mount src=/home/adminlinux/DDPFFAdoptation/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/DDPFFAdoptation/output,target=/app/build/output,type=bind ddpff:1.0 0000.pcd
```

You can use `.ply` or `.pcd` formats as a dataset format.

The original repo: https://github.com/arindamrc/DDPFF