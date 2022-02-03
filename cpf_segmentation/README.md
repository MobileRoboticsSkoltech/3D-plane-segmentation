# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory:
```
docker build -t cpf_segmentation:1.0 .
```
2) Or you can run ready docker image `tojiboyevf/cpf_segmentation:1.0`

Then finally after building docker image run following command:

```
sudo docker run --rm  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind cpf_segmentation:1.0 filename
```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. `filename ` is the file you are going to test. For example:
```
sudo docker run --rm --mount src=/home/adminlinux/cpf_segmentation/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cpf_segmentation/output,target=/app/build/output,type=bind cpf_segmentation:1.0 0000.pcd
```

You can use `.ply` or `.pcd` formats as a dataset format. To test the library, use following general command:

The original repo: https://github.com/MarkusEich/cpf_segmentation

