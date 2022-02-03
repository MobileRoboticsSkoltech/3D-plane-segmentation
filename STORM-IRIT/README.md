# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory:
```
docker build -t storm-irit:1.0 .
```

2) Or you can run ready docker image `tojiboyevf/storm-irit:1.0`

Then finally after building docker image run following command:

```
sudo docker run --rm  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind storm-irit:1.0 filename
```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. `filename ` is the file you are going to test. For example:
```
sudo docker run --rm --mount src=/home/adminlinux/STORM-IRIT/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/STORM-IRIT/output,target=/app/build/output,type=bind storm-irit:1.0 n_0000.ply
```

<span style="color:red"> **Note that an oriented normal vector is required for each input point** </span>. Authors used the normal estimation filter in Meshlab, using default parameters if point cloud doesn't have an oriented normal vector for each input point. You can use `.ply` or `.obj` format as dataset formats.

## Meshlab

First you have to install Meshlab https://www.meshlab.net

### Instruction for Ubuntu Linux

If you are using Linux then you can do this:
```
apt-get update
apt-get install meshlab
```

Then you can open your `.ply/.obj` file with Meshlab. Press right button on your file and select 'Open with Meshlab'. 

Then find 'Filters' in menu bar.
Then do following 'Filters->Normals, Curvatures and Orientation -> Compute normals for point sets'. Then opens another window with name 'Compute normals for point sets'. You can change parameters or leave it as default. After that press the button 'Apply'. 

Now you have to export file. First find 'File' in menu bar. Then do following 'File->Export Mesh As...' then choose the folder, show the file name and file type. STORM-IRIT works with `.ply/.obj` that's why choose one of these formats.

The original repo: https://github.com/STORM-IRIT/Plane-Detection-Point-Cloud

