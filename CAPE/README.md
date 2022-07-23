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

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result.
`/pathOfInputFolder` must also contain the intrinsics: `calib_params.xml`.

For example:

```
sudo docker run --rm  --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 
```

If you want to use non-default parameters for the algorithm, then you should add `params.ini` to `/pathOfInput` folder and specify both the input folder path and the parameters file path:
```
sudo docker run --rm  --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 input input/params.ini
```

By default, the algorithm only saves the tables of labels in the `/pathOfOutputFolder`. 
If you want to save the visualization images that the algorithm generates, then add parameter `--save-img` after specifying the input folder and the parameters file. They will be also saved in `/pathOfOutputFolder`:
```
sudo docker run --rm   --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 input input/params.ini --save_img
```

If you want to see real-time visualization then add parameter `--vis` at the end after specifying the input folder and the parameters file. You have to give permission so run command first `xhost +` and add other parameters in docker run command:
```
xhost +

sudo docker run --rm  --privileged --net=host --ipc=host    -e DISPLAY=$DISPLAY    -v /tmp/.X11-unix,/tmp/.X11-unix  --mount src=/home/adminlinux/cape/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/cape/output,target=/app/build/output,type=bind cape:1.0 input input/params.ini --vis
```



You can use depth image with `.png` format as a dataset. But images should start with `depth_` and in `calib_params.xml` file you have to write the calibration of IR(infrared) and RGB cameras. Here is the example [link](https://github.com/tojiboyevf/CAPE/tree/master/input). 

The original repo: https://github.com/pedropro/CAPE

## Example of `params.ini`:
```ini
[Params]
depthSigmaCoeff=0.000001425
depthSigmaMargin=10
cylinderScoreMin=100
# 0.15^2
cylinderRansacSqrMaxDist=0.0225
# cos(pi/12)
cosAngleMax=0.9659258262890683
maxMergeDist=50.0
patchSize=20
minNrOfValidPointsOnePerXThreshold=2
planesegMaxDiff=150
planarFittingJumpsCounterThresholdParam=1
histogramBinsPerCoordParam=20
regionGrowingCandidateSizeThresholdParam=5
regionGrowingCellsActivatedThresholdParam=4
regionPlanarFittingPlanarityScoreThresholdParam=100
cylinderDetectionCellsActivatedThreshold=5
refinementMultiplierParam=9

```