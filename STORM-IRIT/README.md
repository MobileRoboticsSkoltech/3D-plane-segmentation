# Instruction
## Options of building the docker image of repo:

1) You can build docker image using current `Dockerfile` in this directory. But before running dockerfile you have to change the location of the some folders and files. The script `runbeforedocker.sh` does everything for you. Then build the docker image:
```
docker build -t pdpc:1.0 .
```

Then finally after creating docker image run following command:

```sudo docker run --rm -ti  --mount src=/pathOfInputFolder,target=/app/build/input,type=bind --mount src=/pathOfOutputFolder,target=/app/build/output,type=bind pdpc:1.0```

Here `src=/pathOfInputFolder` means the path of dataset that you are going to test and `src=/pathOfOutputFolder` means the path of output folder that you are going to store the result. For example:
```
sudo docker run --rm -ti --mount src=/home/adminlinux/STORM-IRIT/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/STORM-IRIT/output,target=/app/build/output,type=bind pdpc:1.0
```

2) Or you can run ready docker image `tojiboyevf/pdpc:1.0`

Authors didn't add visualization into their code.

## Testing repo:

If you are using docker image by default the path of working directory will be in `build` folder.

You can use `.ply` format as a dataset format. To test the library, use following general commands:

1) First preprocess the data using the next commands:

```
./pdpcComputeMultiScaleFeatures -i input/data.ply -o output/data_res -v
./pdpcSegmentation -i input/data.ply -s output/data_res_scales.txt -f output/data_res_features.txt -o output/data -v
```

The first program `pdpcComputeMultiScaleFeatures` computes surface curvatures and normals from the point cloud at multiple scales and stores them as `.txt` files in `output` folder: `data_res_scales.txt` and `data_res_features.txt`.

The second program `pdpcSegmentation` performs planar region growings at all scales and we get  `data_comp.txt` and `data_seg.txt` files.
Note that an oriented normal vector is required for each input point. 

2) After that create a new folder to store the final result:
```
mkdir output/res1
```
Finally, the program `pdpcPostProcess` can perform 3 different operations depending on the given options
```
./pdpcPostProcess -i input/data.ply -s output/data_seg.txt -c output/data_comp.txt -o output/res1/res1 -col -v -range 20 24 25 30 40 42
./pdpcPostProcess -i input/data.ply -s output/data_seg.txt -c output/data_comp.txt -o output/res2/res2 -col -v -pers 15 20 25
./pdpcPostProcess -i input/data.ply -s output/data_seg.txt -c output/data_comp.txt -o output/res3/res3 -col -v -scales 5 15 20 25
```
- `-range birth1 death1 birth2 death2` generates two files showing components that persist in the scale ranges (`birth1`,`death1`) and (`birth2`,`death2`)
- `-pers pers1 pers2` generates two files showing components that are more persistent than the persistence thresholds `pers1` and `pers2`
- `-scale scale1 scale2` generates two files showing the most persistent components that include the scale thresholds `scale1` and `scale2`

Results are generated as text files with one integer per line corresponding to one label per point (where `-1` means that the point is unlabeled). 
The option `-col` so that the colored PLY files are also generated. 
To modify some parameters please check the help of the programs by running them with the option `-h`.

The original repo: https://github.com/STORM-IRIT/Plane-Detection-Point-Cloud
