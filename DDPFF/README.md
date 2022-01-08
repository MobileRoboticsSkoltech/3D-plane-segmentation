# DDPFF
-------------

Accompanying code for the paper **Plane Segmentation Using Depth-Dependent Flood Fill** [(pdf)](https://www.hrl.uni-bonn.de/publications/roychoudhury21iros.pdf) by Arindam Roychoudhury, Marcell Missura and Maren Bennewitz accepted at IROS, 2021.

The source files containing the algorithm are [DDPFF.h](rep/include/rep/DDPFF.h) and [DDPFF.cpp](rep/src/DDPFF.cpp).


## Build and execution intructions


1. Create a build folder (in source or out of source). Here we assume we create a build folder within the source directory:
    `$mkdir build`
2. Navigate to the build folder:
    `$cd build`
3. Execute cmake:
    `$cmake ..`
4. Execute the application:
    `$./app/App`


## Application usage guide


A window appears when the application is executed. 

| Function | Navigation | Shortcut |
--- | --- | ---
Load point cloud frames | File &rarr; Load state | Ctrl+L
Navigate between frames | `\|<`, `<`, `>`, `>\|` buttons| Right and left arrow keys
Configuration |  View &rarr; Config | C
Toggle point cloud | View &rarr; Point Cloud | P
Activate DDPFF algorithm | View &rarr; Depth dependent planar segments | Ctrl+D
Activate algorithm |  View &rarr; Show Visualization | V
Activate merged planar segments | View &rarr; Show merged segments | Shift +U
Activate unmerged planar segments | View &rarr; Show unmerged segments | U
Show normals | View &rarr; Show plane normals | N
Activate camera view | View &rarr; Camera view | G
Activate ground truth planes | View &rarr; Show ground truth planes | Ctrl+G
Load Artificial Scenes | File &rarr; Load Scene | Ctrl+Shift+L
Choose scene | Scenes &rarr; Choose one of 6 scenes | 

The camera position in the artificial scenes can be modified by updating the scene parameters in the *config* widget. They can also be adjusted using the arrow keys and the *Ctrl* and *Shift* modifiers. Please select *Scenes &rarr; Adjust Scene* first to enable scene parameter updates using the arrow keys.
 

## Parameter symbol-name mappings

| Parameter symbol in paper | Parameter name in application config |
--- | --- 
&kappa;<sub>point</sub> | `c_point`
&gamma;<sub>point</sub> | `pointThreshold_min`
&epsilon;<sub>point</sub> | `pointThreshold_max`
&kappa;<sub>flood</sub> | `c_plane`
&gamma;<sub>flood</sub> | `planeThreshold_flood`
&epsilon;<sub>flood</sub> | `planeThreshold_flood_max`
&kappa;<sub>merge</sub> | `c_plane_merge`
&gamma;<sub>merge</sub> | `planeThreshold_merge`
&epsilon;<sub>merge</sub> | `planeThreshold_merge_max`
&kappa;<sub>angle</sub> | `c_angle`
&gamma;<sub>angle</sub> | `angleThresholdFloodFill`
&epsilon;<sub>angle</sub> | `angleThresholdFloodFill_max`
&kappa;<sub>&rho;</sub> | `c_range`
&gamma;<sub>&rho;</sub> | `normalSampleDistance_min`
&epsilon;<sub>&rho;</sub> | `normalSampleDistance_max`

The values of the parameters can be found in the paper.


## Software requirements


1. C++ 17
2. Cmake 3.8 or greater
3. Qt 5.15
4. Eigen 3.3
5. OpenGL
6. QGLViewer
7. OpenCV 4
8. PCL 1.10


## Using Docker

It is easiest to set up the development environment using docker. An image with the complete list of required softwares is available at [dockerhub](https://hub.docker.com/repository/docker/arindamrc/ddpffenv-integrated). 

First, update the location of the source directory in the file `docker/args.sh`. Change the variable `SOURCELOC` to point to the source local directory of the repository. 

Now, to create the development environment simply navigate to the directory "docker" and execute `$./init_qtcreator_integrated.sh`. This sets up a container with all the requisite softwares as well as launches the qtcreator IDE. This container shares the source directory and the `data` directory with the host system. Import the project into the IDE, build and execute it. To launch qtcreator subsequently use `./start_qtcreator_integrated`.


## Datasets

The application supports the following datasets:


| Dataset | URL | Instructions |
--- | --- | ---
Kinect dataset[1] | http://www.ais.uni-bonn.de/download/segmentation/kinect.zip | Rename the archive to *kinect.pcd.gt.zip* and load. 
SEGCOMP ABW dataset[2] | ftp://figment.csee.usf.edu/pub/segmentation-comparison/ABW-TEST-IMAGES.tar | Navigate to `globals/include/globals/constants.h` and change `IMAGE_WIDTH` and `IMAGE_HEIGHT` to 512. Recompile. Rename the archive to *ABW-TEST-IMAGES.abw.tar* and load.
TUM RGB-D Slam dataset[3] | https://vision.in.tum.de/data/datasets/rgbd-dataset/download | Each sequence which is a *\*.tgz* file can be loaded individually.
Point clouds collected using Asus Xtion Pro Live | https://drive.google.com/file/d/1qqIUQoXyZsv3jmwHhzcJK8Msy8loSWrN/view?usp=sharing | Uncompress and load individual files.

Please use `conf/configABW.conf`  when using the SEGCOMP ABW dataset. For the other datasets either `conf/config5.conf` (for higher accuracy) or `conf/config10.conf` (for higher performance) would work. Please rename each configuration files to `conf/config.conf` before using them.


## References

[1]: Bastian Oehler, Joerg Stueckler, Jochen Welle, Dirk Schulz, and Sven Behnke. "Efficient Multi-Resolution Plane Segmentation of 3D Point Clouds" Proceedings of the 4th International Conference on Intelligent Robotics and Applications (ICIRA), Aachen, December 2011

[2]: A. Hoover, G. Jean-Baptiste, X. Jiang, P. J. Flynn, H. Bunke, D. B. Goldgof, K. Bowyer, D. W. Eggert, A. Fitzgibbon, and R. B. Fisher, "An experimental comparison of range image segmentation algorithms," IEEE transactions on pattern analysis and machine intelligence, vol. 18, no. 7, pp. 673–689, 1996.

[3]: J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers, A Benchmark for the Evaluation of RGB-D SLAM Systems, Proc. of the International Conference on Intelligent Robot Systems (IROS), Oct. 2012.

