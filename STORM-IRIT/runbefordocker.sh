#!/bin/bash

set -ex

mkdir Plane-Detection-Point-Cloud input output

cp -fR app src CMakeLists.txt Plane-Detection-Point-Cloud/

rm -r app src CMakeLists.txt

wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0.2/CGAL-5.0.2.zip

unzip CGAL-5.0.2.zip

rm CGAL-5.0.2.zip

mv CGAL-5.0.2 CGAL
