#!/bin/bash

set -ex

process(){
    base_name=$(basename -- "$1")
    filename="${base_name%.*}"
    if [ -d output/$filename ] 
    then
        rm -rf output/$filename
    fi
    mkdir output/$filename
    if [ "$#" -eq 2 ]; 
    then 
        ./plane_fitter_pcd $base_name input/$2
    else
        ./plane_fitter_pcd $base_name
    fi
    python3 store_colors.py output/$filename/"${filename}.pcd"
}

if [ "$#" -eq 1 ]; 
then 
    process $1
elif [ "$#" -eq 2 ]; 
then
    process $1 $2
else
    for f in /app/build/input/*; do
       process $f
    done
fi