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
    ./segmentation_test $base_name
    python3 store_labels.py output/$filename/"${filename}_seg.pcd"
}

if [ "$#" -eq 1 ]; 
then 
    process $1
else
    for f in /app/build/input/*; do
       process $f
    done
fi