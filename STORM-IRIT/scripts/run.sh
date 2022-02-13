#!/bin/bash

set -ex

process(){
    base_name=$(basename -- "$1")
    filename="${base_name%.*}"
    path=output/$filename
    
    python3 preprocess_cloud.py $base_name
    ./pdpcComputeMultiScaleFeatures -i $path/"${filename}.ply" -v -o $path/$filename
    ./pdpcSegmentation -i $path/"${filename}.ply" -s $path/"${filename}_scales.txt" -f $path/"${filename}_features.txt" -o $path/$filename -v

    # mkdir output/"${filename}_range" output/"${filename}_pers" output/"${filename}_scales"

    ./pdpcPostProcess -i $path/"${filename}.ply" -s $path/"${filename}_seg.txt" -c $path/"${filename}_comp.txt" -o $path/"${filename}_range"/$filename -col -v -range 20 24 25 30 40 42

    ./pdpcPostProcess -i $path/"${filename}.ply" -s $path/"${filename}_seg.txt" -c $path/"${filename}_comp.txt" -o $path/"${filename}_pers"/$filename -col -v -pers 15 20 25

    ./pdpcPostProcess -i $path/"${filename}.ply" -s $path/"${filename}_seg.txt" -c $path/"${filename}_comp.txt" -o $path/"${filename}_scales"/$filename -col -v -scales 5 15 20 25


}

if [ "$#" -eq 1 ]; 
then 
    process $1
else
    for f in /app/build/input/*; do
        process $f
    done
fi

