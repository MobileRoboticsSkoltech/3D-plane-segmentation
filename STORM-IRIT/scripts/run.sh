#!/bin/bash

set -ex

process(){
    base_name=$(basename -- "$1")
    filename="${base_name%.*}"
    path=output/$filename
    
    python3 preprocess_cloud.py $base_name
    ./pdpcComputeMultiScaleFeatures -i "${filename}.ply" -v -o $path/$filename
    ./pdpcSegmentation -i "${filename}.ply" -s $path/"${filename}_scales.txt" -f $path/"${filename}_features.txt" -o $path/$filename -v

    ./pdpcPostProcess -i "${filename}.ply" -s $path/"${filename}_seg.txt" -c $path/"${filename}_comp.txt" -o $path/"${filename}_range" -col -v -range 20 24 25 30 40 42

    ./pdpcPostProcess -i "${filename}.ply" -s $path/"${filename}_seg.txt" -c $path/"${filename}_comp.txt" -o $path/"${filename}_pers" -col -v -pers 15 20 25

    ./pdpcPostProcess -i "${filename}.ply" -s $path/"${filename}_seg.txt" -c $path/"${filename}_comp.txt" -o $path/"${filename}_scales" -col -v -scales 5 15 20 25
    rm $path/"${filename}_scales.txt"
    rm $path/"${filename}_features.txt"
    rm $path/"${filename}_seg.txt"
    rm $path/"${filename}_comp.txt"
    for f in $path/*.ply; do
        python3 convert_ply.py $f
    done
    for f in $path/*.txt; do
        python3 convert_txt.py $f
    done
}

if [ "$#" -eq 1 ]; 
then 
    process $1
else
    for f in /app/build/input/*; do
        process $f
    done
fi

