#!/bin/bash

set -ex

base_name=$(basename -- "$1")
filename="${base_name%.*}"

./pdpcComputeMultiScaleFeatures -i input/$base_name -v -o output/$filename
./pdpcSegmentation -i input/$base_name -s output/"${filename}_scales.txt" -f output/"${filename}_features.txt" -o output/$filename -v

mkdir output/"${filename}_range" output/"${filename}_pers" output/"${filename}_scales"

./pdpcPostProcess -i input/$base_name -s output/"${filename}_seg.txt" -c output/"${filename}_comp.txt" -o output/"${filename}_range"/$filename -col -v -range 20 24 25 30 40 42

./pdpcPostProcess -i input/$base_name -s output/"${filename}_seg.txt" -c output/"${filename}_comp.txt" -o output/"${filename}_pers"/$filename -col -v -pers 15 20 25

./pdpcPostProcess -i input/$base_name -s output/"${filename}_seg.txt" -c output/"${filename}_comp.txt" -o output/"${filename}_scales"/$filename -col -v -scales 5 15 20 25
