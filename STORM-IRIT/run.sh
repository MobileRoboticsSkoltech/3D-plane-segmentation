#!/bin/bash

set -ex

filename="${1%.*}"

./pdpcComputeMultiScaleFeatures -i input/$1 -v -o output/$filename
./pdpcSegmentation -i input/$1 -s output/"${filename}_scales.txt" -f output/"${filename}_features.txt" -o output/$filename -v

mkdir output/range output/pers output/scales

./pdpcPostProcess -i input/$1 -s output/"${filename}_seg.txt" -c output/"${filename}_comp.txt" -o output/range/$filename -col -v -range 20 24 25 30 40 42

./pdpcPostProcess -i input/$1 -s output/"${filename}_seg.txt" -c output/"${filename}_comp.txt" -o output/pers/$filename -col -v -pers 15 20 25

./pdpcPostProcess -i input/$1 -s output/"${filename}_seg.txt" -c output/"${filename}_comp.txt" -o output/scales/$filename -col -v -scales 5 15 20 25

