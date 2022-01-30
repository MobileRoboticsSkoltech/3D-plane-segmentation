#!/bin/bash

set -ex

./pdpcComputeMultiScaleFeatures -i input/$1 -v -o output/output
./pdpcSegmentation -i input/$1 -s output/output_scales.txt -f output/output_features.txt -o output/output -v

mkdir output/res1 output/res2 output/res3

./pdpcPostProcess -i input/$1 -s output/output_seg.txt -c output/output_comp.txt -o output/res1/res1 -col -v -range 20 24 25 30 40 42

./pdpcPostProcess -i input/$1 -s output/output_seg.txt -c output/output_comp.txt -o output/res2/res2 -col -v -pers 15 20 25

./pdpcPostProcess -i input/$1 -s output/output_seg.txt -c output/output_comp.txt -o output/res3/res3 -col -v -scales 5 15 20 25

