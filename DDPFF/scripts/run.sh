#!/bin/bash

set -ex

if [ "$#" -eq 1 ]; 
then 
    python3 preprocess_cloud.py $1

    ./DDPFFAdoptation

    python3 build_cloud_from_planes.py $1

else
    for f in /app/build/input/*; do
        python3 preprocess_cloud.py "${f##/*/}"
        ./DDPFFAdoptation
        python3 build_cloud_from_planes.py "${f##/*/}"
    done
fi