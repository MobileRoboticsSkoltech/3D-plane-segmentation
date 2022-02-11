#!/bin/bash

set -ex

python3 preprocess_cloud.py $1

./DDPFFAdoptation

python3 build_cloud_from_planes.py $1