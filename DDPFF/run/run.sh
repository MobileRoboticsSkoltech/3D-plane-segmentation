#!/bin/bash

set -ex

python3 ascii.py $1

./DDPFFAdoptation

python3 colorpcd.py $1