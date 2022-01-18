#!/bin/bash

set -ex

mkdir cpf_segmentation input output

cp -fR gco-v3.0 include src CMakeLists.txt cpf_segmentation/

rm -r gco-v3.0 include src CMakeLists.txt
