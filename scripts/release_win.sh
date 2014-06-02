#!/bin/bash
mkdir -p $1/bin/x64

mkdir -p $1/lib/x64

mkdir -p $1/include
cp -r include/embree2 $1/include

make -C doc readme_bin.txt readme_bin.pdf
cp doc/readme_bin.txt $1/readme.txt
cp doc/readme_bin.pdf $1/readme.pdf
