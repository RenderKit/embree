#!/bin/bash
make clean
kwinject -w -o buildspec.txt make -j 8
kwcheck create -b buildspec.txt --url http://localhost:8080/embree
kwcheck run
kwcheck list -F detailed
kwcheck list -F detailed > klocwork.log

