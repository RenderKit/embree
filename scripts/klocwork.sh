#!/bin/bash
make clean > /dev/null
kwinject -w -o buildspec.txt make -j 8 > /dev/null
#kwcheck create -b buildspec.txt --url http://localhost:8080/embree
#kwcheck run
kwbuildproject --force --url http://localhost:8080/embree buildspec.txt --tables-directory mytables
kwadmin --url http://localhost:8080 load embree mytables
#kwcheck list -F detailed
#kwcheck list -F detailed > klocwork.log
