#!/bin/bash

# terminate if some error occurs
set -e

# create package
cmake --build . --config $1 --target package
