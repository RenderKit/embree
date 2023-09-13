#!\bin\bash

source scripts/cmake-presets/linux-env.sh 
$CMAKE_EXE --build build --config Release --target package