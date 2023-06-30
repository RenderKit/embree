## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

This folder contains sources of the Embree tutorials for illustration
purposes of the API. Follow the next sections to compile the
tutorials.

Compilation under Windows using SYCL
-------------------------------------

Install one of the SYCL compilers supported by Embree, see section
[Compiling Embree / Windows SYCL Compilation] of the Embree
documentation.
  
Then open some "x64 Native Tools Command Prompt" of Visual Studio and
execute the following line which properly configures the environment to
use the DPC++ compiler:

    set "DPCPP_DIR=path_to_dpcpp_compiler"
    set "PATH=%DPCPP_DIR%\bin;%PATH%"
    set "PATH=%DPCPP_DIR%\lib;%PATH%"
    set "CPATH=%DPCPP_DIR%\include;%CPATH%"
    set "INCLUDE=%DPCPP_DIR%\include;%INCLUDE%"
    set "LIB=%DPCPP_DIR%\lib;%LIB%"

Now download and unpack some recent TBB version for Windows, e.g. oneTBB 2021.2.0:

  https://github.com/oneapi-src/oneTBB/releases/download/v2021.2.0/oneapi-tbb-2021.2.0-win.zip
  
To compile tutorials contained in this folder first create and enter a
build folder inside the tutorial you want to compile. E.g. for the
minimal tutorial execute:

    cd minimal
    mkdir build
    cd build

Then configure the tutorial you would like to compile by pointing to
the Embree and TBB installation on your system.

    cmake -G Ninja
          -D CMAKE_CXX_COMPILER=clang++
          -D CMAKE_C_COMPILER=clang
          -D CMAKE_BUILD_TYPE=Release
          -D embree_DIR=%cd%\..\..\..\lib\cmake\embree-4.2.0\
          -D TBB_DIR=path_to_tbb\oneapi-tbb-2021.2.0\lib\cmake\tbb ..

Now you can build the tutorial:

    cmake --build .

In order to run the tutorial you have to set the PATH to the bin
folder in order to find the Embree DLL:

  set PATH=%cd%\..\..\..\bin;%PATH%

Now you can run the tutorial:

    .\minimal.exe
    .\minimal_sycl.exe


Compilation under Linux using SYCL
-----------------------------------

Install one of the SYCL compilers supported by Embree, see section
[Compiling Embree / Linux SYCL Compilation] of the Embree
documentation.

To properly configure your environment, you have to source the
`startup.sh` of the unpacked DPC++ compiler:

    source dpcpp_compiler/startup.sh

Now download and unpack some recent TBB version for Linux, e.g. oneTBB 2021.2.0:

  wget https://github.com/oneapi-src/oneTBB/releases/download/v2021.2.0/oneapi-tbb-2021.2.0-lin.tgz
  tar xzf oneapi-tbb-2021.2.0-lin.tgz

To compile tutorials contained in this folder first create and enter a
build folder inside the tutorial you want to compile. E.g. for the
minimal tutorial execute:

    cd minimal
    mkdir build
    cd build

Then configure the tutorial you would like to compile by pointing to
the Embree and TBB installation on your system.

    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_CXX_COMPILER=clang++ \
          -D CMAKE_C_COMPILER=clang \
          -D embree_DIR=`pwd`/../../../lib/cmake/embree-4.2.0/ \
          -D TBB_DIR=path_to_tbb/oneapi-tbb-2021.2.0/lib/cmake/tbb/ ..

Now you can build the tutorial:

    cmake --build .

Now you can run the tutorial:

    ./minimal
    ./minimal_sycl

