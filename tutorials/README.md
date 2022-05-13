## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

To compile tutorials contained in this folder first create and enter a
build folder inside the tutorial you want to compile (e.g. minimal
tutorial):

    cd minimal
    mkdir build
    cd build

Then configure the tutorial you would like to compile (e.g. minimal),
by pointing to the Embree and TBB installation on your system:

    cmake -D CMAKE_BUILD_TYPE=Release -D embree_DIR=`pwd`/../../../lib/cmake/embree-4.0.4/ -D TBB_DIR=path_to_tbb_install ..

Now you can build the tutorial:

    cmake --build .

And run it:

    ./minimal
    ./minimal_sycl

