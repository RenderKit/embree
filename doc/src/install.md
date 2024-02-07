Installation of Embree
======================


Windows Installation
--------------------

A pre-built version of Embree for Windows is provided as a ZIP archive
[embree-<EMBREE_VERSION>.x64.windows.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.windows.zip). After
unpacking this ZIP file, you should set the path to the `lib` folder
manually to your `PATH` environment variable for applications to find
Embree.


Linux Installation
------------------

A pre-built version of Embree for Linux is provided as a `tar.gz` archive:
[embree-<EMBREE_VERSION>.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.linux.tar.gz). Unpack
this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the C
shell) to set up the environment properly:

    tar xzf embree-<EMBREE_VERSION>.x86_64.linux.tar.gz
    source embree-<EMBREE_VERSION>.x86_64.linux/embree-vars.sh

We recommend adding a relative `RPATH` to your application that points
to the location where Embree (and TBB) can be found, e.g. `$ORIGIN/../lib`.


macOS Installation
------------------

The macOS version of Embree is also delivered as a ZIP file:
[embree-<EMBREE_VERSION>.x86_64.macosx.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.macosx.zip). Unpack
this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the C
shell) to set up the environment properly:

    unzip embree-<EMBREE_VERSION>.x64.macosx.zip    source embree-<EMBREE_VERSION>.x64.macosx/embree-vars.sh

If you want to ship Embree with your application, please use the Embree
library of the provided ZIP file. The library name of that Embree
library is of the form `@rpath/libembree.<EMBREE_VERSION_MAJOR>.dylib`
(and similar also for the included TBB library). This ensures that you
can add a relative `RPATH` to your application that points to the location
where Embree (and TBB) can be found, e.g. `@loader_path/../lib`.


Building Embree Applications
----------------------------

The most convenient way to build an Embree application is through
CMake. Just let CMake find your unpacked Embree package using the
`FIND_PACKAGE` function inside your `CMakeLists.txt` file:

     FIND_PACKAGE(embree <EMBREE_VERSION_MAJOR> REQUIRED)

For CMake to properly find Embree you need to set the `embree_DIR` variable to
the folder containing the `embree_config.cmake` file. You might also have to
set the `TBB_DIR` variable to the path containing `TBB-config.cmake` of a local
TBB install, in case you do not have TBB installed globally on your system,
e.g:

    cmake -D embree_DIR=path_to_embree_package/lib/cmake/embree-<EMBREE_VERSION>/ \
          -D TBB_DIR=path_to_tbb_package/lib/cmake/tbb/ \
          ..

The `FIND_PACKAGE` function will create an `embree` target that
you can add to your target link libraries:

    TARGET_LINK_LIBRARIES(application embree)

For a full example on how to build an Embree application please have a
look at the `minimal` tutorial provided in the `src` folder of the
Embree package and also the contained `README.txt` file.


Building Embree SYCL Applications
----------------------------------

Building Embree SYCL applications is also best done using
CMake. Please first get some compatible SYCL compiler and setup the
environment as decribed in sections [Linux SYCL Compilation] and
[Windows SYCL Compilation].

Also perform the setup steps from the previous [Building Embree
Applications] section.

Please also have a look at the [Minimal] tutorial that is provided
with the Embree release, for an example how to build a simple SYCL
application using CMake and Embree.

To properly compile your SYCL application you have to add additional
SYCL compile flags for each C++ file that contains SYCL device side
code or kernels as described next.


### JIT Compilation

We recommend using just in time compilation (JIT compilation) together
with [SYCL JIT caching] to compile Embree SYCL applications. For JIT
compilation add these options to the compilation phase of all C++
files that contain SYCL code:

    -fsycl -Xclang -fsycl-allow-func-ptr -fsycl-targets=spir64

These options enable SYCL two phase compilation (`-fsycl` option),
enable function pointer support (`-Xclang -fsycl-allow-func-ptr`
option), and just in time (JIT) compilation only
(`-fsycl-targets=spir64` option).

The following link options have to get added to the linking stage of
your application when using just in time compilation:

    -fsycl -fsycl-targets=spir64

For a full example on how to build an Embree SYCL application please
have a look at the SYCL version of the `minimal` tutorial provided in
the `src` folder of the Embree package and also the contained
`README.txt` file.

Please have a look at the [Compiling Embree] section on how to create
an Embree package from sources if required.


### AOT Compilation

Ahead of time compilation (AOT compilation) allows to speed up first
application start up time as device binaries are precompiled. We do
not recommend using AOT compilation as it does not allow the usage of
specialization constants to reduce code complexity.

For ahead of time compilation add these compile options to the
compilation phase of all C++ files that contain SYCL code:

    -fsycl -Xclang -fsycl-allow-func-ptr -fsycl-targets=spir64_gen

These options enable SYCL two phase compilation (`-fsycl` option),
enable function pointer support (`-Xclang -fsycl-allow-func-ptr`
option), and ahead of time (AOT) compilation
(`-fsycl-targets=spir64_gen` option).

The following link options have to get added to the linking stage of
your application when compiling ahead of time for Xe HPG devices:

    -fsycl -fsycl-targets=spir64_gen
    -Xsycl-target-backend=spir64_gen "-device XE_HPG_CORE"

This in particular configures the devices for AOT compilation to
`XE_HPG_CORE`.

To get a list of all device supported by AOT compilation look at the
help of the device option in ocloc tool:

    ocloc compile --help


Building Embree Tests
---------------------

Embree is released with a bundle of tests in an optional testing package.
To run these tests extract the testing package in the same folder as your embree installation.
e.g.:
    
    tar -xzf embree-<EMBREE_VERSION>-testing.zip -C /path/to/installed/embree

The tests are extracted into a new folder inside you embree installation and can be run with:

    cd /path/to/installed/embree/testing
    cmake -B build
    cmake --build build target=tests


