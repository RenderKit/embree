% Embree: High Performance Ray Tracing Kernels 4.4.0
% Intel Corporation

Intel® Embree Overview
======================

Intel® Embree is a high-performance ray tracing library developed at
Intel, which is released as open source under the [Apache 2.0
license](http://www.apache.org/licenses/LICENSE-2.0). Intel® Embree
supports x86 CPUs under Linux, macOS, and Windows; ARM CPUs on Linux
and macOS; as well as Intel® GPUs under Linux and Windows.

Intel® Embree targets graphics application developers to improve the
performance of photo-realistic rendering applications. Embree is
optimized towards production rendering, by putting focus on incoherent
ray performance, high quality acceleration structure construction, a
rich feature set, accurate primitive intersection, and low memory
consumption.

Embree's feature set includes various primitive types such as
triangles (as well quad and grids for lower memory consumption);
Catmull-Clark subdivision surfaces; various types of curve primitives,
such as flat curves (for distant views), round curves (for closeup
views), and normal oriented curves, all supported with different basis
functions (linear, Bézier, B-spline, Hermite, and Catmull Rom);
point-like primitives, such as ray oriented discs, normal oriented
discs, and spheres; user defined geometries with a procedural
intersection function; multi-level instancing; filter callbacks
invoked for any hit encountered; motion blur including multi-segment
motion blur, deformation blur, and quaternion motion blur; and ray
masking.

Intel® Embree contains ray tracing kernels optimized for the latest
x86 processors with support for SSE, AVX, AVX2, and AVX-512
instructions, and uses runtime code selection to choose between these
kernels. Intel® Embree contains algorithms optimized for incoherent
workloads (e.g.  Monte Carlo ray tracing algorithms) and coherent
workloads (e.g. primary visibility and hard shadow rays) as well as
supports for dynamic scenes by implementing high-performance two-level
spatial index structure construction algorithms.

Intel® Embree supports applications written with the Intel® Implicit
SPMD Program Compiler (Intel® ISPC, <https://ispc.github.io/>) by
providing an ISPC interface to the core ray tracing
algorithms. This makes it possible to write a renderer that
automatically vectorizes and leverages SSE, AVX, AVX2, and AVX-512
instructions.

Intel® Embree supports Intel GPUs through the
[SYCL](https://www.khronos.org/sycl/) open standard programming
language. SYCL allows to write C++ code that can be run on various
devices, such as CPUs and GPUs. Using Intel® Embree application
developers can write a single source renderer that executes
efficiently on CPUs and GPUs. Maintaining just one code base
this way can significantly improve productivity and eliminate
inconsistencies between a CPU and GPU version of the renderer. Embree
supports GPUs based on the Xe HPG and Xe HPC microarchitecture,
which support hardware accelerated ray tracing do deliver excellent
levels of ray tracing performance.

Supported Platforms
-------------------

Embree supports Windows (32-bit and 64-bit), Linux (64-bit), and macOS
(64-bit). Under Windows, Linux and macOS x86 based CPUs are supported,
while ARM CPUs are currently only supported under Linux and macOS (e.g. 
Apple M1). ARM support for Windows experimental.

Embree supports Intel GPUs based on the Xe HPG microarchitecture
(Intel® Arc™ GPU) under Linux and Windows and Xe HPC microarchitecture
(Intel® Data Center GPU Flex Series and Intel® Data Center GPU Max
Series) under Linux.

The code compiles with the Intel® Compiler, Intel® oneAPI DPC++
Compiler, GCC, Clang, and the Microsoft Compiler. To use Embree on the
GPU the Intel® oneAPI DPC++ Compiler must be used. Please see section
[Compiling Embree] for details on tested compiler versions.

Embree requires at least an x86 CPU with support for
SSE2 or an Apple M1 CPU.

Embree Support and Contact
--------------------------

If you encounter bugs please report them via [Embree's GitHub Issue
Tracker](https://github.com/embree/embree/issues).

For questions and feature requests please write us at
<embree_support@intel.com>.

To receive notifications of updates and new features of Embree please
subscribe to the [Embree mailing
list](https://groups.google.com/d/forum/embree/).

Installation of Embree
======================


Windows Installation
--------------------

A pre-built version of Embree for Windows is provided as a ZIP archive
[embree-4.4.0.x64.windows.zip](https://github.com/embree/embree/releases/download/v4.4.0/embree-4.4.0.x64.windows.zip). After
unpacking this ZIP file, you should set the path to the `lib` folder
manually to your `PATH` environment variable for applications to find
Embree.


Linux Installation
------------------

A pre-built version of Embree for Linux is provided as a `tar.gz` archive:
[embree-4.4.0.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v4.4.0/embree-4.4.0.x86_64.linux.tar.gz). Unpack
this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the C
shell) to set up the environment properly:

    tar xzf embree-4.4.0.x86_64.linux.tar.gz
    source embree-4.4.0.x86_64.linux/embree-vars.sh

We recommend adding a relative `RPATH` to your application that points
to the location where Embree (and TBB) can be found, e.g. `$ORIGIN/../lib`.


macOS Installation
------------------

The macOS version of Embree is also delivered as a ZIP file:
[embree-4.4.0.x86_64.macosx.zip](https://github.com/embree/embree/releases/download/v4.4.0/embree-4.4.0.x86_64.macosx.zip). Unpack
this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the C
shell) to set up the environment properly:

    unzip embree-4.4.0.x64.macosx.zip    source embree-4.4.0.x64.macosx/embree-vars.sh

If you want to ship Embree with your application, please use the Embree
library of the provided ZIP file. The library name of that Embree
library is of the form `@rpath/libembree.4.dylib`
(and similar also for the included TBB library). This ensures that you
can add a relative `RPATH` to your application that points to the location
where Embree (and TBB) can be found, e.g. `@loader_path/../lib`.


Building Embree Applications
----------------------------

The most convenient way to build an Embree application is through
CMake. Just let CMake find your unpacked Embree package using the
`FIND_PACKAGE` function inside your `CMakeLists.txt` file:

     FIND_PACKAGE(embree 4 REQUIRED)

For CMake to properly find Embree you need to set the `embree_DIR` variable to
the folder containing the `embree_config.cmake` file. You might also have to
set the `TBB_DIR` variable to the path containing `TBB-config.cmake` of a local
TBB install, in case you do not have TBB installed globally on your system,
e.g:

    cmake -D embree_DIR=path_to_embree_package/lib/cmake/embree-4.4.0/ \
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
    
    tar -xzf embree-4.4.0-testing.zip -C /path/to/installed/embree

The tests are extracted into a new folder inside you embree installation and can be run with:

    cd /path/to/installed/embree/testing
    cmake -B build
    cmake --build build target=tests


Compiling Embree
================

We recommend using the prebuild Embree packages from
[https://github.com/embree/embree/releases](https://github.com/embree/embree/releases). If
you need to compile Embree yourself you need to use CMake as described
in the following.

Do not enable fast-math optimizations in your compiler as this mode is
not supported by Embree.

Linux and macOS
---------------

To compile Embree you need a modern C++ compiler that supports
C++11. Embree is tested with the following compilers:

Linux

  - Intel® oneAPI DPC++/C++ Compiler 2024.0.2
  - oneAPI DPC++/C++ Compiler 2023-10-26
  - Clang 5.0.0
  - Clang 4.0.0
  - GCC 10.0.1 (Fedora 32) AVX512 support
  - GCC  8.3.1 (Fedora 29) AVX512 support
  - Intel® Implicit SPMD Program Compiler 1.22.0

macOS x86_64

  - Apple Clang 15

macOS Arm64

  - Apple Clang 14

Embree supports using the Intel® Threading Building Blocks (TBB) as the
tasking system. For performance and flexibility reasons we recommend
using Embree with the Intel® Threading Building Blocks (TBB) and best
also use TBB inside your application. Optionally you can disable TBB
in Embree through the `EMBREE_TASKING_SYSTEM` CMake variable.

Embree supports the Intel® Implicit SPMD Program Compiler (Intel® ISPC), which allows
straightforward parallelization of an entire renderer. If you
want to use Intel® ISPC then you can enable `EMBREE_ISPC_SUPPORT` in
CMake. Download and install the Intel® ISPC binaries from
[ispc.github.io](https://ispc.github.io/downloads.html). After
installation, put the path to `ispc` permanently into your `PATH` environment
variable or you set the `EMBREE_ISPC_EXECUTABLE` variable to point at the ISPC
executable during CMake configuration.

You additionally have to install CMake 3.1.0 or higher and the developer
version of [GLFW](https://www.glfw.org/) version 3.

Under macOS, all these dependencies can be installed
using [MacPorts](http://www.macports.org/):

    sudo port install cmake tbb glfw-devel

Depending on your Linux distribution you can install these dependencies
using `yum` or `apt-get`.  Some of these packages might already be
installed or might have slightly different names.

Type the following to install the dependencies using `yum`:

    sudo yum install cmake
    sudo yum install tbb-devel
    sudo yum install glfw-devel

Type the following to install the dependencies using `apt-get`:

    sudo apt-get install cmake-curses-gui
    sudo apt-get install libtbb-dev
    sudo apt-get install libglfw3-dev

Finally, you can compile Embree using CMake. Create a build directory
inside the Embree root directory and execute `ccmake ..` inside this
build directory.

    mkdir build
    cd build
    ccmake ..

Per default, CMake will use the compilers specified with the `CC` and
`CXX` environment variables. Should you want to use a different
compiler, run `cmake` first and set the `CMAKE_CXX_COMPILER` and
`CMAKE_C_COMPILER` variables to the desired compiler. For example, to
use the Clang compiler instead of the default GCC on most Linux machines
(`g++` and `gcc`), execute

    cmake -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang ..

Running `ccmake` will open a dialog where you can perform various
configurations as described below in [CMake Configuration]. After having
configured Embree, press `c` (for configure) and `g` (for generate) to
generate a Makefile and leave the configuration. The code can be
compiled by executing make.

    make -j 8

The executables will be generated inside the build folder. We recommend
installing the Embree library and header files on your
system. Therefore set the `CMAKE_INSTALL_PREFIX` to `/usr` in cmake
and type:

    sudo make install

If you keep the default `CMAKE_INSTALL_PREFIX` of `/usr/local` then
you have to make sure the path `/usr/local/lib` is in your
`LD_LIBRARY_PATH`.

You can also uninstall Embree again by executing:

    sudo make uninstall

You can also create an Embree package using the following command:

    make package

Please see the [Building Embree Applications] section on how to build
your application with such an Embree package.

Linux SYCL Compilation
-----------------------

There are two options to compile Embree with SYCL support:
The open source ["oneAPI DPC++ Compiler"](https://github.com/intel/llvm/) or
the ["Intel(R) oneAPI DPC++/C++ Compiler"](https://www.intel.com/content/www/us/en/developer/articles/tool/oneapi-standalone-components.html#dpcpp-cpp).
Other SYCL compilers are not supported.

The "oneAPI DPC++ Compiler" is more up-to-date than the "Intel(R) oneAPI
DPC++/C++ Compiler" but less stable. The current tested version of the "oneAPI
DPC++ compiler is

  - [oneAPI DPC++ Compiler 2023-10-26](https://github.com/intel/llvm/releases/tag/nightly-2023-10-26)
  
The compiler can be downloaded and simply extracted. The oneAPI DPC++ compiler
can be set up executing the following commands in a Linux (bash) shell:

    export SYCL_BUNDLE_ROOT=path_to_dpcpp_compiler
    export PATH=$SYCL_BUNDLE_ROOT/bin:$PATH
    export CPATH=$SYCL_BUNDLE_ROOT/include:$CPATH
    export LIBRARY_PATH=$SYCL_BUNDLE_ROOT/lib:$LIBRARY_PATH
    export LD_LIBRARY_PATH=$SYCL_BUNDLE_ROOT/lib:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=$SYCL_BUNDLE_ROOT/linux/lib/x64:$LD_LIBRARY_PATH

where the `path_to_dpcpp_compiler` should point to the unpacked oneAPI DPC++
compiler. This will put `clang++` and `clang` from the oneAPI DPC++ Compiler
into your path.

Please also install all Linux packages described in the previous
section.

Now, you can configure Embree using CMake by executing the following command
in the Embree root directory:

    cmake -B build \
          -DCMAKE_CXX_COMPILER=clang++ \
          -DCMAKE_C_COMPILER=clang \
          -DEMBREE_SYCL_SUPPORT=ON

This will create a directory `build` to use as the CMake build directory,
configure the usage of the oneAPI DPC++ Compiler, and turn on SYCL support
through `EMBREE_SYCL_SUPPORT=ON`.

Alternatively, you can download and run the installer of the

 - [Intel(R) oneAPI DPC++/C++ Compiler](https://www.intel.com/content/www/us/en/developer/articles/tool/oneapi-standalone-components.html#dpcpp-cpp).

After installation, you can set up the compiler by sourcing the
`vars.sh` script in the `env` directory of the compiler install directory, for example,

    source /opt/intel/oneAPI/compiler/latest/env/vars.sh

This script will put the `icpx` and `icx` compiler executables from the
Intel(R) oneAPI DPC++/C++ Compiler in your path.

Now, you can configure Embree using CMake by executing the following command
in the Embree root directory:

    cmake -B build \
          -DCMAKE_CXX_COMPILER=icpx \
          -DCMAKE_C_COMPILER=icx \
          -DEMBREE_SYCL_SUPPORT=ON

More information about setting up the Intel(R) oneAPI DPC++/C++ compiler can be
found in the [Development Reference Guide](https://www.intel.com/content/www/us/en/develop/documentation/oneapi-dpcpp-cpp-compiler-dev-guide-and-reference/top/compiler-setup.html). Please note, that the Intel(R) oneAPI DPC++/C++ compiler
requires [at least CMake version 3.20.5 on Linux](https://www.intel.com/content/www/us/en/develop/documentation/oneapi-dpcpp-cpp-compiler-dev-guide-and-reference/top/compiler-setup/use-the-command-line/use-cmake-with-the-compiler.html).

Independent of the DPC++ compiler choice, you can now build Embree using

    cmake --build build -j 8

The executables will be generated inside the build folder. The
executable names of the SYCL versions of the tutorials end with
`_sycl`.


### Linux Graphics Driver Installation

To run the SYCL code you need to install the latest GPGPU drivers for
your Intel Xe HPG/HPC GPUs from here
[https://dgpu-docs.intel.com/](https://dgpu-docs.intel.com/). Follow
the driver installation instructions for your graphics card and
operating system.

After installing the drivers you have to install an additional package
manually using

    sudo apt install intel-level-zero-gpu-raytracing


Windows
-------

Embree is tested using the following compilers under Windows:

  - Intel® oneAPI DPC++/C++ Compiler 2024.0.2
  - oneAPI DPC++/C++ Compiler 2023-10-26
  - Visual Studio 2022
  - Visual Studio 2019
  - Visual Studio 2017
  - Intel® Implicit SPMD Program Compiler 1.22.0

To compile Embree for AVX-512 you have to use the Intel® Compiler.

Embree supports using the Intel® Threading Building Blocks (TBB) as the
tasking system. For performance and flexibility reasons we recommend
using use Embree with the Intel® Threading Building Blocks (TBB) and best
also use TBB inside your application. Optionally you can disable TBB
in Embree through the `EMBREE_TASKING_SYSTEM` CMake variable.

Embree will either find the Intel® Threading Building Blocks (TBB)
installation that comes with the Intel® Compiler, or you can install the
binary distribution of TBB directly from
[https://github.com/oneapi-src/oneTBB/releases](https://github.com/oneapi-src/oneTBB/releases)
into a folder named `tbb` into your Embree root directory. You also have
to make sure that the libraries `tbb.dll` and `tbb_malloc.dll` can be
found when executing your Embree applications, e.g. by putting the path
to these libraries into your `PATH` environment variable.

Embree supports the Intel® Implicit SPMD Program Compiler (Intel® ISPC), which
allows straightforward parallelization of an entire renderer. When installing
Intel® ISPC, make sure to download an Intel® ISPC version from
[ispc.github.io](https://ispc.github.io/downloads.html) that is compatible with
your Visual Studio version. After installation, put the path to `ispc.exe`
permanently into your `PATH` environment variable or you need to correctly set
the `EMBREE_ISPC_EXECUTABLE` variable during CMake configuration to point to
the ISPC executable. If you want to use Intel® ISPC, you have to enable
`EMBREE_ISPC_SUPPORT` in CMake.

You additionally have to install [CMake](http://www.cmake.org/download/)
(version 3.1 or higher). Note that you need a native Windows CMake
installation because CMake under Cygwin cannot generate solution files
for Visual Studio.

### Using the IDE

Run `cmake-gui`, browse to the Embree sources, set the build directory
and click Configure. Now you can select the Generator, e.g. "Visual
Studio 12 2013" for a 32-bit build or "Visual Studio 12 2013 Win64"
for a 64-bit build.

To use a different compiler than the Microsoft Visual C++ compiler, you
additionally need to specify the proper compiler toolset through the
option "Optional toolset to use (-T parameter)". E.g. to use Clang for
compilation set the toolset to "LLVM_v142".

Do not change the toolset manually in a solution file (neither through
the project properties dialog nor through the "Use Intel Compiler"
project context menu), because then some compiler-specific command line
options cannot be set by CMake.

Most configuration parameters described in the [CMake Configuration]
can be set under Windows as well. Finally, click "Generate" to create
the Visual Studio solution files. 

The following CMake options are only available under Windows:

+ `CMAKE_CONFIGURATION_TYPE`:  List of generated
  configurations. The default value is Debug;Release;RelWithDebInfo.

+  `USE_STATIC_RUNTIME`: Use the static version of the C/C++ runtime
  library. This option is turned OFF by default.

Use the generated Visual Studio solution file `embree4.sln` to compile
the project.

We recommend enabling syntax highlighting for the `.ispc` source and
`.isph` header files. To do so open Visual Studio, go to Tools ⇒
Options ⇒ Text Editor ⇒ File Extension and add the `isph` and `ispc`
extensions for the "Microsoft Visual C++" editor.

### Using the Command Line

Embree can also be configured and built without the IDE using the Visual
Studio command prompt:

    cd path\to\embree
    mkdir build
    cd build
    cmake -G "Visual Studio 16 2019" ..
    cmake --build . --config Release

You can also build only some projects with the `--target` switch.
Additional parameters after "`--`" will be passed to `msbuild`. For
example, to build the Embree library in parallel use

    cmake --build . --config Release --target embree -- /m

### Building Embree - Using vcpkg

You can download and install Embree using the [vcpkg](https://github.com/Microsoft/vcpkg) dependency manager:

    git clone https://github.com/Microsoft/vcpkg.git
    cd vcpkg
    ./bootstrap-vcpkg.sh
    ./vcpkg integrate install
    ./vcpkg install embree3

The Embree port in vcpkg is kept up to date by Microsoft team members
and community contributors. If the version is out of date, please
[create an issue or pull request](https://github.com/Microsoft/vcpkg)
on the vcpkg repository.


Windows SYCL Compilation
-------------------------

There are two options to compile Embree with SYCL support:
The open source ["oneAPI DPC++ Compiler"](https://github.com/intel/llvm/) or
the ["Intel(R) oneAPI DPC++/C++ Compiler"](https://www.intel.com/content/www/us/en/developer/articles/tool/oneapi-standalone-components.html#dpcpp-cpp).
Other SYCL compilers are not supported. You will also need an installed version
of Visual Studio that supports the C++17 standard, e.g. Visual Studio 2019.

The "oneAPI DPC++ Compiler" is more up-to-date than the "Intel(R) oneAPI
DPC++/C++ Compiler" but less stable. The current tested version of the oneAPI
DPC++ compiler is

  - [oneAPI DPC++ Compiler 2023-10-26](https://github.com/intel/llvm/releases/tag/nightly-2023-10-26)

Download and unpack the archive and open the "x64 Native Tools Command Prompt"
of Visual Studio and execute the following lines to properly configure the
environment to use the oneAPI DPC++ compiler:

    set "DPCPP_DIR=path_to_dpcpp_compiler"
    set "PATH=%DPCPP_DIR%\bin;%PATH%"
    set "PATH=%DPCPP_DIR%\lib;%PATH%"
    set "CPATH=%DPCPP_DIR%\include;%CPATH%"
    set "INCLUDE=%DPCPP_DIR%\include;%INCLUDE%"
    set "LIB=%DPCPP_DIR%\lib;%LIB%"

The `path_to_dpcpp_compiler` should point to the unpacked oneAPI DPC++
compiler.

Now, you can configure Embree using CMake by executing the following command
in the Embree root directory:

    cmake -B build
          -G Ninja
          -D CMAKE_BUILD_TYPE=Release
          -D CMAKE_CXX_COMPILER=clang++
          -D CMAKE_C_COMPILER=clang
          -D EMBREE_SYCL_SUPPORT=ON
          -D TBB_ROOT=path_to_tbb\lib\cmake\tbb

This will create a directory `build` to use as the CMake build directory, and
configure a release build that uses `clang++` and `clang` from the oneAPI DPC++
compiler.

The [Ninja](https://ninja-build.org/) generator is currently the easiest way to
use the oneAPI DPC++ compiler.

We also enable SYCL support in Embree using the `EMBREE_SYCL_SUPPORT` CMake
option.

Alternatively, you can download and run the installer of the

 - [Intel(R) oneAPI DPC++/C++ Compiler](https://www.intel.com/content/www/us/en/developer/articles/tool/oneapi-standalone-components.html#dpcpp-cpp).

After installation, you can either open a regular `Command Prompt` and execute
the `vars.bat` script in the `env` directory of the compiler install directory,
for example

    C:\Program Files (x86)\Intel\oneAPI\compiler\latest\env\vars.bat

or simply open the installed "Intel oneAPI command prompt for Intel 64 for Visual Studio".

Both ways will put the `icx` compiler executable from the
Intel(R) oneAPI DPC++/C++ compiler in your path.

Now, you can configure Embree using CMake by executing the following command
in the Embree root directory:

    cmake -B build
          -G Ninja
          -D CMAKE_BUILD_TYPE=Release
          -D CMAKE_CXX_COMPILER=icx
          -D CMAKE_C_COMPILER=icx
          -D EMBREE_SYCL_SUPPORT=ON
          -D TBB_ROOT=path_to_tbb\lib\cmake\tbb

More information about setting up the Intel(R) oneAPI DPC++/C++ compiler can be
found in the [Development Reference Guide](https://www.intel.com/content/www/us/en/develop/documentation/oneapi-dpcpp-cpp-compiler-dev-guide-and-reference/top/compiler-setup.html). Please note, that the Intel(R) oneAPI DPC++/C++ compiler
requires [at least CMake version 3.23 on Windows](https://www.intel.com/content/www/us/en/develop/documentation/oneapi-dpcpp-cpp-compiler-dev-guide-and-reference/top/compiler-setup/use-the-command-line/use-cmake-with-the-compiler.html).

Independent of the DPC++ compiler choice, you can now build Embree using

    cmake --build build

If you have problems with Ninja re-running CMake in an infinite loop,
then first remove the "Re-run CMake if any of its inputs changed."
section from the `build.ninja` file and run the above command again.

You can also create an Embree package using the following command:

    cmake --build build --target package

Please see the [Building Embree SYCL Applications] section on how to build
your application with such an Embree package.


### Windows Graphics Driver Installation

In order to run the SYCL tutorials on HPG hardware, you first need to
install the graphics drivers for your graphics card from
[https://www.intel.com](https://www.intel.com). Please make sure to
have installed version 31.0.101.4644 or newer.


CMake Configuration
-------------------

The default CMake configuration in the configuration dialog should be
appropriate for most usages. The following list describes all
parameters that can be configured in CMake:

+ `CMAKE_BUILD_TYPE`: Can be used to switch between Debug mode
  (Debug), Release mode (Release) (default), and Release mode with
  enabled assertions and debug symbols (RelWithDebInfo).

+ `EMBREE_STACK_PROTECTOR`: Enables protection of return address
  from buffer overwrites. This option is OFF by default.

+ `EMBREE_ISPC_SUPPORT`: Enables Intel® ISPC support of Embree. This option
  is OFF by default.

+ `EMBREE_SYCL_SUPPORT`: Enables GPU support using SYCL. When this
  option is enabled you have to use some DPC++ compiler. Please see
  the sections [Linux SYCL Compilation] and [Windows SYCL Compilation]
  on supported DPC++ compilers. This option is OFF by default.

+ `EMBREE_SYCL_AOT_DEVICES`: Selects a list of GPU devices for
  ahead-of-time (AOT) compilation of device code. Possible values are
  either, "none" which enables only just in time (JIT) compilation, or
  a list of the Embree-supported Xe GPUs for AOT compilation:

  * XE_HPG_CORE : Xe HPG devices
  * XE_HPC_CORE : Xe HPC devices

  One can also specify multiple devices separated by comma to
  compile ahead of time for multiple devices,
  e.g. "XE_HPG_CORE,XE_HP_CORE". When enabling AOT compilation for one
  or multiple devices, JIT compilation will always additionally be
  enabled in case the code is executed on a device no code is
  precompiled for.

  Execute "ocloc compile --help" for more details of possible devices
  to pass. Embree is only supported on Xe HPG/HPC and newer devices.

  Per default, this option is set to "none" to enable JIT
  compilation. We recommend using JIT compilation as this enables the
  use of specialization constants to reduce code complexity.

+ `EMBREE_STATIC_LIB`: Builds Embree as a static library (OFF by
  default). Further multiple static libraries are generated for the
  different ISAs selected (e.g. `embree4.a`, `embree4_sse42.a`,
  `embree4_avx.a`, `embree4_avx2.a`, `embree4_avx512.a`). You have
  to link these libraries in exactly this order of increasing ISA.

+ `EMBREE_API_NAMESPACE`: Specifies a namespace name to put all Embree
  API symbols inside. By default, no namespace is used and plain C symbols
  are exported.

+ `EMBREE_LIBRARY_NAME`: Specifies the name of the Embree library file
  created. By default, the name embree4 is used.

+ `EMBREE_IGNORE_CMAKE_CXX_FLAGS`: When enabled, Embree ignores
  default CMAKE_CXX_FLAGS. This option is turned ON by default.

+ `EMBREE_TUTORIALS`: Enables build of Embree tutorials (default ON).

+ `EMBREE_BACKFACE_CULLING`: Enables backface culling, i.e. only
  surfaces facing a ray can be hit. This option is turned OFF by
  default.

+ `EMBREE_BACKFACE_CULLING_CURVES`: Enables backface culling for curves,
  i.e. only surfaces facing a ray can be hit. This option is turned OFF
  by default.

+ `EMBREE_BACKFACE_CULLING_SPHERES`: Enables backface culling for spheres,
  i.e. only surfaces facing a ray can be hit. This option is turned OFF
  by default.

+ `EMBREE_COMPACT_POLYS`: Enables compact tris/quads, i.e. only
  geomIDs and primIDs are stored inside the leaf nodes.  

+ `EMBREE_FILTER_FUNCTION`: Enables the intersection filter function
  feature (ON by default).

+ `EMBREE_RAY_MASK`: Enables the ray masking feature (OFF by default).

+ `EMBREE_RAY_PACKETS`: Enables ray packet traversal kernels. This
  feature is turned ON by default. When turned on packet traversal is
  used internally and packets passed to rtcIntersect4/8/16 are kept
  intact in callbacks (when the ISA of appropriate width is enabled).

+ `EMBREE_IGNORE_INVALID_RAYS`: Makes code robust against the risk of
  full-tree traversals caused by invalid rays (e.g. rays containing
  INF/NaN as origins). This option is turned OFF by default.

+ `EMBREE_TASKING_SYSTEM`: Chooses between Intel® Threading TBB
  Building Blocks (TBB), Parallel Patterns Library (PPL) (Windows
  only), or an internal tasking system (INTERNAL). By default, TBB is
  used.

+ `EMBREE_TBB_ROOT`: If Intel® Threading Building Blocks (TBB)
  is used as a tasking system, search the library in this directory
  tree.

+ `EMBREE_TBB_COMPONENT`: The component/library name of Intel® Threading 
  Building Blocks (TBB). Embree searches for this library name (default: tbb)
  when TBB is used as the tasking system.

+ `EMBREE_TBB_POSTFIX`: If Intel® Threading Building Blocks (TBB)
  is used as a tasking system, link to tbb<EMBREE_TBB_POSTFIX>.(so,dll,lib).
  Defaults to the empty string.

+ `EMBREE_TBB_DEBUG_ROOT`: If Intel® Threading Building Blocks (TBB)
  is used as a tasking system, search the library in this directory
  tree in Debug mode. Defaults to `EMBREE_TBB_ROOT`.

+ `EMBREE_TBB_DEBUG_POSTFIX`: If Intel® Threading Building Blocks (TBB)
  is used as a tasking system, link to tbb<EMBREE_TBB_DEBUG_POSTFIX>.(so,dll,lib)
  in Debug mode. Defaults to "_debug".

+ `EMBREE_MAX_ISA`: Select highest supported ISA (SSE2, SSE4.2, AVX,
  AVX2, AVX512, or NONE). When set to NONE the
  EMBREE_ISA_* variables can be used to enable ISAs individually. By
  default, the option is set to AVX2.

+ `EMBREE_ISA_SSE2`: Enables SSE2 when EMBREE_MAX_ISA is set to
  NONE. By default, this option is turned OFF.

+ `EMBREE_ISA_SSE42`: Enables SSE4.2 when EMBREE_MAX_ISA is set to
  NONE. By default, this option is turned OFF.

+ `EMBREE_ISA_AVX`: Enables AVX when EMBREE_MAX_ISA is set to NONE. By
  default, this option is turned OFF.

+ `EMBREE_ISA_AVX2`: Enables AVX2 when EMBREE_MAX_ISA is set to
  NONE. By default, this option is turned OFF.

+ `EMBREE_ISA_AVX512`: Enables AVX-512 for Skylake when
  EMBREE_MAX_ISA is set to NONE. By default, this option is turned OFF.

+ `EMBREE_GEOMETRY_TRIANGLE`: Enables support for triangle geometries
  (ON by default).

+ `EMBREE_GEOMETRY_QUAD`: Enables support for quad geometries (ON by
  default).

+ `EMBREE_GEOMETRY_CURVE`: Enables support for curve geometries (ON by
  default).

+ `EMBREE_GEOMETRY_SUBDIVISION`: Enables support for subdivision
  geometries (ON by default).

+ `EMBREE_GEOMETRY_INSTANCE`: Enables support for instances (ON by
  default).

+ `EMBREE_GEOMETRY_INSTANCE_ARRAY`: Enables support for instance arrays (ON by
  default).

+ `EMBREE_GEOMETRY_USER`: Enables support for user-defined geometries
  (ON by default).

+ `EMBREE_GEOMETRY_POINT`: Enables support for point geometries
  (ON by default).

+ `EMBREE_CURVE_SELF_INTERSECTION_AVOIDANCE_FACTOR`: Specifies a
  factor that controls the self-intersection avoidance feature for flat
  curves. Flat curve intersections which are closer than
  curve_radius*`EMBREE_CURVE_SELF_INTERSECTION_AVOIDANCE_FACTOR` to
  the ray origin are ignored. A value of 0.0f disables self-intersection
  avoidance while 2.0f is the default value.

+ `EMBREE_DISC_POINT_SELF_INTERSECTION_AVOIDANCE`: Enables self-intersection
  avoidance for RTC_GEOMETRY_TYPE_DISC_POINT geometry type (ON by default).
  When enabled intersections are skipped if the ray origin lies inside the
  sphere defined by the point primitive.

+ `EMBREE_MIN_WIDTH`: Enabled the min-width feature, which allows
  increasing the radius of curves and points to match some amount of
  pixels. See [rtcSetGeometryMaxRadiusScale] for more details.

+ `EMBREE_MAX_INSTANCE_LEVEL_COUNT`: Specifies the maximum number of nested
  instance levels. Should be greater than 0; the default value is 1.
  Instances nested any deeper than this value will silently disappear in
  release mode, and cause assertions in debug mode.





Embree Tutorials
================

Embree comes with a set of tutorials aimed at helping users understand
how Embree can be used and extended. There is a very basic minimal
that can be compiled as both C and C++, which should get new users started quickly. 
All other tutorials exist in an Intel® ISPC and C++ version to demonstrate 
the two versions of the API. Look for files
named `tutorialname_device.ispc` for the Intel® ISPC implementation of the
tutorial, and files named `tutorialname_device.cpp` for the single ray C++
version of the tutorial. To start the C++ version use the `tutorialname`
executables, to start the Intel® ISPC version use the `tutorialname_ispc`
executables. All tutorials can print available command line options
using the `--help` command line parameter.

For all tutorials except minimal, you can select an initial camera using 
the `--vp` (camera position), `--vi` (camera look-at point), `--vu` 
(camera up vector), and `--fov` (vertical field of view) command line 
parameters:

    ./triangle_geometry --vp 10 10 10 --vi 0 0 0

You can select the initial window size using the `--size` command line
parameter, or start the tutorials in full screen using the `--fullscreen`
parameter:

    ./triangle_geometry --size 1024 1024
    ./triangle_geometry --fullscreen

The initialization string for the Embree device (`rtcNewDevice` call)
can be passed to the ray tracing core through the `--rtcore` command
line parameter, e.g.:

    ./triangle_geometry --rtcore verbose=2,threads=1

The navigation in the interactive display mode follows the camera orbit
model, where the camera revolves around the current center of interest.
With the left mouse button you can rotate around the center of interest
(the point initially set with `--vi`). Holding Control pressed while
clicking the left mouse button rotates the camera around its location.
You can also use the arrow keys for navigation.

You can use the following keys:

F1
:   Default shading

F2
:   Gray EyeLight shading

F3
:   Traces occlusion rays only.

F4
:   UV Coordinate visualization

F5
:   Geometry normal visualization

F6
:   Geometry ID visualization

F7
:   Geometry ID and Primitive ID visualization

F8
:   Simple shading with 16 rays per pixel for benchmarking.

F9
:   Switches to render cost visualization. Pressing again reduces
    brightness.

F10
:   Switches to render cost visualization. Pressing again increases
    brightness.

f
:   Enters or leaves full screen mode.

c
:   Prints camera parameters.

ESC
:   Exits the tutorial.

q
:   Exits the tutorial.

Minimal
-------

This tutorial is designed to get new users started with Embree.
It can be compiled as both C and C++. It demonstrates how to initialize
a device and scene, and how to intersect rays with the scene.
There is no image output to keep the tutorial as simple as possible.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/minimal/minimal.cpp)

Host Device Memory
------------------

This tutorial shows four different ways to use explicit host and device memory
with SYCL.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/host_device_memory/host_device_memory_device.cpp)

Triangle Geometry
-----------------

[![][imgTriangleGeometry]](https://github.com/embree/embree/blob/master/tutorials/triangle_geometry/triangle_geometry_device.cpp)

This tutorial demonstrates the creation of a static cube and ground
plane using triangle meshes. It also demonstrates the use of the
`rtcIntersect1` and `rtcOccluded1` functions to render primary visibility
and hard shadows. The cube sides are colored based on the ID of the hit
primitive.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/triangle_geometry/triangle_geometry_device.cpp)

Dynamic Scene
-------------

[![][imgDynamicScene]](https://github.com/embree/embree/blob/master/tutorials/dynamic_scene/dynamic_scene_device.cpp)

This tutorial demonstrates the creation of a dynamic scene, consisting
of several deforming spheres. Half of the spheres use the
`RTC_BUILD_QUALITY_REFIT` geometry build quality, which allows Embree
to use a refitting strategy for these spheres, the other half uses the
`RTC_BUILD_QUALITY_LOW` geometry build quality, causing a high
performance rebuild of their spatial data structure each frame. The
spheres are colored based on the ID of the hit sphere geometry.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/dynamic_scene/dynamic_scene_device.cpp)

Multi Scene Geometry
-------------

[![][imgDynamicScene]](https://github.com/embree/embree/blob/master/tutorials/multiscene_geometry/multiscene_geometry_device.cpp)

This tutorial demonstrates the creation of multiple scenes sharing the
same geometry objects.  Here, three scenes are built.  One with all
the dynamic spheres of the Dynamic Scene test and two others each with
half.  The ground plane is shared by all three scenes.  The space bar
is used to cycle the scene chosen for rendering.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/multiscene_geometry/multiscene_geometry_device.cpp)

User Geometry
-------------

[![][imgUserGeometry]](https://github.com/embree/embree/blob/master/tutorials/user_geometry/user_geometry_device.cpp)

This tutorial shows the use of user-defined geometry, to re-implement
instancing, and to add analytic spheres. A two-level scene is created,
with a triangle mesh as ground plane, and several user geometries that
instance other scenes with a small number of spheres of different kinds.
The spheres are colored using the instance ID and geometry ID of the hit
sphere, to demonstrate how the same geometry instanced in different
ways can be distinguished.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/user_geometry/user_geometry_device.cpp)

Viewer
------

[![][imgViewer]](https://github.com/embree/embree/blob/master/tutorials/viewer/viewer_device.cpp)

This tutorial demonstrates a simple OBJ viewer that traces primary
visibility rays only. A scene consisting of multiple meshes is created,
each mesh sharing the index and vertex buffer with the application.
It also demonstrates how to support additional per-vertex data, such as
shading normals.

You need to specify an OBJ file at the command line for this tutorial to
work:

    ./viewer -i model.obj

[Source Code](https://github.com/embree/embree/blob/master/tutorials/viewer/viewer_device.cpp)

Intersection Filter
-------------------

[![][imgIntersectionFilter]](https://github.com/embree/embree/blob/master/tutorials/intersection_filter/intersection_filter_device.cpp)

This tutorial demonstrates the use of filter callback functions to
efficiently implement transparent objects. The filter function used for
primary rays lets the ray pass through the geometry if it is entirely
transparent. Otherwise, the shading loop handles the transparency
properly, by potentially shooting secondary rays. The filter function
used for shadow rays accumulates the transparency of all surfaces along
the ray, and terminates traversal if an opaque occluder is hit.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/intersection_filter/intersection_filter_device.cpp)

Instanced Geometry
------------------

[![][imgInstancedGeometry]](https://github.com/embree/embree/blob/master/tutorials/instanced_geometry/instanced_geometry_device.cpp)

This tutorial demonstrates the in-build instancing feature of Embree, by
instancing a number of other scenes built from triangulated spheres. The
spheres are again colored using the instance ID and geometry ID of the
hit sphere, to demonstrate how the same geometry instanced in different
ways can be distinguished.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/instanced_geometry/instanced_geometry_device.cpp)

Instance Array Geometry
----------------------

[![][imgForest]](https://github.com/embree/embree/blob/master/tutorials/forest/forest_device.cpp)

This tutorial demonstrates the usage of instance arrays in Embree. Instance arrays
are large collections of similar objects. Examples are sand dunes that consist
of millions of instances of a few grain models or, like here, a forest consisting of
many instances of a few tree models.

In this application can switch between representing the scene with regular
instances or (one!) instance array. It also prints several stats, that
demonstrate the memory savings and faster BVH build times when using instance
arrays for such scenes. Instance arrays come with a small overhead on CPU and
should be preferred if memory consumption is more important than raytracing
performance.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/forest/forest_device.cpp)

Multi Level Instancing
----------------------

[![][imgMultiLevelInstancing]](https://github.com/embree/embree/blob/master/tutorials/multi_instanced_geometry/multi_instanced_geometry_device.cpp)

This tutorial demonstrates multi-level instancing, i.e., nesting instances
into instances. To enable the tutorial, set the compile-time variable
`EMBREE_MAX_INSTANCE_LEVEL_COUNT` to a value other than the default 1.
This variable is available in the code as `RTC_MAX_INSTANCE_LEVEL_COUNT`.

The renderer uses a basic path tracing approach, and the
image will progressively refine over time.
There are two levels of instances in this scene: multiple instances of
the same tree nest instances of a twig.
Intersections on up to `RTC_MAX_INSTANCE_LEVEL_COUNT` nested levels of
instances work out of the box. Users may obtain the *instance ID stack* for
a given hitpoint from the `instID` member.
During shading, the instance ID stack is used to accumulate
normal transformation matrices for each hit. The tutorial visualizes
transformed normals as colors.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/multi_instanced_geometry/multi_instanced_geometry_device.cpp)

Path Tracer
-----------

[![][imgPathtracer]](https://github.com/embree/embree/blob/master/tutorials/pathtracer/pathtracer_device.cpp)

This tutorial is a simple path tracer, based on the viewer tutorial.

You need to specify an OBJ file and light source at the command line for
this tutorial to work:

    ./pathtracer -i model.obj --ambientlight 1 1 1

As example models we provide the "Austrian Imperial Crown" model by
[Martin Lubich](http://www.loramel.net) and the "Asian Dragon" model from the
[Stanford 3D Scanning Repository](http://graphics.stanford.edu/data/3Dscanrep/).

[crown.zip](https://github.com/embree/models/releases/download/release/crown.zip)

[asian_dragon.zip](https://github.com/embree/models/releases/download/release/asian_dragon.zip)

To render these models execute the following:

    ./pathtracer -c crown/crown.ecs
    ./pathtracer -c asian_dragon/asian_dragon.ecs

[Source Code](https://github.com/embree/embree/blob/master/tutorials/pathtracer/pathtracer_device.cpp)

Hair
----

[![][imgHairGeometry]](https://github.com/embree/embree/blob/master/tutorials/hair_geometry/hair_geometry_device.cpp)

This tutorial demonstrates the use of the hair geometry to render a
hairball.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/hair_geometry/hair_geometry_device.cpp)

Curve Geometry
--------------

[![][imgCurveGeometry]](https://github.com/embree/embree/blob/master/tutorials/curve_geometry/curve_geometry_device.cpp)

This tutorial demonstrates the use of the Linear Basis, B-Spline, and Catmull-Rom curve geometries.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/curve_geometry/curve_geometry_device.cpp)

Subdivision Geometry
--------------------

[![][imgSubdivisionGeometry]](https://github.com/embree/embree/blob/master/tutorials/subdivision_geometry/subdivision_geometry_device.cpp)

This tutorial demonstrates the use of Catmull-Clark subdivision
surfaces.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/subdivision_geometry/subdivision_geometry_device.cpp)

Displacement Geometry
---------------------

[![][imgDisplacementGeometry]](https://github.com/embree/embree/blob/master/tutorials/displacement_geometry/displacement_geometry_device.cpp)

This tutorial demonstrates the use of Catmull-Clark subdivision
surfaces with procedural displacement mapping using a constant edge
tessellation level.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/displacement_geometry/displacement_geometry_device.cpp)

Grid Geometry
---------------------

[![][imgGridGeometry]](https://github.com/embree/embree/tree/master/tutorials/grid_geometry)

This tutorial demonstrates the use of the memory efficient grid
primitive to handle highly tessellated and displaced geometry.

[Source Code](https://github.com/embree/embree/tree/master/tutorials/grid_geometry)

Point Geometry
---------------------

[![][imgPointGeometry]](https://github.com/embree/embree/blob/master/tutorials/point_geometry/point_geometry_device.cpp)

This tutorial demonstrates the use of the three representations
of point geometry.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/point_geometry/point_geometry_device.cpp)

Motion Blur Geometry
--------------------

[![][imgMotionBlurGeometry]](https://github.com/embree/embree/blob/master/tutorials/motion_blur_geometry/motion_blur_geometry_device.cpp)

This tutorial demonstrates rendering of motion blur using the
multi-segment motion blur feature. Shown is motion blur of a triangle mesh,
quad mesh, subdivision surface, line segments, hair geometry, Bézier
curves, instantiated triangle mesh where the instance moves,
instantiated quad mesh where the instance and the quads move, and user
geometry.

The number of time steps used can be configured using the `--time-steps
<int>` and `--time-steps2 <int>` command line parameters, and the
geometry can be rendered at a specific time using the the `--time
<float>` command line parameter.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/motion_blur_geometry/motion_blur_geometry_device.cpp)

Quaternion Motion Blur
----------------------

[![][imgQuaternionMotionBlur]](https://github.com/embree/embree/blob/master/tutorials/quaternion_motion_blur/quaternion_motion_blur_device.cpp)

This tutorial demonstrates rendering of motion blur using quaternion
interpolation. Shown is motion blur using spherical linear interpolation of
the rotational component of the instance transformation on the left and
simple linear interpolation of the instance transformation on the right. The
number of time steps can be modified as well.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/quaternion_motion_blur/quaternion_motion_blur_device.cpp)

Interpolation
-------------

[![][imgInterpolation]](https://github.com/embree/embree/blob/master/tutorials/interpolation/interpolation_device.cpp)

This tutorial demonstrates interpolation of user-defined per-vertex data.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/interpolation/interpolation_device.cpp)

Closest Point
----------------------

[![][imgClosestPoint]](https://github.com/embree/embree/blob/master/tutorials/closest_point/closest_point_device.cpp)

This tutorial demonstrates a use-case of the point query API. The scene
consists of a simple collection of objects that are instanced and for several
point in the scene (red points) the closest point on the surfaces of the
scene are computed (white points). The closest point functionality is
implemented for Embree internal and for user-defined instancing. The tutorial
also illustrates how to handle instance transformations that are not
similarity transforms.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/closest_point/closest_point_device.cpp)

Voronoi
----------------------

[![][imgVoronoi]](https://github.com/embree/embree/blob/master/tutorials/voronoi/voronoi_device.cpp)

This tutorial demonstrates how to implement nearest neighbour lookups using
the point query API. Several colored points are located on a plane and the
corresponding voroni regions are illustrated.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/voronoi/voronoi_device.cpp)

Collision Detection
----------------------

[![][imgCollision]](https://github.com/embree/embree/blob/master/tutorials/collide/collide_device.cpp)

This tutorial demonstrates how to implement collision detection using
the collide API. A simple cloth solver is setup to collide with a sphere.

The cloth can be reset with the `space` bar.  The sim stepped once with `n` 
and continuous simulation started and paused with `p`.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/collide/collide_device.cpp)

BVH Builder
-----------

This tutorial demonstrates how to use the templated hierarchy builders
of Embree to build a bounding volume hierarchy with a user-defined
memory layout using a high-quality SAH builder using spatial splits, a
standard SAH builder, and a very fast Morton builder.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/bvh_builder/bvh_builder_device.cpp)

BVH Access
-----------

This tutorial demonstrates how to access the internal triangle
acceleration structure build by Embree. Please be aware that the
internal Embree data structures might change between Embree updates.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/bvh_access/bvh_access.cpp)

Find Embree
-----------

This tutorial demonstrates how to use the `FIND_PACKAGE` CMake feature
to use an installed Embree. Under Linux and macOS the tutorial finds
the Embree installation automatically, under Windows the `embree_DIR`
CMake variable must be set to the following folder of the Embree
installation: `C:\Program Files\Intel\Embree3`.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/find_embree/CMakeLists.txt)

Next Hit
-----------

This tutorial demonstrates how to robustly enumerate all hits along
the ray using multiple ray queries and an intersection filter
function. To improve performance, the tutorial also supports
collecting the next N hits in a single ray query.

[Source Code](https://github.com/embree/embree/blob/master/tutorials/next_hit/next_hit_device.cpp)



[Embree API]: #embree-api
[Embree Tutorials]: #embree-tutorials
[Ray Layout]: #ray-layout
[Extending the Ray Structure]: #extending-the-ray-structure
[Embree Example Renderer]: https://embree.github.io/renderer.html
[Triangle Geometry]: #triangle-geometry
[Stream Viewer]: #stream-viewer
[User Geometry]: #user-geometry
[Instanced Geometry]: #instanced-geometry
[Instance Array Geometry]: #instance-array-geometry
[Multi Level Instancing]: #multi-level-instancing
[Intersection Filter]: #intersection-filter
[Hair]: #hair
[Curves]: #bézier-curves
[Subdivision Geometry]: #subdivision-geometry
[Displacement Geometry]: #displacement-geometry
[Quaternion Motion Blur]: #quaternion-motion-blur
[BVH Builder]: #bvh-builder
[Interpolation]: #interpolation
[Closest Point]: #closest-point
[Voronoi]: #voronoi
[imgHalfEdges]: https://embree.github.io/images/half_edges.png
[imgTriangleUV]: https://embree.github.io/images/triangle_uv.png
[imgQuadUV]: https://embree.github.io/images/quad_uv.png
[imgTriangleGeometry]: https://embree.github.io/images/triangle_geometry.jpg
[imgDynamicScene]: https://embree.github.io/images/dynamic_scene.jpg
[imgUserGeometry]: https://embree.github.io/images/user_geometry.jpg
[imgViewer]: https://embree.github.io/images/viewer.jpg
[imgInstancedGeometry]: https://embree.github.io/images/instanced_geometry.jpg
[imgForest]: https://embree.github.io/images/forest.jpg
[imgMultiLevelInstancing]: https://embree.github.io/images/multi_level_instancing.jpg
[imgIntersectionFilter]: https://embree.github.io/images/intersection_filter.jpg
[imgPathtracer]: https://embree.github.io/images/pathtracer.jpg
[imgHairGeometry]: https://embree.github.io/images/hair_geometry.jpg
[imgCurveGeometry]: https://embree.github.io/images/curve_geometry.jpg
[imgSubdivisionGeometry]: https://embree.github.io/images/subdivision_geometry.jpg
[imgDisplacementGeometry]: https://embree.github.io/images/displacement_geometry.jpg
[imgGridGeometry]: https://embree.github.io/images/grid_geometry.jpg
[imgPointGeometry]: https://embree.github.io/images/point_geometry.jpg
[imgMotionBlurGeometry]: https://embree.github.io/images/motion_blur_geometry.jpg
[imgQuaternionMotionBlur]: https://embree.github.io/images/quaternion_motion_blur.jpg
[imgInterpolation]: https://embree.github.io/images/interpolation.jpg
[imgClosestPoint]: https://embree.github.io/images/closest_point.jpg
[imgVoronoi]: https://embree.github.io/images/voronoi.jpg
[imgCollision]: https://embree.github.io/images/collide.jpg
