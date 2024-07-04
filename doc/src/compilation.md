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


\pagebreak

