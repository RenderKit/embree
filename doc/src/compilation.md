Compiling Embree
================

We recommend to use CMake to build Embree. Do not enable fast-math
optimizations; these might break Embree.

Linux and macOS
---------------

To compile Embree you need a modern C++ compiler that supports
C++11. Embree is tested with the following compilers:

Linux

  - Intel® oneAPI DPC++/C++ Compiler 2022.0.0
  - Intel® Compiler 2020 Update 1
  - Intel® Compiler 2019 Update 4
  - Intel® Compiler 2017 Update 1
  - Intel® Compiler 2016 Update 3
  - Clang 5.0.0
  - Clang 4.0.0
  - GCC 10.0.1 (Fedora 32) AVX512 support
  - GCC  8.3.1 (Fedora 28) AVX512 support
  - GCC  7.3.1 (Fedora 27) AVX2 support
  - GCC  7.3.1 (Fedora 26) AVX2 support
  - GCC  6.4.1 (Fedora 25) AVX2 support

macOS x86

  - Intel® Compiler 2020 Update 1
  - Intel® Compiler 2019 Update 4
  - Apple LLVM 10.0.1 (macOS 10.14.6)

macOS M1

  - Apple Clang 12.0.0

Embree supports using the Intel® Threading Building Blocks (TBB) as the
tasking system. For performance and flexibility reasons we recommend
to use Embree with the Intel® Threading Building Blocks (TBB) and best
also use TBB inside your application. Optionally you can disable TBB
in Embree through the `EMBREE_TASKING_SYSTEM` CMake variable.

Embree supports the Intel® Implicit SPMD Program Compiler (Intel® ISPC), which allows
straightforward parallelization of an entire renderer. If you do not
want to use Intel® ISPC then you can disable `EMBREE_ISPC_SUPPORT` in
CMake. Otherwise, download and install the Intel® ISPC binaries (we have
tested Intel® ISPC version 1.9.1) from
[ispc.github.io](https://ispc.github.io/downloads.html). After
installation, put the path to `ispc` permanently into your `PATH`
environment variable or you need to correctly set the
`EMBREE_ISPC_EXECUTABLE` variable during CMake configuration.

You additionally have to install CMake 3.1.0 or higher and the developer
version of GLUT.

Under macOS, all these dependencies can be installed
using [MacPorts](http://www.macports.org/):

    sudo port install cmake tbb-devel glfw-devel

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

Finally you can compile Embree using CMake. Create a build directory
inside the Embree root directory and execute `ccmake ..` inside this
build directory.

    mkdir build
    cd build
    ccmake ..

Per default CMake will use the compilers specified with the `CC` and
`CXX` environment variables. Should you want to use a different
compiler, run `cmake` first and set the `CMAKE_CXX_COMPILER` and
`CMAKE_C_COMPILER` variables to the desired compiler. For example, to
use the Intel® Compiler instead of the default GCC on most Linux machines
(`g++` and `gcc`), execute

    cmake -DCMAKE_CXX_COMPILER=icpc -DCMAKE_C_COMPILER=icc ..

Similarly, to use Clang set the variables to `clang++` and `clang`,
respectively. Note that the compiler variables cannot be changed anymore
after the first run of `cmake` or `ccmake`.

Running `ccmake` will open a dialog where you can perform various
configurations as described below in [CMake Configuration]. After having
configured Embree, press `c` (for configure) and `g` (for generate) to
generate a Makefile and leave the configuration. The code can be
compiled by executing make.

    make -j

The executables will be generated inside the build folder. We recommend
to finally install the Embree library and header files on your
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

with DPC++ under Linux
----------------------

The Embree DPC++ compilation under Linux has been tested with the following DPC++ compilers:

  - [oneAPI DPC++ compiler 2022.04.18](https://github.com/intel/llvm/releases/download/sycl-nightly%2F20220418/dpcpp-compiler.tar.gz)
  
Please download and install one of these compilers. E.g. to install
the oneAPI DPC++ compiler 2022.04.18 compiler:

    wget https://github.com/intel/llvm/releases/download/sycl-nightly%2F20220418/dpcpp-compiler.tar.gz
    tar xzf dpcpp-compiler.tar.gz
    source ./dpcpp_compiler/startup.sh

Please also install all Linux packages described in the previous
section.

Now you can compile Embree using CMake. Create a build directory
inside the Embree root directory and execute `cmake ..` inside this
build directory.

    mkdir build
    cd build
    cmake -DCMAKE_CXX_COMPILER=clang++ \
          -DCMAKE_C_COMPILER=clang \
          -DEMBREE_ISPC_SUPPORT=OFF \
          -DEMBREE_DPCPP_SUPPORT=ON \
          -DEMBREE_DPCPP_AOT_DEVICES=dg2 ..
           

The `startup.sh` script above did put the DPC++ version of `clang++`
and `clang` into your path, thus the CMake compiler selection above
configures the Embree project to use the just installed DPC++
compiler.

We disable ISPC with `EMBREE_ISPC_SUPPORT=OFF` and turn on DPC++
support through `EMBREE_DPCPP_SUPPORT=ON`.

Under Linux, code generated with JIT compilation is not functioning at
the moment, thus AOT compilation for the DG2 device has to get enabled
as a workaround using the `EMBREE_DPCPP_AOT_DEVICES=dg2` cmake
setting. For AOT compilation to work you have to install the
[Linux HPG/HPC Driver Installation] section.

Now you can compile the Embree code:

    cmake --build . -j

The executables will be generated inside the build folder. The
executable names of the DPC++ versions of the tutorials end with
`_sycl`.


### Linux HPG/HPC Driver Installation

To run the SYCL code, but also for AOT compilation, you need to
install the KMD and UMD drivers of your Intel Xe HPG/HPC GPUs for one
of the supported Linux distributions:

  - Ubuntu 20.04
  - SLES 15SP3
  - RHEL 8.5

Latest drivers can get found on [intel.com/sdp](https://intel.com/sdp)
searching for the latest `Xe HPC Family Driver FW and Tools`
package. Follow the driver installation instructions. We tested Embree
with the `Xe HPC Family Driver FW and Tools #11` driver dated
2022-03-29.


Windows
-------
        
Embree is tested using the following compilers under Windows:

  - Visual Studio 2019
  - Visual Studio 2017
  - Visual Studio 2015 (Update\ 1)
  - Intel® oneAPI DPC++/C++ Compiler 2022.0.0
  - Intel® Compiler 2019 Update 6
  - Intel® Compiler 2017 Update 8
  - LLVM Clang 9.0.0

To compile Embree for AVX-512 you have to use the Intel® Compiler.

Embree supports using the Intel® Threading Building Blocks (TBB) as the
tasking system. For performance and flexibility reasons we recommend
to use Embree with the Intel® Threading Building Blocks (TBB) and best
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

Embree supports the Intel® Implicit SPMD Program Compiler (Intel® ISPC), which allows
straightforward parallelization of an entire renderer. When installing
Intel® ISPC, make sure to download an Intel® ISPC version from
[ispc.github.io](https://ispc.github.io/downloads.html) that is
compatible with your Visual Studio version. After installation, put
the path to `ispc.exe` permanently into your `PATH` environment
variable or you need to correctly set the `EMBREE_ISPC_EXECUTABLE` variable
during CMake configuration. If you do not want to use Intel® ISPC then you
can disable `EMBREE_ISPC_SUPPORT` in CMake.

We have tested Embree with the following Intel® ISPC versions:

  - Intel® ISPC 1.14.1
  - Intel® ISPC 1.13.0
  - Intel® ISPC 1.12.0
  - Intel® ISPC 1.9.2

You additionally have to install [CMake](http://www.cmake.org/download/)
(version 2.8.11 or higher). Note that you need a native Windows CMake
installation, because CMake under Cygwin cannot generate solution files
for Visual Studio.

### Using the IDE

Run `cmake-gui`, browse to the Embree sources, set the build directory
and click Configure. Now you can select the Generator, e.g. "Visual
Studio 12 2013" for a 32-bit build or "Visual Studio 12 2013 Win64"
for a 64-bit build.

To use a different compiler than the Microsoft Visual C++ compiler, you
additionally need to specify the proper compiler toolset through the
option "Optional toolset to use (-T parameter)". E.g. to use Clang for
compilation set the toolset to "LLVM_v142", to use the Intel®
Compiler 2017 for compilation set the toolset to "Intel C++
Compiler 17.0".

Do not change the toolset manually in a solution file (neither through
the project properties dialog, nor through the "Use Intel Compiler"
project context menu), because then some compiler specific command line
options cannot be set by CMake.

Most configuration parameters described in the [CMake Configuration]
can be set under Windows as well. Finally, click "Generate" to create
the Visual Studio solution files. 

The following CMake options are only available under Windows:

+ `CMAKE_CONFIGURATION_TYPE`:  List of generated
  configurations. Default value is Debug;Release;RelWithDebInfo.

+  `USE_STATIC_RUNTIME`: Use the static version of the C/C++ runtime
  library. This option is turned OFF by default.

Use the generated Visual Studio solution file `embree2.sln` to compile
the project. To build Embree with support for the AVX2 instruction set
you need at least Visual Studio 2013 (Update\ 4).

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
    cmake -G "Visual Studio 12 2013 Win64" ..
    cmake --build . --config Release

To use the Intel® Compiler, set the proper toolset, e.g. for Intel
Compiler 17.0:

    cmake -G "Visual Studio 12 2013 Win64" -T "Intel C++ Compiler 17.0" ..
    cmake --build . --config Release

You can also build only some projects with the `--target` switch.
Additional parameters after "`--`" will be passed to `msbuild`. For
example, to build the Embree library in parallel use

    cmake --build . --config Release --target embree -- /m


with DPC++ under Windows
------------------------

The Embree DPC++ compilation under Windows has been tested with the following DPC++ compilers:

  - [oneAPI DPC++ compiler 165 from 2022.03.10](https://github.com/intel/llvm/suites/5602828495/artifacts/182002768)
  
Please download and install one of these compilers.

Then open some "x64 Native Tools Command Prompt" of Visual Studio and
execute the following line with properly configures the environment to
use the DPC++ compiler:

    set "DPCPP_DIR=path_to_dpcpp_compiler"
    set "PATH=%PATH%;%DPCPP_DIR%\bin"
    set "PATH=%PATH%;%DPCPP_DIR%\lib"
    set "CPATH=%DPCPP_DIR%\include;%CPATH%"
    set "INCLUDE=%DPCPP_DIR%\include;%INCLUDE%"
    set "LIB=%DPCPP_DIR%\lib;%LIB%"

The `path_to_dpcpp_compiler` should point to your unpacked DPC++
compiler.

Now you can configure Embree using CMake. Create a build directory
inside the Embree root directory and execute `cmake ..` inside this
build directory.

    mkdir build
    cd build

    cmake -G Ninja
          -D CMAKE_BUILD_TYPE=Release
          -D CMAKE_CXX_COMPILER=clang++
          -D CMAKE_C_COMPILER=clang
          -D CMAKE_CXX_FLAGS=-fuse-ld=link
          -D CMAKE_C_FLAGS=-fuse-ld=link
          -D EMBREE_MAX_ISA=SSE2
          -D EMBREE_ISPC_SUPPORT=OFF
          -D EMBREE_DPCPP_SUPPORT=ON
          -D EMBREE_DPCPP_AOT_DEVICES=none
          -D EMBREE_FILTER_FUNCTION_IN_GEOMETRY=OFF
          -D EMBREE_GEOMETRY_USER_IN_GEOMETRY=OFF 
          -D TBB_ROOT=path_to_tbb\lib\cmake\tbb ..

This uses the Ninja generator which is required for the DPC++
compiler, and configures a release build with using `clang++` and
`clang` from the just installed DPC++ compiler. The LLVM linker
`lld-link` is selected by Ninja by default, but that linker is not
shipped with the DPC++ compiler. Thus linker has to get force to
the Visual Studio Linker `link` using compilation flags.

We also enable DPC++ support in Embree using the
`EMBREE_DPCPP_SUPPORT` CMake option, and only enable just in time
compilation (JIT compilation) by setting `EMBREE_DPCPP_AOT_DEVICES` to
`none`. Ahead of time compilation (AOT compilation) is currently not
working under Windows.

Support to store filter function pointers and user geometry callbacks
inside the geometry object are disabled, as these feature causes low
performance.

Now you can build Embree:

    cmake --build .

If you have problems with Ninja re-running CMake in an infinite loop,
then first remove the "Re-run CMake if any of its inputs changed."
section from the `build.ninja` file and run the above command again.

You can also create an Embree package using the following command:

    cmake --build . --target package

Please see the [Building Embree Applications] section on how to build
your application with such an Embree package.


### Windows HPG Driver Installation

In order to run the DPC++/SYCL tutorials on HPG hardware, you first
need to install the proper graphics drivers. Latest drivers can get
found on [intel.com/sdp](https://intel.com/sdp) searching for the
latest `Discrete Graphics2 (DG2) FRD Kit: DGD25KEF1Q Resource Drive -
Qual` package. Follow the driver installation instructions. We tested
Embree with driver dated 2022-03-22.


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
  is ON by default.

+ `EMBREE_DPCPP_SUPPORT`: Enables GPU support using DPC++. When this
  option is enabled you have to use some DPC++ compiler. Please see
  section [Linux DPC++ Compilation] and [Windows DPC++ Compilation]
  on supported DPC++ compilers.

+ `EMBREE_DPCPP_AOT_DEVICES`: Selects a list of Xe GPU devices for
  ahead of time (AOT) compilation of GPU code. Possible values are
  either, "none" which enables only just in time (JIT) compilation, or
  specifying one of the Embree supported Xe GPUs for AOT compilation:

  * dg2         : Xe HPG devices
  * XE_HPG_CORE : Xe HPG devices
  * pvc         : Xe HPC devices
  * XE_HPC_CORE : Xe HPC devices

  One can also specify multiple Xe devices separated by comma to
  compile ahead of time for multiple devices, e.g. "dg2,pvc". When
  enabling AOT compilation for one or multiple Xe devices, JIT
  compilation will always additionally be enabled in case the code is
  started on a device no code is precompiled for.

  Execute "ocloc compile --help" for more details of possible devices
  to pass. Embree is only supported on Xe HPG/HPC and newer devices.

  Per default this option is set to "none" under Windows, and "dg2"
  under Linux.

+ `EMBREE_STATIC_LIB`: Builds Embree as a static library (OFF by
  default). Further multiple static libraries are generated for the
  different ISAs selected (e.g. `embree4.a`, `embree4_sse42.a`,
  `embree4_avx.a`, `embree4_avx2.a`, `embree4_avx512.a`). You have
  to link these libraries in exactly this order of increasing ISA.

+ `EMBREE_API_NAMESPACE`: Specifies a namespace name to put all Embree
  API symbols inside. By default no namespace is used and plain C symbols
  exported.

+ `EMBREE_LIBRARY_NAME`: Specifies the name of the Embree library file
  created. By default the name embree4 is used.

+ `EMBREE_IGNORE_CMAKE_CXX_FLAGS`: When enabled, Embree ignores
  default CMAKE_CXX_FLAGS. This option is turned ON by default.

+ `EMBREE_TUTORIALS`: Enables build of Embree tutorials (default ON).

+ `EMBREE_BACKFACE_CULLING`: Enables backface culling, i.e. only
  surfaces facing a ray can be hit. This option is turned OFF by
  default.

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
  only), or an internal tasking system (INTERNAL). By default TBB is
  used.

+ `EMBREE_TBB_ROOT`: If Intel® Threading Building Blocks (TBB)
  is used as a tasking system, search the library in this directory
  tree.

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
  default the option is set to AVX2.

+ `EMBREE_ISA_SSE2`: Enables SSE2 when EMBREE_MAX_ISA is set to
  NONE. By default this option is turned OFF.

+ `EMBREE_ISA_SSE42`: Enables SSE4.2 when EMBREE_MAX_ISA is set to
  NONE. By default this option is turned OFF.

+ `EMBREE_ISA_AVX`: Enables AVX when EMBREE_MAX_ISA is set to NONE. By
  default this option is turned OFF.

+ `EMBREE_ISA_AVX2`: Enables AVX2 when EMBREE_MAX_ISA is set to
  NONE. By default this option is turned OFF.

+ `EMBREE_ISA_AVX512`: Enables AVX-512 for Skylake when
  EMBREE_MAX_ISA is set to NONE. By default this option is turned OFF.

+ `EMBREE_GEOMETRY_TRIANGLE`: Enables support for trianglegeometries
  (ON by default).

+ `EMBREE_GEOMETRY_QUAD`: Enables support for quad geometries (ON by
  default).

+ `EMBREE_GEOMETRY_CURVE`: Enables support for curve geometries (ON by
  default).

+ `EMBREE_GEOMETRY_SUBDIVISION`: Enables support for subdivision
  geometries (ON by default).

+ `EMBREE_GEOMETRY_INSTANCE`: Enables support for instances (ON by
  default).

+ `EMBREE_GEOMETRY_USER`: Enables support for user defined geometries
  (ON by default).

+ `EMBREE_GEOMETRY_POINT`: Enables support for point geometries
  (ON by default).

+ `EMBREE_CURVE_SELF_INTERSECTION_AVOIDANCE_FACTOR`: Specifies a
  factor that controls the self intersection avoidance feature for flat
  curves. Flat curve intersections which are closer than
  curve_radius*`EMBREE_CURVE_SELF_INTERSECTION_AVOIDANCE_FACTOR` to
  the ray origin are ignored. A value of 0.0f disables self
  intersection avoidance while 2.0f is the default value.

+ `EMBREE_MIN_WIDTH`: Enabled the min-width feature, which allows
  increasing the radius of curves and points to match some amount of
  pixels. See [rtcSetGeometryMaxRadiusScale] for more details.

+ `EMBREE_MAX_INSTANCE_LEVEL_COUNT`: Specifies the maximum number of nested
  instance levels. Should be greater than 0; the default value is 1.
  Instances nested any deeper than this value will silently disappear in
  release mode, and cause assertions in debug mode.


\pagebreak

