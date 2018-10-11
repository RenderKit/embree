% Embree: High Performance Ray Tracing Kernels 3.2.3
% Intel Corporation

Embree Overview
===============

Intel® Embree is a collection of high-performance ray tracing kernels,
developed at Intel. The target users of Intel® Embree are graphics application
engineers who want to improve the performance of their
photo-realistic rendering application by leveraging Embree's
performance-optimized ray tracing kernels. The kernels are optimized
for the latest Intel® processors with support for SSE, AVX, AVX2, and
AVX-512 instructions. Intel® Embree supports runtime code selection to choose
the traversal and build algorithms that best matches the instruction
set of your CPU. We recommend using Intel® Embree through its API to get the
highest benefit from future improvements. Intel® Embree is released as Open
Source under the
[Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0).

Intel® Embree supports applications written with the Intel® SPMD Program
Compiler (ISPC, <https://ispc.github.io/>) by also providing an ISPC
interface to the core ray tracing algorithms. This makes it possible
to write a renderer in ISPC that automatically vectorizes and
leverages SSE, AVX, AVX2, and AVX-512 instructions. ISPC also supports
runtime code selection, thus ISPC will select the best code path for
your application.

Intel® Embree contains algorithms optimized for incoherent workloads (e.g.
Monte Carlo ray tracing algorithms) and coherent workloads
(e.g. primary visibility and hard shadow rays).

The single-ray traversal kernels of Intel® Embree provide high performance
for incoherent workloads and are very easy to integrate into existing
rendering applications. Using the stream kernels, even higher
performance for incoherent rays is possible, but integration might
require significant code changes to the application to use the stream
paradigm. In general for coherent workloads, the stream mode with
coherent flag set gives the best performance.

Intel® Embree also supports dynamic scenes by implementing high-performance
two-level spatial index structure construction algorithms.

In addition to the ray tracing kernels, Intel® Embree provides some
[Embree Tutorials] to demonstrate how to use the
[Embree API].

Supported Platforms
-------------------

Embree supports Windows (32-bit and 64-bit), Linux (64-bit), and macOS
(64-bit). The code compiles with the Intel® Compiler, GCC, Clang,
and the Microsoft Compiler.

Using the Intel® Compiler improves performance by approximately
10%. Performance also varies across different operating
systems, with Linux typically performing best as it supports
transparently transitioning to 2MB pages.

Embree is optimized for Intel CPUs supporting SSE, AVX, AVX2, and
AVX-512 instructions, and requires at least a CPU with support for
SSE2.

Embree Support and Contact
--------------------------

If you encounter bugs please report them via [Embree's GitHub Issue
Tracker](https://github.com/embree/embree/issues).

For questions and feature requests please write us at
<embree_support@intel.com>.

To receive notifications of updates and new features of Embree please
subscribe to the [Embree mailing
list](https://groups.google.com/d/forum/embree/).

Acknowledgements
----------------

This software is based in part on the work of the Independent JPEG Group.

Installation of Embree
======================

Windows MSI Installer
---------------------

You can install the Embree library using the Windows MSI installer
[embree-3.2.3-x64.msi](https://github.com/embree/embree/releases/download/v3.2.3/embree-3.2.3.x64.msi). This
will install the 64-bit Embree version by default in `Program
Files\Intel\Embree v3.2.3 x64`.

You have to set the path to the `bin` folders manually to your `PATH`
environment variable for applications to find Embree.

To compile applications with Embree using CMake, please have a look at
the `find_embree` tutorial. To compile this tutorial, you need to set
the `embree_DIR` CMake variable of this tutorial to `Program
Files\Intel\Embree v3.2.3 x64`.

To uninstall Embree, open `Programs and Features` by clicking the
`Start button`, clicking `Control Panel`, clicking `Programs`, and
then clicking `Programs and Features`. Select `Embree
3.2.3 x64` and uninstall it.

Windows ZIP File
-----------------

Embree is also delivered as a ZIP file
[embree-3.2.3.x64.windows.zip](https://github.com/embree/embree/releases/download/v3.2.3/embree-3.2.3.x64.windows.zip). After
unpacking this ZIP file, you should set the path to the `lib` folder
manually to your `PATH` environment variable for applications to find
Embree. To compile applications with Embree, you also have to set the
`Include Directories` path in Visual Studio to the `include` folder of
the Embree installation.

If you plan to ship Embree with your application, best use the Embree
version from this ZIP file.

Linux RPMs
----------

Uncompress the `tar.gz` file
[embree-3.2.3.x86_64.rpm.tar.gz](https://github.com/embree/embree/releases/download/v3.2.3/embree-3.2.3.x86_64.rpm.tar.gz)
to obtain the individual RPM files:

    tar xzf embree-3.2.3.x86_64.rpm.tar.gz

To install Embree using the RPM packages on your Linux system, type
the following:

    sudo rpm --install embree3-lib-3.2.3-1.x86_64.rpm
    sudo rpm --install embree3-devel-3.2.3-1.noarch.rpm
    sudo rpm --install embree3-examples-3.2.3-1.x86_64.rpm

You also have to install the Intel® Threading Building Blocks (TBB)
using `yum`:

    sudo yum install tbb.x86_64 tbb-devel.x86_64

On Debian-based Linux distributions you first need to convert the RPM
filed into DEB files using the `alien` tool:

    sudo apt-get install alien dpkg-dev debhelper build-essential

    sudo alien embree3-lib-3.2.3-1.x86_64.rpm
    sudo alien embree3-devel-3.2.3-1.noarch.rpm
    sudo alien embree3-examples-3.2.3-1.x86_64.rpm

    sudo dpkg -i embree3-lib_3.2.3-2_amd64.deb
    sudo dpkg -i embree3-devel_3.2.3-2_all.deb
    sudo dpkg -i embree3-examples_3.2.3-2_amd64.deb

Also install the Intel® Threading Building Blocks (TBB) using `apt-get`:

    sudo apt-get install libtbb-dev

Alternatively you can download the latest TBB version from
[https://www.threadingbuildingblocks.org/download](https://www.threadingbuildingblocks.org/download)
and set the `LD_LIBRARY_PATH` environment variable to point
to the TBB library.

Note that the Embree RPMs are linked against the TBB version coming
with CentOS. This older TBB version is missing some features required
to get optimal build performance, and does not support building of
scenes lazily during rendering. To get a full featured Embree, please
install using the `tar.gz` files, which always ship with the latest TBB
version.

Under Linux, Embree is installed by default in the `/usr/lib64` and
`/usr/include` directories. This way applications will find Embree
automatically. The Embree tutorials are installed into the
`/usr/bin/embree3` folder. Specify the full path to
the tutorials to start them.

To uninstall Embree, just execute the following:

    sudo rpm --erase embree3-lib-3.2.3-1.x86_64
    sudo rpm --erase embree3-devel-3.2.3-1.noarch
    sudo rpm --erase embree3-examples-3.2.3-1.x86_64

Linux tar.gz Files
------------------

The Linux version of Embree is also delivered as a `tar.gz` file:
[embree-3.2.3.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v3.2.3/embree-3.2.3.x86_64.linux.tar.gz). Unpack this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the
C shell) to set up the environment properly:

    tar xzf embree-3.2.3.x86_64.linux.tar.gz
    source embree-3.2.3.x86_64.linux/embree-vars.sh

If you want to ship Embree with your application, best use the Embree
version provided in the `tar.gz` file.

We recommend adding a relative `RPATH` to your application that points
to the location where Embree (and TBB) can be found, e.g. `$ORIGIN/../lib`.

macOS PKG Installer
-------------------

To install the Embree library on your macOS system use the
provided package installer inside
[embree-3.2.3.x86_64.dmg](https://github.com/embree/embree/releases/download/v3.2.3/embree-3.2.3.x86_64.dmg). This
will install Embree by default into `/opt/local/lib` and
`/opt/local/include` directories. The Embree tutorials are installed
into the `/Applications/Embree3` directory.

You also have to install the Intel® Threading Building Blocks (TBB)
using [MacPorts](http://www.macports.org/):

    sudo port install tbb

Alternatively you can download the latest TBB version from
[https://www.threadingbuildingblocks.org/download](https://www.threadingbuildingblocks.org/download)
and set the `DYLD_LIBRARY_PATH` environment variable to point
to the TBB library.

To uninstall Embree, execute the uninstaller script
`/Applications/Embree3/uninstall.command`.

macOS tar.gz file
-----------------

The macOS version of Embree is also delivered as a `tar.gz` file:
[embree-3.2.3.x86_64.macosx.tar.gz](https://github.com/embree/embree/releases/download/v3.2.3/embree-3.2.3.x86_64.macosx.tar.gz). Unpack this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the
C shell) to set up the environment properly:

    tar xzf embree-3.2.3.x64.macosx.tar.gz
    source embree-3.2.3.x64.macosx/embree-vars.sh

If you want to ship Embree with your application, please use the Embree
library of the provided `tar.gz` file. The library name of that Embree
library is of the form `@rpath/libembree.3.dylib`
(and similar also for the included TBB library). This ensures that you
can add a relative `RPATH` to your application that points to the location
where Embree (and TBB) can be found, e.g. `@loader_path/../lib`.

Compiling Embree
================

We recommend to use CMake to build Embree. Do not enable fast-math
optimizations; these might break Embree.

Linux and macOS
---------------

To compile Embree you need a modern C++ compiler that supports C++11.
Embree is tested with Intel® Compiler 17.0 (Update 1), Intel®
Compiler 16.0 (Update 1), Clang 3.8.0 (supports AVX2), Clang 4.0.0
(supports AVX512) and GCC 5.4.0. If the GCC that comes with your
Fedora/Red Hat/CentOS distribution is too old then you can run the
provided script `scripts/install_linux_gcc.sh` to locally install a
recent GCC into `$HOME/devtools-2`.

Embree supports using the Intel® Threading Building Blocks (TBB) as the
tasking system. For performance and flexibility reasons we recommend
to use Embree with the Intel® Threading Building Blocks (TBB) and best
also use TBB inside your application. Optionally you can disable TBB
in Embree through the `EMBREE_TASKING_SYSTEM` CMake variable.

Embree supports the Intel® SPMD Program Compiler (ISPC), which allows
straightforward parallelization of an entire renderer. If you do not
want to use ISPC then you can disable `EMBREE_ISPC_SUPPORT` in
CMake. Otherwise, download and install the ISPC binaries (we have
tested ISPC version 1.9.1) from
[ispc.github.io](https://ispc.github.io/downloads.html). After
installation, put the path to `ispc` permanently into your `PATH`
environment variable or you need to correctly set the
`ISPC_EXECUTABLE` variable during CMake configuration.

You additionally have to install CMake 2.8.11 or higher and the developer
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

    make

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

If you cannot install Embree on your system (e.g. when you don't have
administrator rights) you need to add embree_root_directory/build to
your `LD_LIBRARY_PATH`.


Windows
-------

Embree is tested under Windows using the Visual Studio 2017, Visual
Studio 2015 (Update 1) compiler (Win32 and x64), Visual Studio 2013
(Update 5) compiler (Win32 and x64), Intel® Compiler 17.0 (Update 1)
(Win32 and x64), Intel® Compiler 16.0 (Update 1) (Win32 and x64), and
Clang 3.9 (Win32 and x64). Using the Visual Studio 2015 compiler,
Visual Studio 2013 compiler, Intel® Compiler, and Clang you can
compile Embree for AVX2. To compile Embree for AVX-512 you have to use
the Intel® Compiler.

Embree supports using the Intel® Threading Building Blocks (TBB) as the
tasking system. For performance and flexibility reasons we recommend
to use Embree with the Intel® Threading Building Blocks (TBB) and best
also use TBB inside your application. Optionally you can disable TBB
in Embree through the `EMBREE_TASKING_SYSTEM` CMake variable.

Embree will either find the Intel® Threading Building Blocks (TBB)
installation that comes with the Intel® Compiler, or you can install the
binary distribution of TBB directly from
[www.threadingbuildingblocks.org](https://www.threadingbuildingblocks.org/download)
into a folder named `tbb` into your Embree root directory. You also have
to make sure that the libraries `tbb.dll` and `tbb_malloc.dll` can be
found when executing your Embree applications, e.g. by putting the path
to these libraries into your `PATH` environment variable.

Embree supports the Intel® SPMD Program Compiler (ISPC), which allows
straightforward parallelization of an entire renderer. When
installing ISPC, make sure to download an ISPC version from
[ispc.github.io](https://ispc.github.io/downloads.html) that is
compatible with your Visual Studio version. There are two ISPC
versions, one for Visual Studio 2013 and earlier, and one for Visual
Studio 2015 and later. When using the wrong ISPC version you will get
link errors. After installation, put the path to `ispc.exe`
permanently into your `PATH` environment variable or you need to
correctly set the `ISPC_EXECUTABLE` variable during CMake
configuration. We have tested ISPC version 1.9.1. If you do not want
to use ISPC then you can disable `EMBREE_ISPC_SUPPORT` in CMake.

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
compilation set the toolset to "LLVM-vs2013", to use the Intel®
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
you need at least Visual Studio 2013 (Update 4).

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

+ `EMBREE_ISPC_SUPPORT`: Enables ISPC support of Embree. This option
  is ON by default.

+ `EMBREE_STATIC_LIB`: Builds Embree as a static library (OFF by
  default). When using the statically compiled Embree library, you
  have to define ENABLE_STATIC_LIB before including rtcore.h in your
  application. Further multiple static libraries are generated for the
  different ISAs selected (e.g. `embree3.a`, `embree3_sse42.a`,
  `embree3_avx.a`, `embree3_avx2.a`, `embree3_avx512knl.a`,
  `embree3_avx512skx.a`). You have to link these libraries in exactly
  this order of increasing ISA.

+ `EMBREE_IGNORE_CMAKE_CXX_FLAGS`: When enabled, Embree ignores
  default CMAKE_CXX_FLAGS. This option is turned ON by default.

+ `EMBREE_TUTORIALS`: Enables build of Embree tutorials (default ON).

+ `EMBREE_BACKFACE_CULLING`: Enables backface culling, i.e. only
  surfaces facing a ray can be hit. This option is turned OFF by
  default.

+ `EMBREE_FILTER_FUNCTION`: Enables the intersection filter function
  feature (ON by default).

+ `EMBREE_RAY_MASK`: Enables the ray masking feature (OFF by default).

+ `EMBREE_RAY_PACKETS`: Enables ray packet traversal kernels. This
  feature is turned ON by default. When turned on packet traversal is
  used internally and packets passed to rtcIntersect4/8/16 are kept
  intact in callbacks (when the ISA of appropiate width is enabled).

+ `EMBREE_IGNORE_INVALID_RAYS`: Makes code robust against the risk of
  full-tree traversals caused by invalid rays (e.g. rays containing
  INF/NaN as origins). This option is turned OFF by default.

+ `EMBREE_TASKING_SYSTEM`: Chooses between Intel® Threading TBB
  Building Blocks (TBB), Parallel Patterns Library (PPL) (Windows
  only), or an internal tasking system (INTERNAL). By default TBB is
  used.

+ `EMBREE_MAX_ISA`: Select highest supported ISA (SSE2, SSE4.2, AVX,
  AVX2, AVX512KNL, AVX512SKX, or NONE). When set to NONE the
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

+ `EMBREE_ISA_AVX512KNL`: Enables AVX-512 for Xeon Phi when
  EMBREE_MAX_ISA is set to NONE. By default this option is turned OFF.

+ `EMBREE_ISA_AVX512SKX`: Enables AVX-512 for Skylake when
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

+ `EMBREE_CURVE_SELF_INTERSECTION_AVOIDANCE_FACTOR`: Specifies a
  factor that controls the self intersection avoidance feature for flat
  curves. Flat curve intersections which are closer than
  curve_radius*`EMBREE_CURVE_SELF_INTERSECTION_AVOIDANCE_FACTOR` to
  the ray origin are ignored. A value of 0.0f disables self
  intersection avoidance while 2.0f is the default value.


Using Embree
=============

The most convenient way of using Embree is through CMake. Just let
CMake find Embree using the `FIND_PACKAGE` function inside your
`CMakeLists.txt` file:

     FIND_PACKAGE(embree 3.0 REQUIRED)

If you installed Embree using the Linux RPM or macOS PKG installer,
this will automatically find Embree. If you used the `zip` or `tar.gz`
files to extract Embree, you need to set the `embree_DIR` variable to
the folder you extracted Embree to. If you used the Windows MSI
installer, you need to set `embree_DIR` to point to the Embree install
location (e.g. `C:\Program Files\Intel\Embree3`).

The `FIND_PACKAGE` CMake function will set the `EMBREE_INCLUDE_DIRS`
variable to point to the directory containing the Embree headers. You
should add this folder to the include directories of your build:

    INCLUDE_DIRECTORIES(${EMBREE_INCLUDE_DIRS})

Further, the `EMBREE_LIBRARY` variable will point to the Embree
library to link against. Link against Embree the following way:

    TARGET_LINK_LIBRARIES(application ${EMBREE_LIBRARY})

Now please have a look at the [Embree Tutorials] source code and the
[Embree API] section to get started.


Embree API
==========

The Embree API is a low-level C99 ray tracing API which can be used to
construct 3D scenes and perform ray queries of different types inside
these scenes. All API calls carry the prefix `rtc` (or `RTC` for
types) which stands for **r**ay **t**racing **c**ore.

The API also exists in an ISPC version, which is almost identical but
contains additional functions that operate on ray packets with a size
of the native SIMD width used by ISPC. For simplicity this document
refers to the C99 version of the API functions. For changes when
upgrading from the Embree 2 to the current Embree 3 API see Section
[Upgrading from Embree 2 to Embree 3].

The API supports scenes consisting of different geometry types such as
triangle meshes, quad meshes (triangle pairs), grid meshes, flat
curves, round curves, oriented curves, subdivision meshes, instances,
and user-defined geometries. See Section [Scene Object] for more
information.

Finding the closest hit of a ray segment with the scene
(`rtcIntersect`-type functions), and determining whether any hit
between a ray segment and the scene exists (`rtcOccluded`-type
functions) are both supported. The API supports queries for single
rays, ray packets, and ray streams. See Section [Ray Queries] for
more information.

The API is designed in an object-oriented manner, e.g. it contains
device objects (`RTCDevice` type), scene objects (`RTCScene` type),
geometry objects (`RTCGeometry` type), buffer objects (`RTCBuffer`
type), and BVH objects (`RTCBVH` type). All objects are reference
counted, and handles can be released by calling the appropriate release
function (e.g. `rtcReleaseDevice`) or retained by incrementing the
reference count (e.g. `rtcRetainDevice`). In general, API calls that
access the same object are not thread-safe, unless specified
differently. However, attaching geometries to the same scene and
performing ray queries in a scene is thread-safe.

Device Object
-------------

Embree supports a device concept, which allows different components of
the application to use the Embree API without interfering with each
other. An application typically first creates a device using the
[rtcNewDevice] function. This device can then be used to construct
further objects, such as scenes and geometries. Before the application
exits, it should release all devices by invoking [rtcReleaseDevice]. An
application typically creates only a single device. If required
differently, it should only use a small number of devices at any given
time.

Each user thread has its own error flag per device. If an error occurs
when invoking an API function, this flag is set to an error code (if
it isn't already set by a previous error). See Section
[rtcGetDeviceError] for information on how to read the error code
and Section [rtcSetDeviceErrorFunction] on how to register a
callback that is invoked for each error encountered. It is recommended
to always set a error callback function, to detect all errors.

Scene Object
------------

A scene is a container for a set of geometries, and contains a spatial
acceleration structure which can be used to perform different types of
ray queries.

A scene is created using the `rtcNewScene` function call, and released
using the `rtcReleaseScene` function call. To populate a scene with
geometries use the `rtcAttachGeometry` call, and to detach them use the
`rtcDetachGeometry` call. Once all scene geometries are attached, an
`rtcCommitScene` call (or `rtcJoinCommitScene` call) will finish the
scene description and trigger building of internal data structures.
After the scene got committed, it is safe to perform ray queries (see
Section [Ray Queries]) or to query the scene bounding box (see
[rtcGetSceneBounds] and [rtcGetSceneLinearBounds]).

If scene geometries get modified or attached or detached, the
`rtcCommitScene` call must be invoked before performing any further ray
queries for the scene; otherwise the effect of the ray query is
undefined. The modification of a geometry, committing the scene, and
tracing of rays must always happen sequentially, and never at the
same time.

Scene flags can be used to configure a scene to use less memory
(`RTC_SCENE_FLAG_COMPACT`), use more robust traversal algorithms
(`RTC_SCENE_FLAG_ROBUST`), and to optimize for dynamic content. See
Section [rtcSetSceneFlags] for more details.

A build quality can be specified for a scene to balance between
acceleration structure build performance and ray query performance.
See Section [rtcSetSceneBuildQuality] for more details on build
quality.

Geometry Object
---------------

A new geometry is created using the `rtcNewGeometry` function.
Depending on the geometry type, different buffers must be bound (e.g.
using `rtcSetSharedGeometryBuffer`) to set up the geometry data. In
most cases, binding of a vertex and index buffer is required. The
number of primitives and vertices of that geometry is typically
inferred from the size of these bound buffers.

Changes to the geometry always must be committed using the
`rtcCommitGeometry` call before using the geometry. After committing,
a geometry is not included in any scene. A geometry can be added to
a scene by using the `rtcAttachGeometry` function (to automatically
assign a geometry ID) or using the `rtcAttachGeometryById` function
(to specify the geometry ID manually). A geometry can only be attached
to a single scene at a time.

All geometry types support multi-segment motion blur with an arbitrary
number of equidistant time steps (in the range of 2 to 129). Each
geometry can have a different number of time steps. The motion blur
geometry is defined by linearly interpolating the geometries of
neighboring time steps. To construct a motion blur geometry, first the
number of time steps of the geometry must be specified using the
`rtcSetGeometryTimeStepCount` function, and then a vertex buffer for
each time step must be bound, e.g. using the
`rtcSetSharedGeometryBuffer` function.

The API supports per-geometry filter callback functions (see
`rtcSetGeometryIntersectFilterFunction` and
`rtcSetGeometryOccludedFilterFunction`) that are invoked for each
intersection found during the `rtcIntersect`-type or
`rtcOccluded`-type calls. The former ones are called geometry
intersection filter functions, the latter ones geometry occlusion
filter functions. These filter functions are designed to be used to
ignore intersections outside of a user-defined silhouette of a
primitive, e.g. to model tree leaves using transparency textures.

Ray Queries
-----------

The API supports finding the closest hit of a ray segment with the
scene (`rtcIntersect`-type functions), and determining whether any hit
between a ray segment and the scene exists (`rtcOccluded`-type
functions).

Supported are single ray queries (`rtcIntersect1` and `rtcOccluded1`)
as well as ray packet queries for ray packets of size 4
(`rtcIntersect4` and `rtcOccluded4`), ray packets of size 8
(`rtcIntersect8` and `rtcOccluded8`), and ray packets of size 16
(`rtcIntersect16` and `rtcOccluded16`).

Ray streams in a variety of layouts are supported as well, such as
streams of single rays (`rtcIntersect1M` and `rtcOccluded1M`), streams
of pointers to single rays (`rtcIntersect1p` and `rtcOccluded1p`),
streams of ray packets (`rtcIntersectNM` and `rtcOccludedNM`), and
large packet-like streams in structure of pointer layout
(`rtcIntersectNp` and `rtcOccludedNp`).

See Sections [rtcIntersect1] and [rtcOccluded1] for a detailed
description of how to set up and trace a ray.

See tutorial [Triangle Geometry] for a complete example of how to
trace single rays and ray packets. Also have a look at the tutorial
[Stream Viewer] for an example of how to trace ray streams.

Miscellaneous
-------------

A context filter function, which can be set per ray query is supported
(see `rtcInitIntersectContext`). This filter function is designed to
change the semantics of the ray query, e.g. to accumulate opacity for
transparent shadows, count the number of surfaces along a ray,
collect all hits along a ray, etc.

The internal algorithms to build a BVH are exposed through the `RTCBVH`
object and `rtcBuildBVH` call. This call makes it possible to build a
BVH in a user-specified format over user-specified primitives. See the
documentation of the `rtcBuildBVH` call for more details.

For getting the most performance out of Embree, see the Section
[Performance Recommendations].



Upgrading from Embree 2 to Embree 3
===================================

We decided to introduce an improved API in Embree 3 that is not
backward compatible with the Embree 2 API. This step was required to
remove various deprecated API functions that accumulated over time,
improve extensibility of the API, fix suboptimal design decisions,
fix design mistakes (such as incompatible single ray and ray packet
layouts), clean up inconsistent naming, and increase flexibility.

To make porting to the new API easy, we provide a conversion script
that can do most of the work, and will annotate the code with
remaining changes required. The script can be invoked the following
way for CPP files:

    ./scripts/cpp-patch.py --patch embree2_to_embree3.patch
      --in infile.cpp --out outfile.cpp

When invoked for ISPC files, add the `--ispc` option:

    ./scripts/cpp-patch.py --ispc --patch embree2_to_embree3.patch
      --in infile.ispc --out outfile.ispc

Apply the script to each source file of your project that contains
Embree API calls or types. The input file and output file can also be
identical to perform the patch in-place. Please always backup your
original code before running the script, and inspect the code changes
done by the script using diff (e.g. `git diff`), to make sure no
undesired code locations got changed. Grep the code for comments
containing `EMBREE_FIXME` and perform the action described in the
comment.

The following changes need to be performed when switching from Embree
2 to Embree 3. Most of these changes are automatically done by the
script if not described differently.

We strongly recommend to set an error callback function (see
`rtcSetDeviceErrorFunction`) when porting to Embree 3 to detect all
runtime errors early.

Device
------

*   `rtcInit` and `rtcExit` got removed. Please use the device concept
    using the `rtcNewDevice` and `rtcReleaseDevice` functions
    instead.

*   Functions that conceptually should operate on a device but did not
    get a device argument got removed. The upgrade script replaces
    these functions by the proper functions that operate on a device,
    however, manually propagating the device handle to these function
    calls might still be required.

Scene
-----

*   The API no longer distinguishes between a static and a dynamic
    scene. Some users had issues as they wanted to do minor
    modifications to static scenes, but maintain high traversal
    performance.

    The new approach gives more flexibility, as each scene is
    changeable, and build quality settings can be changed on a commit
    basis to balance between build performance and render performance.
   
*   The `rtcCommitThread` function got removed; use
    `rtcJoinCommitScene` instead.

*   The scene now supports different build quality settings. Please use
    those instead of the previous way of `RTC_SCENE_STATIC`,
    `RTC_SCENE_DYNAMIC`, and `RTC_SCENE_HIGH_QUALITY` flags.

Geometry
--------

*   There is now only one `rtcNewGeometry` function to create geometries
    which gets passed an enum to specify the type of geometry to create.
    The number of vertices and primitives of the geometries is inferred
    from the size of data buffers.

*   We introduced an object type `RTCGeometry` for all geometries.
    Previously a geometry was not a standalone object and could only
    exist inside a scene. The new approach comes with more flexibility
    and more readable code.

    Operations like `rtcInterpolate` can now be performed on the
    geometry object directly without the need of a scene. Further, an
    application can choose to create its geometries independent of a
    scene, e.g. each time a geometry node is added to its scene graph.

    This modification changed many API functions to get passed one
    `RTCGeometry` object instead of a `RTCScene` and `geomID`. The
    script does all required changed automatically. However, in some
    cases the script may introduce `rtcGetGeometry(scene, geomID)`
    calls to retrieve the geometry handle. Best store the geometry
    handle inside your scene representation (and release it in the
    destructor) and access the handle directly instead of calling
    `rtcGetGeometry`.

*   Geometries are not included inside a scene anymore but can be
    attached to a single scene using the `rtcAttachGeomety` or
    `rtcAttachGeometryByID` functions.

*   As geometries are separate objects, commit semantics got introduced
    for them too. Thus geometries must be committed through the
    `rtcCommitGeometry` call before getting used. This allows for
    earlier error checking and pre-calculating internal data per
    geometry object.

    Such commit points were previously not required in the Embree 2
    API. The upgrade script attempts to insert the commits
    automatically, but cannot do so properly under all
    circumstances. Thus please check if every `rtcCommitGeometry` call
    inserted by the script is properly placed, and if a
    `rtcCommitGeometry` call is placed after a sequence of
    changes to a geometry.

*   Only the latest version of the previous displacement function call
    (`RTCDisplacementFunc2`) is now supported, and the callback is
    passed as a structure containing all arguments.

*   The deprecated `RTCBoundaryMode` type and `rtcSetBoundaryMode`
    function got removed and replaced by `RTCSubdivisionMode` enum and
    the `rtcSetGeometrySubdivisionMode` function. The script does this
    replacement automatically.

*   Ribbon curves and lines now avoid self-intersections automatically
    The application can be simplified by removing special code paths
    that previously did the self-intersection handling.

*   The previous Embree 2 way of instancing was suboptimal as it
    required user geometries to update the `instID` field of the ray
    differently when used inside an instanced scene or inside a
    top-level scene. The user geometry intersection code now just
    has to copy the `context.instID` field into the `ray.instID`
    field to function properly under all circumstances.

*   The internal instancing code will update the `context.instID` field
    properly when entering or leaving an instance. When instancing is
    implemented manually through user geometries, the code must be
    modified to set the `context.instID` field properly and no longer
    pass `instID` through the ray. This change must done manually
    and cannot be performed by the script.

*   We flipped the direction of the geometry normal to the widely used
    convention that a shape with counter-clockwise layout of vertices
    has the normal pointing upwards (right-hand rule). Most modeling tools
    follow that convention.

    The conversion script does not perform this change, thus if
    required adjust your code to flip `Ng` for triangle, quad, and
    subdivision surfaces.

Buffers
-------

*   With Embree 3 we are introducing explicit `RTCBuffer` objects.
    However, you can still use the short way of sharing buffers with
    Embree through the `rtcSetSharedGeometryBuffer` call.

*   The `rtcMapBuffer` and `rtcUnmapBuffer` API calls were removed, and
    we added the `rtcGetBufferData` call instead.

    Previously the `rtcMapBuffer` call had the semantics of creating
    an internal buffer when no buffer was shared for the corresponding
    buffer slot. These invocations of `rtcMapBuffer` must be
    replaced by an explicit creation of an internally managed buffer
    using the `rtcNewGeometryBuffer` function.

    The upgrade script cannot always detect if the `rtcMapBuffer` call
    would create an internal buffer or just map the buffer pointer.
    Thus check whether the `rtcNewGeometryBuffer` and
    `rtcGetBufferData` calls are correct after the conversion.

*   The `rtcUpdateGeometryBuffer` function now must be called for
    every buffer that got modified by the application. Note that the
    conversion script cannot automatically detect each location where
    a buffer update is now required.

*   The buffer type no longer encodes the time step or user vertex
    buffer index. Now `RTC_VERTEX_BUFFER_TYPE` and additional `slot`
    specifies the vertex buffer for a specific time step, and
    `RTC_USER_VERTEX_BUFFER_TYPE` and additional `slot` specifies a
    vertex attribute.

Miscellaneous
-------------

*   The header files for Embree 3 are now inside the `embree3` folder
    (instead of `embree2` folder) and `libembree.so` is now called
    `libembree3.so` to be able to install multiple Embree versions side
    by side. We made the headers C99 compliant.

*   All API objects are now reference counted with release functions to
    decrement and retain functions to increment the reference count
    (if required).

*   Most callback functions no longer get different arguments as input,
    but a pointer to a structure containing all arguments. This
    results in more readable code, faster callback invocation (as some
    arguments do not change between invocations) and is extensible, as
    new members to the structure can be later added in a backward
    compatible way (if required).

    The conversion script can convert the definition and declaration
    of the old callback functions in most cases. Before running the
    script, make sure that you never type-cast a callback function
    when assigning it (as this has the danger of assigning a callback
    function with a wrong type if the conversion did not detect some
    callbacks as such). If the script does not detect a callback
    function, make sure the argument types match exactly the types in
    the header (e.g. write `const int` instead of `int const` or
    convert the callback manually).

*   An intersection context is now required for each ray query
    invocation. The context should be initialized using the
    `rtcInitIntersectContext` function.

*   The `rtcIntersect`-type functions get as input an `RTCRayHit` type,
    which is similar to before, but has the ray and hit parts split
    into two sub-structures.

    The `rtcOccluded`-type functions get as input an `RTCRay` type,
    which does not contain hit data anymore. When an occlusion is
    found, the `tfar` element of the ray is set to `-inf`.

    Required code changes cannot be done by the upgrade script and
    need to be done manually.

*   The ray layout for single rays and packets of rays had certain
    incompatibilities (alignment of `org` and `dir` for single rays
    caused gaps in the single ray layout that were not in the ray
    packet layout). This issue never showed up because single rays
    and ray packets were separate in the system initially. This layout
    issue is now fixed, and a single ray has the same layout as a ray
    packet of size 1.

*   Previously Embree supported placing additional data at the end of
    the ray structure, and accessing that data inside user geometry
    callbacks and filter callback functions.

    With Embree 3 this is no longer supported, and the ray passed to a
    callback function may be copied to a different memory location. To
    attach additional data to your ray, simply extend the intersection
    context with a pointer to that data.

    This change cannot be done by the script. Further, code will still
    work if you extend the ray as the implementation did not change yet.

*   The ray structure now contains an additional `id` and `flags`
    field. The `id` can be used to store the index of the ray with
    respect to a ray packet or ray stream. The `flags` is reserved for
    future use, and currently must be set to 0.

*   All previous intersection filter callback variants have been
    removed, except for the `RTCFilterFuncN` which gets a varying size
    ray packet as input. The semantics of this filter function type
    have changed from copying the hit on acceptance to clearing the
    ray's valid argument in case of non-acceptance. This way, chaining
    multiple filters is more efficient.

    We kept the guarantee that for `rtcIntersect1/4/8/16` and
    `rtcOccluded1/4/8/16` calls the packet size and ray order will not
    change from the initial size and ordering when entering a filter
    callback.

*   We no longer export ISPC-specific symbols. This has the advantage
    that certain linking issues went away, e.g. it is now possible to
    link an ISPC application compiled for any combination of ISAs, and
    link this to an Embree library compiled with a different set of
    ISAs. Previously the ISAs of the application had to be a subset of
    the ISAs of Embree, and when the user enabled exactly one ISA, they
    had to do this in Embree and the application.

*   We no longer export the ISPC tasking system, which means that the
    application has the responsibility to implement the ISPC tasking
    system itself. ISPC comes with example code on how to do this. This
    change is not performed by the script and must be done manually.

*   Fixed many naming inconsistencies, and changed names of further API
    functions. All these renamings are properly done by the script and
    need no further attention.



Embree API Reference
====================

## rtcNewDevice
``` {include=src/api/rtcNewDevice.md}
```


## rtcRetainDevice
``` {include=src/api/rtcRetainDevice.md}
```


## rtcReleaseDevice
``` {include=src/api/rtcReleaseDevice.md}
```


## rtcGetDeviceProperty
``` {include=src/api/rtcGetDeviceProperty.md}
```


## rtcGetDeviceError
``` {include=src/api/rtcGetDeviceError.md}
```


## rtcSetDeviceErrorFunction
``` {include=src/api/rtcSetDeviceErrorFunction.md}
```


## rtcSetDeviceMemoryMonitorFunction
``` {include=src/api/rtcSetDeviceMemoryMonitorFunction.md}
```


## rtcNewScene
``` {include=src/api/rtcNewScene.md}
```


## rtcRetainScene
``` {include=src/api/rtcRetainScene.md}
```


## rtcReleaseScene
``` {include=src/api/rtcReleaseScene.md}
```


## rtcAttachGeometry
``` {include=src/api/rtcAttachGeometry.md}
```


## rtcAttachGeometryByID
``` {include=src/api/rtcAttachGeometryByID.md}
```


## rtcDetachGeometry
``` {include=src/api/rtcDetachGeometry.md}
```


## rtcGetGeometry
``` {include=src/api/rtcGetGeometry.md}
```


## rtcCommitScene
``` {include=src/api/rtcCommitScene.md}
```


## rtcJoinCommitScene
``` {include=src/api/rtcJoinCommitScene.md}
```


## rtcSetSceneProgressMonitorFunction
``` {include=src/api/rtcSetSceneProgressMonitorFunction.md}
```


## rtcSetSceneBuildQuality
``` {include=src/api/rtcSetSceneBuildQuality.md}
```


## rtcSetSceneFlags
``` {include=src/api/rtcSetSceneFlags.md}
```


## rtcGetSceneFlags
``` {include=src/api/rtcGetSceneFlags.md}
```



## rtcGetSceneBounds
``` {include=src/api/rtcGetSceneBounds.md}
```


## rtcGetSceneLinearBounds
``` {include=src/api/rtcGetSceneLinearBounds.md}
```


## rtcNewGeometry
``` {include=src/api/rtcNewGeometry.md}
```


## RTC_GEOMETRY_TYPE_TRIANGLE
``` {include=src/api/RTC_GEOMETRY_TYPE_TRIANGLE.md}
```


## RTC_GEOMETRY_TYPE_QUAD
``` {include=src/api/RTC_GEOMETRY_TYPE_QUAD.md}
```


## RTC_GEOMETRY_TYPE_GRID
``` {include=src/api/RTC_GEOMETRY_TYPE_GRID.md}
```


## RTC_GEOMETRY_TYPE_SUBDIVISION
``` {include=src/api/RTC_GEOMETRY_TYPE_SUBDIVISION.md}
```


## RTC_GEOMETRY_TYPE_CURVE
``` {include=src/api/RTC_GEOMETRY_TYPE_CURVE.md}
```


## RTC_GEOMETRY_TYPE_USER
``` {include=src/api/RTC_GEOMETRY_TYPE_USER.md}
```


## RTC_GEOMETRY_TYPE_INSTANCE
``` {include=src/api/RTC_GEOMETRY_TYPE_INSTANCE.md}
```


## rtcRetainGeometry
``` {include=src/api/rtcRetainGeometry.md}
```


## rtcReleaseGeometry
``` {include=src/api/rtcReleaseGeometry.md}
```


## rtcCommitGeometry
``` {include=src/api/rtcCommitGeometry.md}
```


## rtcEnableGeometry
``` {include=src/api/rtcEnableGeometry.md}
```


## rtcDisableGeometry
``` {include=src/api/rtcDisableGeometry.md}
```


## rtcSetGeometryTimeStepCount
``` {include=src/api/rtcSetGeometryTimeStepCount.md}
```


## rtcSetGeometryVertexAttributeCount
``` {include=src/api/rtcSetGeometryVertexAttributeCount.md}
```


## rtcSetGeometryMask
``` {include=src/api/rtcSetGeometryMask.md}
```


## rtcSetGeometryBuildQuality
``` {include=src/api/rtcSetGeometryBuildQuality.md}
```


## rtcSetGeometryBuffer
``` {include=src/api/rtcSetGeometryBuffer.md}
```


## rtcSetSharedGeometryBuffer
``` {include=src/api/rtcSetSharedGeometryBuffer.md}
```


## rtcSetNewGeometryBuffer
``` {include=src/api/rtcSetNewGeometryBuffer.md}
```


## rtcGetGeometryBufferData
``` {include=src/api/rtcGetGeometryBufferData.md}
```


## rtcUpdateGeometryBuffer
``` {include=src/api/rtcUpdateGeometryBuffer.md}
```


## rtcSetGeometryIntersectFilterFunction
``` {include=src/api/rtcSetGeometryIntersectFilterFunction.md}
```


## rtcSetGeometryOccludedFilterFunction
``` {include=src/api/rtcSetGeometryOccludedFilterFunction.md}
```


## rtcFilterIntersection
``` {include=src/api/rtcFilterIntersection.md}
```


## rtcFilterOcclusion
``` {include=src/api/rtcFilterOcclusion.md}
```


## rtcSetGeometryUserData
``` {include=src/api/rtcSetGeometryUserData.md}
```


## rtcGetGeometryUserData
``` {include=src/api/rtcGetGeometryUserData.md}
```



## rtcSetGeometryUserPrimitiveCount
``` {include=src/api/rtcSetGeometryUserPrimitiveCount.md}
```


## rtcSetGeometryBoundsFunction
``` {include=src/api/rtcSetGeometryBoundsFunction.md}
```


## rtcSetGeometryIntersectFunction
``` {include=src/api/rtcSetGeometryIntersectFunction.md}
```


## rtcSetGeometryOccludedFunction
``` {include=src/api/rtcSetGeometryOccludedFunction.md}
```



## rtcSetGeometryInstancedScene
``` {include=src/api/rtcSetGeometryInstancedScene.md}
```


## rtcSetGeometryTransform
``` {include=src/api/rtcSetGeometryTransform.md}
```


## rtcGetGeometryTransform
``` {include=src/api/rtcGetGeometryTransform.md}
```



## rtcSetGeometryTessellationRate
``` {include=src/api/rtcSetGeometryTessellationRate.md}
```


## rtcSetGeometryTopologyCount
``` {include=src/api/rtcSetGeometryTopologyCount.md}
```


## rtcSetGeometrySubdivisionMode
``` {include=src/api/rtcSetGeometrySubdivisionMode.md}
```


## rtcSetGeometryVertexAttributeTopology
``` {include=src/api/rtcSetGeometryVertexAttributeTopology.md}
```


## rtcSetGeometryDisplacementFunction
``` {include=src/api/rtcSetGeometryDisplacementFunction.md}
```


## rtcGetGeometryFirstHalfEdge
``` {include=src/api/rtcGetGeometryFirstHalfEdge.md}
```


## rtcGetGeometryFace
``` {include=src/api/rtcGetGeometryFace.md}
```


## rtcGetGeometryNextHalfEdge
``` {include=src/api/rtcGetGeometryNextHalfEdge.md}
```


## rtcGetGeometryPreviousHalfEdge
``` {include=src/api/rtcGetGeometryPreviousHalfEdge.md}
```


## rtcGetGeometryOppositeHalfEdge
``` {include=src/api/rtcGetGeometryOppositeHalfEdge.md}
```


## rtcInterpolate
``` {include=src/api/rtcInterpolate.md}
```


## rtcInterpolateN
``` {include=src/api/rtcInterpolateN.md}
```



## rtcNewBuffer
``` {include=src/api/rtcNewBuffer.md}
```


## rtcNewSharedBuffer
``` {include=src/api/rtcNewSharedBuffer.md}
```


## rtcRetainBuffer
``` {include=src/api/rtcRetainBuffer.md}
```


## rtcReleaseBuffer
``` {include=src/api/rtcReleaseBuffer.md}
```


## rtcGetBufferData
``` {include=src/api/rtcGetBufferData.md}
```


## RTCRay
``` {include=src/api/RTCRay.md}
```


## RTCHit
``` {include=src/api/RTCHit.md}
```


## RTCRayHit
``` {include=src/api/RTCRayHit.md}
```


## RTCRayN
``` {include=src/api/RTCRayN.md}
```


## RTCHitN
``` {include=src/api/RTCHitN.md}
```


## RTCRayHitN
``` {include=src/api/RTCRayHitN.md}
```


## rtcInitIntersectContext
``` {include=src/api/rtcInitIntersectContext.md}
```


## rtcIntersect1
``` {include=src/api/rtcIntersect1.md}
```


## rtcOccluded1
``` {include=src/api/rtcOccluded1.md}
```


## rtcIntersect4/8/16
``` {include=src/api/rtcIntersect4.md}
```


## rtcOccluded4/8/16
``` {include=src/api/rtcOccluded4.md}
```


## rtcIntersect1M
``` {include=src/api/rtcIntersect1M.md}
```


## rtcOccluded1M
``` {include=src/api/rtcOccluded1M.md}
```


## rtcIntersect1Mp
``` {include=src/api/rtcIntersect1Mp.md}
```


## rtcOccluded1Mp
``` {include=src/api/rtcOccluded1Mp.md}
```


## rtcIntersectNM
``` {include=src/api/rtcIntersectNM.md}
```


## rtcOccludedNM
``` {include=src/api/rtcOccludedNM.md}
```


## rtcIntersectNp
``` {include=src/api/rtcIntersectNp.md}
```


## rtcOccludedNp
``` {include=src/api/rtcOccludedNp.md}
```


## rtcNewBVH
``` {include=src/api/rtcNewBVH.md}
```


## rtcRetainBVH
``` {include=src/api/rtcRetainBVH.md}
```


## rtcReleaseBVH
``` {include=src/api/rtcReleaseBVH.md}
```


## rtcBuildBVH
``` {include=src/api/rtcBuildBVH.md}
```


Performance Recommendations
===========================

MXCSR control and status register
---------------------------------

It is strongly recommended to have the `Flush to Zero` and `Denormals
are Zero` mode of the MXCSR control and status register enabled for
each thread before calling the `rtcIntersect`-type and
`rtcOccluded`-type functions. Otherwise, under some circumstances
special handling of denormalized floating point numbers can
significantly reduce application and Embree performance. When using
Embree together with the Intel® Threading Building Blocks, it is
sufficient to execute the following code at the beginning of the
application main thread (before the creation of the
`tbb::task_scheduler_init` object):

    #include <xmmintrin.h>
    #include <pmmintrin.h>
    ...
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

If using a different tasking system, make sure each rendering thread
has the proper mode set.

Thread Creation and Affinity Settings
--------------------------------------

Tasking systems like TBB create worker threads on demand, which will
add a runtime overhead for the very first `rtcCommitScene` call. In
case you want to benchmark the scene build time, you should start the
threads at application startup. You can let Embree start TBB threads
by passing `start_threads=1` to the `cfg` parameter of `rtcNewDevice`.

On machines with a high thread count (e.g. dual-socket Xeon or Xeon
Phi machines), affinitizing TBB worker threads increases build and
rendering performance. You can let Embree affinitize TBB worker
threads by passing `set_affinity=1` to the `cfg` parameter of
`rtcNewDevice`. By default, threads are not affinitized by Embree with
the exception of Xeon Phi Processors where they are affinitized by
default.

All Embree tutorials automatically start and affinitize TBB worker
threads by passing `start_threads=1,set_affinity=1` to `rtcNewDevice`.

Fast Coherent Rays
------------------

For getting the highest performance for highly coherent rays, e.g.
primary or hard shadow rays, it is recommended to use packets or
streams of single rays/packets with setting the
`RTC_INTERSECT_CONTEXT_FLAG_COHERENT` flag in the
`RTCIntersectContext` passed to the `rtcIntersect`/`rtcOccluded`
calls. The total number of rays in a coherent stream of ray packets
should be around 64, e.g. 8 times 8-wide packets, or 4 times 16-wide
packets. The rays inside each packet should be grouped as coherent as
possible.

Huge Page Support
-----------------

It is recommended to use huge pages under Linux to increase rendering
performance. Embree supports 2MB huge pages under Windows, Linux, and
macOS. Under Linux huge page support is enabled by default, and under
Windows and macOS disabled by default. Huge page support can be
enabled in Embree by passing `hugepages=1` to `rtcNewDevice` or
disabled by passing `hugepages=0` to `rtcNewDevice`.

We recommend using 2MB huge pages with Embree under Linux as this
improves ray tracing performance by about 5-10%. Under Windows using
huge pages requires the application to run in elevated mode which is a
security issue, thus likely not an option for most use cases. Under
macOS huge pages are rarely available as memory tends to get quickly
fragmented, thus we do not recommend using huge pages on macOS.

### Huge Pages under Linux

Linux supports transparent huge pages and explicit huge pages. To
enable transparent huge page support under Linux, execute the
following as root:

    echo always > /sys/kernel/mm/transparent_hugepage/enabled

When transparent huge pages are enabled, the kernel tries to merge 4KB
pages to 2MB pages when possible as a background job. Many Linux
distributions have transparent huge pages enabled by default. See the
following webpage for more information on
[transparent huge pages under Linux](https://www.kernel.org/doc/Documentation/vm/transhuge.txt).
In this mode each application, including your rendering application
based on Embree, will automatically tend to use huge pages.

Using transparent huge pages, the transitioning from 4KB to 2MB pages
might take some time. For that reason Embree also supports allocating
2MB pages directly when a huge page pool is configured. Such a pool
can be configured by writing some number of huge pages to allocate to
`/proc/sys/vm/nr_overcommit_hugepages` as root user. E.g. to configure
2GB of address space for huge page allocation, execute the following as
root:

    echo 1000 > /proc/sys/vm/nr_overcommit_hugepages

See the following webpage for more information on [huge pages under
Linux](https://www.kernel.org/doc/Documentation/vm/hugetlbpage.txt).

### Huge Pages under Windows

To use huge pages under Windows, the current user must have the "Lock
pages in memory" (SeLockMemoryPrivilege) assigned. This can be
configured through the "Local Security Policy" application, by adding
a user to "Local Policies" -> "User Rights Assignment" -> "Lock pages
in memory". You have to log out and in again for this change to take
effect.

Further, your application must be executed as an elevated process
("Run as administrator") and the "SeLockMemoryPrivilege" must be
explicitly enabled by your application. Example code on how to
enable this privilege can be found in the "common/sys/alloc.cpp" file
of Embree. Alternatively, Embree will try to enable this privilege
when passing `enable_selockmemoryprivilege=1` to `rtcNewDevice`.
Further, huge pages should be enabled in Embree by passing
`hugepages=1` to `rtcNewDevice`.

When the system has been running for a while, physical memory gets
fragmented, which can slow down the allocation of huge pages
significantly under Windows.

### Huge Pages under macOS

To use huge pages under macOS you have to pass `hugepages=1` to
`rtcNewDevice` to enable that feature in Embree.

When the system has been running for a while, physical memory gets
quickly fragmented, and causes huge page allocations to fail. For this
reason, huge pages are not very useful under macOS in practice.

## Avoid store-to-load forwarding issues with single rays

We recommend to use a single SSE store to set up the `org` and `tnear`
components, and a single SSE store to set up the `dir` and `time`
components of a single ray (`RTCRay` type). Storing these values using
scalar stores causes a store-to-load forwarding penalty because Embree
is reading these components using SSE loads later on.


Embree Tutorials
================

Embree comes with a set of tutorials aimed at helping users understand
how Embree can be used and extended. All tutorials exist in an ISPC and
C++ version to demonstrate the two versions of the API. Look for files
named `tutorialname_device.ispc` for the ISPC implementation of the
tutorial, and files named `tutorialname_device.cpp` for the single ray C++
version of the tutorial. To start the C++ version use the `tutorialname`
executables, to start the ISPC version use the `tutorialname_ispc`
executables. All tutorials can print available command line options
using the `--help` command line parameter.

For all tutorials, you can select an initial camera using the `--vp`
(camera position), `--vi` (camera look-at point), `--vu` (camera up
vector), and `--fov` (vertical field of view) command line parameters:

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

Triangle Geometry
-----------------

![][imgTriangleGeometry]

This tutorial demonstrates the creation of a static cube and ground
plane using triangle meshes. It also demonstrates the use of the
`rtcIntersect1` and `rtcOccluded1` functions to render primary visibility
and hard shadows. The cube sides are colored based on the ID of the hit
primitive.

Dynamic Scene
-------------

![][imgDynamicScene]

This tutorial demonstrates the creation of a dynamic scene, consisting
of several deforming spheres. Half of the spheres use the
`RTC_BUILD_QUALITY_REFIT` geometry build quality, which allows Embree
to use a refitting strategy for these spheres, the other half uses the
`RTC_BUILD_QUALITY_LOW` geometry build quality, causing a high
performance rebuild of their spatial data structure each frame. The
spheres are colored based on the ID of the hit sphere geometry.

User Geometry
-------------

![][imgUserGeometry]

This tutorial shows the use of user-defined geometry, to re-implement
instancing, and to add analytic spheres. A two-level scene is created,
with a triangle mesh as ground plane, and several user geometries that
instance other scenes with a small number of spheres of different kinds.
The spheres are colored using the instance ID and geometry ID of the hit
sphere, to demonstrate how the same geometry instanced in different
ways can be distinguished.

Viewer
------

![][imgViewer]

This tutorial demonstrates a simple OBJ viewer that traces primary
visibility rays only. A scene consisting of multiple meshes is created,
each mesh sharing the index and vertex buffer with the application.
It also demonstrates how to support additional per-vertex data, such as
shading normals.

You need to specify an OBJ file at the command line for this tutorial to
work:

    ./viewer -i model.obj

Stream Viewer
-------------

![][imgViewerStream]

This tutorial is a simple OBJ viewer that demonstrates the use of ray
streams. You need to specify an OBJ file at the command line for this
tutorial to work:

    ./viewer_stream -i model.obj

Instanced Geometry
------------------

![][imgInstancedGeometry]

This tutorial demonstrates the in-build instancing feature of Embree, by
instancing a number of other scenes built from triangulated spheres. The
spheres are again colored using the instance ID and geometry ID of the
hit sphere, to demonstrate how the same geometry instanced in different
ways can be distinguished.

Intersection Filter
-------------------

![][imgIntersectionFilter]

This tutorial demonstrates the use of filter callback functions to
efficiently implement transparent objects. The filter function used for
primary rays lets the ray pass through the geometry if it is entirely
transparent. Otherwise, the shading loop handles the transparency
properly, by potentially shooting secondary rays. The filter function
used for shadow rays accumulates the transparency of all surfaces along
the ray, and terminates traversal if an opaque occluder is hit.

Path Tracer
-----------

![][imgPathtracer]

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

Hair
----

![][imgHairGeometry]

This tutorial demonstrates the use of the hair geometry to render a
hairball.

Bézier Curves
-------------

![][imgCurveGeometry]

This tutorial demonstrates the use of the Bézier curve geometry.

Subdivision Geometry
--------------------

![][imgSubdivisionGeometry]

This tutorial demonstrates the use of Catmull-Clark subdivision
surfaces.

Displacement Geometry
---------------------

![][imgDisplacementGeometry]

This tutorial demonstrates the use of Catmull-Clark subdivision
surfaces with procedural displacement mapping using a constant edge
tessellation level.

Grid Geometry
---------------------

![][imgGridGeometry]

This tutorial demonstrates the use of the memory efficient grid
primitive to handle highly tessellated and displaced geometry.


Motion Blur Geometry
--------------------

![][imgMotionBlurGeometry]

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

Interpolation
-------------

![][imgInterpolation]

This tutorial demonstrates interpolation of user-defined per-vertex data.

BVH Builder
-----------

This tutorial demonstrates how to use the templated hierarchy builders
of Embree to build a bounding volume hierarchy with a user-defined
memory layout using a high-quality SAH builder using spatial splits, a
standard SAH builder, and a very fast Morton builder.

BVH Access
-----------

This tutorial demonstrates how to access the internal triangle
acceleration structure build by Embree. Please be aware that the
internal Embree data structures might change between Embree updates.

Find Embree
-----------

This tutorial demonstrates how to use the `FIND_PACKAGE` CMake feature
to use an installed Embree. Under Linux and macOS the tutorial finds
the Embree installation automatically, under Windows the `embree_DIR`
CMake variable must be set to the following folder of the Embree
installation: `C:\Program Files\Intel\Embree3`.

[Embree API]: #embree-api
[Embree Tutorials]: #embree-tutorials
[Ray Layout]: #ray-layout
[Extending the Ray Structure]: #extending-the-ray-structure
[Embree Example Renderer]: https://embree.github.io/renderer.html
[Triangle Geometry]: #triangle-geometry
[Stream Viewer]: #stream-viewer
[User Geometry]: #user-geometry
[Instanced Geometry]: #instanced-geometry
[Intersection Filter]: #intersection-filter
[Hair]: #hair
[Curves]: #bézier-curves
[Subdivision Geometry]: #subdivision-geometry
[Displacement Geometry]: #displacement-geometry
[BVH Builder]: #bvh-builder
[Interpolation]: #interpolation
[imgHalfEdges]: https://embree.github.io/images/half_edges.png
[imgTriangleUV]: https://embree.github.io/images/triangle_uv.png
[imgQuadUV]: https://embree.github.io/images/quad_uv.png
[imgTriangleGeometry]: https://embree.github.io/images/triangle_geometry.jpg
[imgDynamicScene]: https://embree.github.io/images/dynamic_scene.jpg
[imgUserGeometry]: https://embree.github.io/images/user_geometry.jpg
[imgViewer]: https://embree.github.io/images/viewer.jpg
[imgViewerStream]: https://embree.github.io/images/viewer_stream.jpg
[imgInstancedGeometry]: https://embree.github.io/images/instanced_geometry.jpg
[imgIntersectionFilter]: https://embree.github.io/images/intersection_filter.jpg
[imgPathtracer]: https://embree.github.io/images/pathtracer.jpg
[imgHairGeometry]: https://embree.github.io/images/hair_geometry.jpg
[imgCurveGeometry]: https://embree.github.io/images/curve_geometry.jpg
[imgSubdivisionGeometry]: https://embree.github.io/images/subdivision_geometry.jpg
[imgDisplacementGeometry]: https://embree.github.io/images/displacement_geometry.jpg
[imgGridGeometry]: https://embree.github.io/images/grid_geometry.jpg
[imgMotionBlurGeometry]: https://embree.github.io/images/motion_blur_geometry.jpg
[imgInterpolation]: https://embree.github.io/images/interpolation.jpg
