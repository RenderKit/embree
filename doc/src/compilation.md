Compiling Embree
================

Linux and Mac OS\ X
-------------------

Embree is tested with Intel Compiler 15.0.0, CLANG 3.4.2, and
GCC 4.8.2.

Embree also requires the Intel® SPMD Program Compiler
(ISPC) to compile. We have tested ISPC version 1.8.0, but more recent
versions of ISPC should also work. You can download and install the
ISPC binaries from
[ispc.github.io](https://ispc.github.io/downloads.html). After
installation, either put the path to the `ispc` executable permanently
into your `PATH`:

    export PATH=path-to-ispc:$PATH

Or provide the path to the `ispc` executable to CMake via the
`ISPC_EXECUTABLE` variable.

You additionally have to install CMake 2.8.12 or higher and the
developer version of GLUT. Under Mac OS\ X, these dependencies can be
installed using [MacPorts](http://www.macports.org/):

    sudo port install cmake freeglut

Under Linux you can install these dependencies using `yum` or `apt-get`.
Depending on your Linux distribution, some of these packages might
already be installed or might have slightly different names.

Type the following to install the dependencies using `yum`:

    sudo yum install cmake.x86_64
    sudo yum install freeglut.x86_64 freeglut-devel.x86_64
    sudo yum install libXmu.x86_64 libXi.x86_64
    sudo yum install libXmu-devel.x86_64 libXi-devel.x86_64

Type the following to install the dependencies using `apt-get`:

    sudo apt-get install cmake-curses-gui
    sudo apt-get install freeglut3-dev
    sudo apt-get install libxmu-dev libxi-dev

Finally you can compile Embree using CMake. Create a build directory and
execute "ccmake .." inside this directory.

    mkdir build
    cd build
    ccmake ..

This will open a configuration dialog where you can perform various
configurations as described below. After having configured Embree, press
c (for configure) and g (for generate) to generate a Makefile and leave
the configuration. The code can be compiled by executing make.

    make

The executables will be generated inside the build folder. We recommend
to finally install the Embree library and header files on your system:

    sudo make install

If you cannot install Embree on your system (e.g. when you don't have
administrator rights) you need to add embree_root_directory/build to
your `LD_LIBRARY_PATH` (and `SINK_LD_LIBRARY_PATH` in case you want to
use Embree on Xeon Phi™).

The default configuration in the configuration dialog should be
appropriate for most usages. The following table described all
parameters that can be configured:

  ---------------------------- -------------------------------- --------
  Option                       Description                      Default
  ---------------------------- -------------------------------- --------
  BUILD_EMBREE_SHARED_LIB      Build Embree as a shared         ON
                               library.

  BUILD_TUTORIALS              Builds the C++ version of the    ON
                               Embree tutorials.

  BUILD_TUTORIALS_ISPC         Builds the ISPC version of the   ON
                               Embree tutorials.

  CMAKE_BUILD_TYPE             Can be used to switch between    Release
                               Debug mode (Debug) and Release
                               mode (Release).

  COMPILER                     Select either GCC, ICC, or       GCC
                               CLANG as compiler.

  RTCORE_BACKFACE_CULLING      Enables backface culling, i.e.   OFF
                               only surfaces facing a ray can
                               be hit.

  RTCORE_BUFFER_STRIDE         Enables the buffer stride        ON
                               feature.

  RTCORE_INTERSECTION_FILTER   Enables the intersection filter  ON
                               feature.

  RTCORE_RAY_MASK              Enables the ray masking feature. OFF

  RTCORE_SPINLOCKS             Enables faster spinlocks for     OFF
                               some builders.

  RTCORE_RETURN_SUBDIV_NORMAL  Instead of the triangle normal   OFF
                               the ray returns a smooth normal
                               based on evaluating the 
                               subdivision surface patch.

  XEON_ISA                     Select highest supported ISA on  AVX2
                               Xeon™ CPUs (SSE2, SSE3, SSSE3,
                               SSE4.1, SSE4.2, AVX, AVX-I, or
                               AVX2).

  XEON_PHI_ISA                 Enables generation of Xeon Phi™  OFF
                               version of kernels and tutorials
                               (when BUILD_TUTORIALS is ON).
  ---------------------------- -------------------------------- --------
  : CMake build options for Embree.


Xeon Phi™
---------

Embree supports the Xeon Phi™ coprocessor under Linux. To compile Embree
for Xeon Phi you need to enable the `XEON_PHI_ISA` option in CMake and
have the Intel Compiler and the Intel [Manycore Platform Software
Stack](https://software.intel.com/en-us/articles/intel-manycore-platform-software-stack-mpss)
(MPSS) installed.

Enabling the buffer stride feature reduces performance for building
spatial hierarchies on Xeon Phi.

Windows
-------

Embree requires Visual Studio 12 2013 and the Intel SPMD Program
Compiler (ISPC) to compile. We have tested ISPC version 1.8.0, but
more recent versions of ISPC should also work. You can download and
install the ISPC binaries from
[ispc.github.io](https://ispc.github.io/downloads.html). After
installation, put the path to `ispc.exe` permanently into your `PATH`
environment variable or you need to correctly set the
`ISPC_EXECUTABLE` variable during CMake configuration.

You additionally have to install [CMake](http://www.cmake.org/download/)
(version 2.8.12 or higher). Note that you need a native Windows CMake
installation, because CMake under Cygwin cannot generate solution files
for Visual Studio.

### Using the IDE

Run `cmake-gui`, browse to the Embree sources, set the build directory
and click Configure. Now you can select the Generator, e.g. "Visual
Studio 12 2013" for a 32\ bit build or "Visual Studio 12 2013 Win64" for
a 64\ bit build. Most configuration parameters described for the [Linux
build](#linux-and-mac-osx) can be set under Windows as well. Finally,
click "Generate" to create the Visual Studio solution files.

For compilation of Embree under Windows use the generated Visual Studio
solution file `embree.sln`. The solution is by default setup to use the
Microsoft Compiler. You can switch to the Intel Compiler by right
clicking onto the solution in the Solution Explorer and then selecting
the Intel Compiler. We recommend using 64\ bit mode and the Intel
Compiler for best performance.

To build Embree with support for the AVX2 instruction set you need at
least Visual Studio 2013 Update\ 4. When switching to the Intel Compiler
to build with AVX2 you currently need to manually *remove* the switch
`/arch:AVX2` from the `embree_avx2` project, which can be found under
Properties ⇒ C/C++ ⇒ All Options ⇒ Additional Options.

To build all projects of the solution it is recommend to build the CMake
utility project `ALL_BUILD`, which depends on all projects. Using "Build
Solution" would also build all other CMake utility projects (such as
`INSTALL`), which is usually not wanted.

We recommend enabling syntax highlighting for the `.ispc` source and
`.isph` header files. To do so open Visual Studio 2008, go to Tools ⇒
Options ⇒ Text Editor ⇒ File Extension and add the isph and ispc
extension for the "Microsoft Visual C++" editor.

### Using the Command Line

Embree can also be configured and built without the IDE using the Visual
Studio command prompt:

    cd path\to\embree
    mkdir build
    cd build
    cmake -G "Visual Studio 12 2013 Win64" ..
    cmake --build . --config Release

You can also build only some projects with the `--target` switch.
Additional parameters after "`--`" will be passed to `msbuild`. For
example, to build the Embree library in parallel use

    cmake --build . --config Release --target embree -- /m

