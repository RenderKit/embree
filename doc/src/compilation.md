Compiling Embree
================

Linux and Mac OS\ X
-------------------

Embree requires the Intel SPMD Compiler (ISPC) to compile. We have
tested ISPC version 1.6.0 and 1.7.0, but more recent versions of ISPC
should also work. You can download and install the ISPC binaries from
[ispc.github.com](http://ispc.github.com/downloads.html). After
installation, put the path to the ispc executable permanently into your
PATH.

    export PATH=path-to-ispc:$PATH

You additionally have to install CMake and the developer version of
GLUT. Under Mac OS\ X, these dependencies can be installed using
[MacPorts](http://www.macports.org/):

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
your LD_LIBRARY_PATH (and SINK_LD_LIBRARY_PATH in case you want to use
Embree on Xeon Phi™).

The default configuration in the configuration dialog should be
appropiate for most usages. The following table described all parameters
that can be configured:

  ---------------------------- -------------------------------- --------
  Option                       Description                      Default
  ---------------------------- -------------------------------- --------
  BUILD_TUTORIALS              Builds the C++ version of the    ON
                               Embree tutorials.

  BUILD_TUTORIALS_ISPC         Builds the ISPC version of the   ON
                               Embree tutorials.

  CMAKE_BUILD_TYPE             Can be used to switch between    Release
                               Debug mode (Debug) and Release
                               mode (Release).

  COMPILER                     Select either GCC, ICC, or       GCC
                               CLANG as compiler.

  RTCORE_INTERSECTION_FILTER   Enables the intersection filter  ON
                               feature.

  RTCORE_BUFFER_STRIDE         Enables buffer stride feature.   ON

  RTCORE_RAY_MASK              Enables the ray masking feature. OFF

  RTCORE_SPINLOCKS             Enables faster spinlocks for     OFF
                               some builders.

  XEON_ISA                     Select highest ISA on Xeon™ CPUs AVX2
                               (SSE2, SSE3, SSSE3, SSE4.1,
                               SSE4.2, AVX, AVX-I, AVX2).

  XEON_PHI_ISA                 Enables generation of Xeon Phi™  OFF
                               version of kernels and
                               tutorials.
  ---------------------------- -------------------------------- --------
  : CMake build options for Embree.

You need at least Intel Compiler 11.1 or GCC 4.4 to enable AVX and Intel
Compiler 12.1 or GCC 4.7 to enable AVX2.

Xeon Phi™
---------

Embree supports the Xeon Phi™ coprocessor under Linux. To compile Embree
for Xeon Phi™ you need to enable the XEON_PHI_ISA option in CMake and
have the Intel Compiler and the Intel™ Manycore Platform Software Stack
(MPSS) installed.

Enabling the buffer stride feature reduces performance for building
spatial hierarchies on Xeon Phi™.

Windows
-------

Embree requires the Intel SPMD Compiler (ISPC) to compile. We have
tested ISPC version 1.6.0 and 1.7.0, but more recent versions of ISPC
should also work. You can download and install the ISPC binaries from
[ispc.github.com](http://ispc.github.com/downloads.html). After
installation, put the path to ispc.exe permanently into your `PATH`
environment variable. You have to restart Visual Studio for this change
to take effect.

For compilation of Embree under Windows use the Visual Studio solution
file embree.sln. The project compiles in 32 bit and 64 bit mode. The
solution is by default setup to use the Microsoft Compiler. You can
switch to the Intel Compiler by right clicking onto the solution in the
Solution Explorer and then selecting the Intel Compiler. We recommend
using 64 bit mode and the Intel Compiler for best performance.

In Visual Studio, you will find 4 build configurations, `Debug` (for
SSE2 debug mode), `Release` (for SSE2 release mode), `ReleaseAVX` (for
AVX release mode), and `ReleaseAVX2` (for AVX2 release mode). When using
the Microsoft Compiler you can only use the Debug and Release
configuration.  For enabling the `ReleaseAVX` configuration you need at
least Intel Compiler 11.1 and for the `ReleaseAVX2` configuration you
need at least Intel Compiler 12.1.

Most configuration parameters described for the Linux build can be set
under Windows by commenting out the appropiate lines in the
`common/sys/platform.h` file.

We recommend enabling syntax highlighting for the `.ispc` source and
`.isph` header files. To do so open Visual Studio 2008, go to Tools ⇒
Options ⇒ Text Editor ⇒ File Extension and add the isph and ispc
extension for the "Microsoft Visual C++" editor.

