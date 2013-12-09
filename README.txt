// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

Embree is a collection of high-performance ray tracing kernels,
developed at Intel. The target user of Embree are graphics application
engineers that want to improve the performance of their application by
leveraging the optimized ray tracing kernels of Embree. The kernels
are optimized for photo-realistic rendering on the latest Intel®
processors with support for SSE, AVX, AVX2, and the 16-wide Xeon Phi
vector instructions. Embree supports runtime code selection to choose
the traversal and build algorithms that best matches the instruction
set of your CPU. Embree is released as Open Source under the Apache
2.0 license.

Embree supports applications written with the Intel SPMD Programm
Compiler (ISPC, http://ispc.github.com) by providing an ISPC interface
to the core ray tracing algorithms. This makes it possible to write a
renderer in ISPC that leverages SSE, AVX, AVX2, and Xeon Phi
instructions without any code change. ISPC also supports runtime code
selection using the multi-target feature. This way it is possible to
let ISPC select at runtime the best code path for your application,
while Embree selects the optimal code path for the ray tracing
algorithms.

Embree contains algorithms optimized for incoherent workloads (e.g.
Monte Carlo ray tracing algorithms) and coherent workloads
(e.g. primary visibility and hard shadow rays). For standard CPUs, the
single-ray traversal kernels in Embree provide the best performance
for incoherent workloads and are very easy to integrate into existing
rendering applications. For Xeon Phi, a renderer written in ISPC using
the optimized hybrid ray/packet traversal algorithms have shown to
perform best and requires writing the renderer in ISPC. In general for
coherent workloads, ISPC outperforms the single ray mode on each
platform. Embree also supports dynamic scenes by implementing high
performance spatial index structure construction algorithms.

In addition to the ray tracing kernels, Embree provides some tutorials
to demonstrate how to use the Embree API. More documentation about the
API is found in embree/include/embree2/rtcore.h

--- Supported Platforms ---

Embree supports Windows, Linux and MacOSX, each in 32bit and 64bit
modes. The code compiles with the Intel Compiler, the Microsoft
Compiler, GCC and CLANG. Using the Intel Compiler improves performance
by approximately 10%. Performance also varies across different
operating systems. Embree requires a CPU with at least support for
SSE2. Embree is optimized for Intel CPUs supporting SSE, AVX, and AVX2
instructions.

The Xeon Phi version of Embree only works under Linux in 64bit
mode. For compilation of the the Xeon Phi code the Intel Compiler is
required. The host side code compiles with GCC and the Intel Compiler.

--- Compiling Embree on Linux and MacOSX ---

Embree requires ISPC to compile. We have tested ISPC version 1.5.0,
but more recent versions of ISPC should also work. You can download
and install ISPC binaries from ispc.github.com. After installation,
put ISPC permanently into your PATH:

  export PATH=path-to-ispc:$PATH

You additionally have to install CMake and the developer version of
GLUT. Under MaxOSX, these dependencies can be installed using
MacPorts:

   sudo port install cmake freeglut

Under Linux you can install the dependencies using yum:

   sudo yum install cmake.x86_64
   sudo yum install freeglut.x86_64 freeglut-devel.x86_64
   sudo yum install libXmu.x86_64 libXi.x86_64 libXmu-devel.x86_64 libXi-devel.x86_64

Finally you can compile Embree using CMake. Create a build directory
and execute "ccmake .." inside this directory.

   mkdir build
   cd build
   ccmake ..

This will open a configuration dialog where you should set the
CMAKE_BUILD_TYPE to “Release” and the compiler to "GCC", "CLANG" or
"ICC". You should also select all targets that you want Embree to
generate optimized code for. We recommend to enable TARGET_SSE41,
TARGET_AVX, and TARGET_AVX2 if you want to use Embree on standard
CPUs, and you have to enable TARGET_XEON_PHI if you want to use Embree
on Xeon Phi. You need at least ICC 11.1 or GCC 4.4 to enable AVX and
ICC 12.1 or GCC 4.7 to enable AVX2. Now press c (for configure) and g
(for generate) to generate a Makefile and leave the configuration. The
code can be compiled by executing make.

      make

The executables will be generated inside the build folder. We
recommend to finally install the Embree library and header files on
your system:

  sudo make install

--- Compiling Embree on Windows ---

Embree requires ISPC to compile. We have tested ISPC version 1.5.0,
but more recent versions of ISPC should also work. You can download
and install ISPC binaries from ispc.github.com. After installation,
put the path to ispc.exe permanently into your PATH environment
variable. You have to restart Visual Studio for this change to take
effect.

For compilation under Windows use the Visual Studio 2008 solution file
embree.sln. The project compiles in 32 bit and 64 bit mode. The
solution is by default setup to use the Microsoft Compiler. You can
switch to the Intel Compiler by right clicking onto the solution in the
Solution Explorer and then selecting the ICC compiler. When switching
to ICC you have to enable the embree_avx and embree_avx2 projects, by
right clicking onto them selecting "Reload Project". We recommend
using 64 bit mode and the Intel Compiler for best performance.

We recommend enabling syntax highlighting for the .ispc source 
and .isph header files. To do so open Visual Studio 2008, go to 
Tools -> Options -> Text Editor -> File Extension and add the isph
and ispc extension for the "Microsoft Visual C++" editor.

--- Folder structure ---

    embree                 Embree ray tracing kernels
    embree/include/        User API to the ray tracing kernels
    tutorials              Embree tutorials for
    tutorials/tutorial00     creating a simple static scene
    tutorials/tutorial01     creating a dynamic scene
    tutorials/tutorial02     user defined geometry
    tutorials/tutorial03     simple OBJ loader
    tutorials/tutorial04     instancing of geometry

--- Running the Tutorials ---

Some tutorials come as C++ and ISPC version, e.g.:

  ./tutorial00
  ./tutorial00_ispc

You can select an initial camera using the -vp (camera position), -vi
(camera lookat point), -vu (camera up vector), and -fov (field of
view) command line parameters:

  ./tutorial00 -vp 10 10 10 -vi 0 0 0

You can select the initial windows size using the -size command line
parameter, or start the tutorials in fullscreen using the -fullscreen
parameter:

  ./tutorial00 -size 1024 1024
  ./tutorial00 -fullscreen

Implementation specific parameters can be passed to the ray tracing
core through the -rtcore command line parameter, e.g.:

  ./tutorial00 -rtcore verbose=2,threads=1,accel=bvh4.triangle1

The navigation in the interactive display mode follows the camera
orbit model, where the camera revolves around the current center of
interest. With the left mouse button you can rotate around the center
of interest (the point initially set with -vi). Holding Control
pressed while klicking the left mouse button rotates the camera around
its location. You can also use the arrow keys for navigation.

--- Contact ---

Please contact embree_support@intel.com if you have questions related to
Embree or if you want to report a bug.
