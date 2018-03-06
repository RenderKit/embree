Embree Example Renderer
=======================

The Embree Example Renderer is a photo-realistic path tracer that builds
on the Embree high performance ray tracing kernels. The renderer is used
to demonstrate how Embree is used in practice and to measure Embree's
performance in a realistic application scenario. The Embree Example
Renderer is not a full featured renderer and not designed to be used for
production rendering. The Embree Example Renderer is released as Open
Source under the [Apache 2.0
license](http://www.apache.org/licenses/LICENSE-2.0).

We provide binaries for the Embree Example Renderer for Linux (64-bit),
macOS (64-bit), and Windows (64-bit):

[embree-renderer-2.3.2-linux.zip](http://github.com/embree/embree-renderer-bin/archive/v2.3.2_linux.zip)  
[embree-renderer-2.3.2-macos.zip](http://github.com/embree/embree-renderer-bin/archive/v2.3.2_macos.zip)  
[embree-renderer-2.3.2-win.zip](http://github.com/embree/embree-renderer-bin/archive/v2.3.2_win.zip)  

In case the Windows reports a missing `MSVCP120.DLL` please install the
[Visual C++ Redistributable Packages for Visual Studio
2013.](http://www.microsoft.com/en-us/download/details.aspx?id=40784)

If you need to recompile the Embree Example Renderer for your platform,
please download the sources and follow the compilation instructions
below:

[embree-renderer-2.3.2.zip](http://github.com/embree/embree-renderer/archive/v2.3.2.zip)

Alternatively you can also use `git` to get the latest Embree Example
Renderer 2.3.2:

    git clone https://github.com/embree/embree-renderer.git embree-renderer
    cd embree-renderer
    git checkout v2.3.2

If you encounter bugs please report them to the [GitHub Issue
Tracker](https://github.com/embree/embree-renderer/issues) for the
Embree Renderer.

Compiling under Windows
-----------------------

For compilation under Windows you first have to install the Embree ray
tracing kernels including the Intel® SPMD Compiler (ISPC). After
installation you have to set the `EMBREE_INSTALL_DIR` environment
variable to the root folder of Embree.

Use the Visual Studio 2008 or Visual Studio 2010 solution file to
compile the Embree Example Renderer. Inside Visual Studio you can switch
between the Microsoft Compiler and the Intel Compiler by right clicking
on the solution and then selecting the compiler. The project compiles
with both compilers in 32-bit and 64-bit mode. We recommend using
64-bit mode and the Intel Compiler for best performance.

To enable AVX and AVX2 for the ISPC code select the build configurations
`ReleaseAVX` and `ReleaseAVX2`. You have to compile the Embree kernels
with the same or higher instruction set than the Embree example
renderer.

By default, the solution file requires ISPC to be installed properly.
For compiling the solution without ISPC, simply delete the device_ispc
project from the solution file.

Compiling under Linux and macOS
-------------------------------

To compile the Embree Example Renderer using CMake create a `build`
directory and execute `ccmake ..` inside this directory.

    mkdir build
    cd build
    ccmake ..

This will open a configuration dialog where you should set the build
mode to `Release` and the compiler to either GCC, CLANG, or ICC. You can
configure which parts of the Embree Example Renderer to build:

  ---------------------------------- -----------------------------------
  Option                             Description
  ---------------------------------- -----------------------------------
  BUILD_SINGLE_RAY_DEVICE            Single ray rendering device for
                                     CPUs.

  BUILD_SINGLE_RAY_DEVICE_XEON_PHI   Single ray rendering device for the
                                     Intel® Xeon Phi™ coprocessor.

  BUILD_ISPC_DEVICE                  ISPC CPU rendering device operating
                                     on ray packets of size 4 (SSE) or
                                     8 (AVX).

  BUILD_ISPC_DEVICE_XEON_PHI         ISPC Xeon Phi rendering device
                                     operating on ray packets of size 16.

  BUILD_NETWORK_DEVICE               Network device to render on render
                                     server.
  ---------------------------------- -----------------------------------
  : CMake build options for Embree Example Renderer.

When enabling any ISPC renderer, you also have to install ISPC. If you
select `BUILD_ISPC_DEVICE`, you should select which instructions sets to
enable for ISPC (`TARGET_SSE2`, `TARGET_SSE41`, `TARGET_AVX`, and
`TARGET_AVX2`).

All target ISAs you select when compiling the Embree Example Render,
have also to be enabled when compiling Embree. Due to some limitation of
ISPC you have to enable more than one target ISA if you also enabled
more than one target ISA when compiling Embree, otherwise you will get
link errors.

If you installed Embree, CMake will find it automatically and set the
`EMBREE_INCLUDE_PATH` and `EMBREE_LIBRARY` variables.

If you cannot install Embree on your system (e.g. when you don't have
administrator rights) some additional configurations are required to use
Embree from its build folder. Set the `EMBREE_INCLUDE_PATH` to the
embree_root_directory/include folder and the `EMBREE_LIBRARY` to
embree_root_directory/build/libembree.2.3.2.dylib for macOS or
embree_root_directory/build/libembree.so.2.3.2 for Linux. Under Linux
you have to additionally add embree_root_directory/build to your
`LD_LIBRARY_PATH` (and `SINK_LD_LIBRARY_PATH` in case you want to use
Embree on the Intel® Xeon Phi™ coprocessor).

Press c (for configure) and g (for generate) to generate a Makefile and
leave the configuration. The code can now be compiled by executing make.
The executables will be generated in the build folder.

    make

Using the Embree Example Renderer
---------------------------------

The example renderer also ships with a few simple test scenes, each
consisting of a scene file (`.xml` or `.obj`) and a command script file
(`.ecs`). The command script file contains command line parameters that
set the camera parameters, lights and render settings. The following
command line will render the Cornell Box scene with 16 samples per pixel
and write the resulting image to the file `cornell_box.tga` in the
current directory:

    ./renderer -c ../models/cornell_box.ecs -spp 16 -o cornell_box.tga

To interactively display the same scene, enter the following command:

    ./renderer -c ../models/cornell_box.ecs

A window will open and you can control the camera using the mouse and
keyboard. Pressing c in interactive mode outputs the current camera
parameters, pressing r enables or disables the progressive refinement
mode.

By default the renderer uses the single ray device. For selecting a
different device use the `-device` command line parameter as first
argument:

    ./renderer -device singleray -c ../models/cornell_box.ecs
    ./renderer -device ispc -c ../models/cornell_box.ecs

    ./renderer -device singleray_xeonphi -c ../../models/cornell_box.ecs
    ./renderer -device ispc_xeonphi -c ../../models/cornell_box.ecs

### Network Mode

For using the network device start the render server on some machine:

    renderer_server

Make sure that port 8484 is not blocked by the firewall. Now you can
connect from a second machine to the render server:

    renderer -connect ip_of_render_server -c ../../models/cornell_box.ecs

### Navigation

The navigation in the interactive display mode follows the camera orbit
model, where the camera revolves around the current center of interest.
The camera navigation assumes the $y$-axis to point upwards. If your
scene is modeled using the $z$-axis as up axis we recommend rotating the
scene.

Left Mouse Button
:   Rotate around center of interest

Middle Mouse Button
:   Pan

Right Mouse Button
:   Dolly (move camera closer or away from center of interest)

Ctrl+Left Mouse Button
:   Pick center of interest

Ctrl+Shift+Left Mouse Button
:   Pick focal distances

Alt+Left Mouse Button
:   Roll camera around view direction

L Key
:   Decrease lens radius by one world space unit

Shift+L Key
:   Increase lens radius by one world space unit

Example Models
--------------

We provide also a more advanced model of the Imperial Crown of Austria
to test the renderer with: [Crown0413.zip
(74MB)](https://software.intel.com/sites/default/files/managed/6b/ad/crown0413.zip).

<img src="images/crown.jpg" alt="Imperial Crown of Austria" width="400" height="480">

Model courtesy [Martin Lubich](http://www.loramel.net).

