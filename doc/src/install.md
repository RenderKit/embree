Installation of Embree
======================

Windows
-------

To install the Embree library on your system add the folder `lib/x64` to
your `PATH`. To compile applications with Embree you also have to set
the "Include Directories" path in Visual Studio to the `include/` folder
of Embree.

Before you can run the tutorials in the `bin/x64` folder you have to
install the Embree library.

Linux and Mac OS\ X
-------------------

To install the Embree library and Embree header files on your system
type the following:

    sudo ./install.sh

Instead of installing the Embree library, you can also source the file
`paths.sh`:

    source ./paths.sh

Linking ISPC applications with Embree
-------------------------------------

The precompiled Embree library uses the multi-target mode of ISPC. For
your ISPC application to properly link against Embree you also have to
enable this mode. You can do this by specifying multiple targets when
compiling your application with ISPC, e.g.:

    ispc --target sse2,sse4,avx,avx2 -o code.o code.ispc

