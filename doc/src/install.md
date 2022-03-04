Installation of Embree
======================

Windows ZIP File
-----------------

Embree linked against Visual Studio 2015 are provided as a ZIP file
[embree-<EMBREE_VERSION>.x64.vc14.windows.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.vc14.windows.zip). After
unpacking this ZIP file, you should set the path to the `lib` folder
manually to your `PATH` environment variable for applications to find
Embree.

Linux tar.gz Files
------------------

The Linux version of Embree is also delivered as a `tar.gz` file:
[embree-<EMBREE_VERSION>.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.linux.tar.gz). Unpack
this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the C
shell) to set up the environment properly:

    tar xzf embree-<EMBREE_VERSION>.x86_64.linux.tar.gz
    source embree-<EMBREE_VERSION>.x86_64.linux/embree-vars.sh

We recommend adding a relative `RPATH` to your application that points
to the location where Embree (and TBB) can be found, e.g. `$ORIGIN/../lib`.

macOS ZIP file
-----------------

The macOS version of Embree is also delivered as a ZIP file:
[embree-<EMBREE_VERSION>.x86_64.macosx.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.macosx.zip). Unpack
this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the C
shell) to set up the environment properly:

    unzip embree-<EMBREE_VERSION>.x64.macosx.zip
    source embree-<EMBREE_VERSION>.x64.macosx/embree-vars.sh

If you want to ship Embree with your application, please use the Embree
library of the provided ZIP file. The library name of that Embree
library is of the form `@rpath/libembree.<EMBREE_VERSION_MAJOR>.dylib`
(and similar also for the included TBB library). This ensures that you
can add a relative `RPATH` to your application that points to the location
where Embree (and TBB) can be found, e.g. `@loader_path/../lib`.

