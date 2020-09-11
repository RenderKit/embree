Installation of Embree
======================

Windows MSI Installer
---------------------

You can install the Embree library using the Windows MSI installer
[embree-<EMBREE_VERSION>-x64.vc14.msi](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.vc14.msi). This
will install the 64-bit Embree version by default in `Program
Files\Intel\Embree<EMBREE_VERSION_MAJOR>`.

You have to set the path to the `bin` folders manually to your `PATH`
environment variable for applications to find Embree.

To compile applications with Embree using CMake, please have a look at
the `find_embree` tutorial. To compile this tutorial, you need to set
the `embree_DIR` CMake variable of this tutorial to `Program
Files\Intel\Embree<EMBREE_VERSION_MAJOR>`.

To uninstall Embree, open `Programs and Features` by clicking the
`Start button`, clicking `Control Panel`, clicking `Programs`, and
then clicking `Programs and Features`. Select `Embree
<EMBREE_VERSION> x64` and uninstall it.

Windows ZIP File
-----------------

Embree linked against Visual Studio 2015 are provided as a ZIP file
[embree-<EMBREE_VERSION>.x64.vc14.windows.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.vc14.windows.zip). After
unpacking this ZIP file, you should set the path to the `lib` folder
manually to your `PATH` environment variable for applications to find
Embree. To compile applications with Embree, you also have to set the
`Include Directories` path in Visual Studio to the `include` folder of
the Embree installation.

If you plan to ship Embree with your application, best use the Embree
version from this ZIP file.

Linux tar.gz Files
------------------

The Linux version of Embree is also delivered as a `tar.gz` file:
[embree-<EMBREE_VERSION>.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.linux.tar.gz). Unpack
this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the C
shell) to set up the environment properly:

    tar xzf embree-<EMBREE_VERSION>.x86_64.linux.tar.gz
    source embree-<EMBREE_VERSION>.x86_64.linux/embree-vars.sh

If you want to ship Embree with your application, best use the Embree
version provided in the `tar.gz` file.

We recommend adding a relative `RPATH` to your application that points
to the location where Embree (and TBB) can be found, e.g. `$ORIGIN/../lib`.

macOS PKG Installer
-------------------

To install the Embree library on your macOS system use the
provided package installer inside
[embree-<EMBREE_VERSION>.x86_64.pkg](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.pkg). This
will install Embree by default into `/opt/local/lib` and
`/opt/local/include` directories. The Embree tutorials are installed
into the `/Applications/Embree<EMBREE_VERSION_MAJOR>` directory.

You also have to install the Intel® Threading Building Blocks (TBB)
using [MacPorts](http://www.macports.org/):

    sudo port install tbb

Alternatively you can download the latest TBB version from
[https://www.threadingbuildingblocks.org/download](https://www.threadingbuildingblocks.org/download)
and set the `DYLD_LIBRARY_PATH` environment variable to point
to the TBB library.

To uninstall Embree, execute the uninstaller script
`/Applications/Embree<EMBREE_VERSION_MAJOR>/uninstall.command`.

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

