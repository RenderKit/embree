Installation of Embree
======================

Windows MSI Installer
---------------------

You can install the Embree library using the Windows MSI installer
[embree-<EMBREE_VERSION>-x64.vc12.msi](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.vc12.msi). This
will install the 64-bit Embree version by default in `Program
Files\Intel\Embree v<EMBREE_VERSION> x64`.

You have to set the path to the `bin` folders manually to your `PATH`
environment variable for applications to find Embree.

To compile applications with Embree using CMake, please have a look at
the `find_embree` tutorial. To compile this tutorial, you need to set
the `embree_DIR` CMake variable of this tutorial to `Program
Files\Intel\Embree v<EMBREE_VERSION> x64`.

To uninstall Embree, open `Programs and Features` by clicking the
`Start button`, clicking `Control Panel`, clicking `Programs`, and
then clicking `Programs and Features`. Select `Embree
<EMBREE_VERSION> x64` and uninstall it.

Windows ZIP File
-----------------

Embree linked against Visual Studio 2013
[embree-<EMBREE_VERSION>.x64.vc12.windows.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.vc12.windows.zip)
and Visual Studio 2015
[embree-<EMBREE_VERSION>.x64.vc14.windows.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.vc14.windows.zip)
are provided as a ZIP file. After unpacking this ZIP file, you should
set the path to the `lib` folder manually to your `PATH` environment
variable for applications to find Embree. To compile applications with
Embree, you also have to set the `Include Directories` path in Visual
Studio to the `include` folder of the Embree installation.

If you plan to ship Embree with your application, best use the Embree
version from this ZIP file.

Linux RPMs
----------

Uncompress the `tar.gz` file
[embree-<EMBREE_VERSION>.x86_64.rpm.tar.gz](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.rpm.tar.gz)
to obtain the individual RPM files:

    tar xzf embree-<EMBREE_VERSION>.x86_64.rpm.tar.gz

To install Embree using the RPM packages on your Linux system, type
the following:

    sudo rpm --install embree<EMBREE_VERSION_MAJOR>-lib-<EMBREE_VERSION>-1.x86_64.rpm
    sudo rpm --install embree<EMBREE_VERSION_MAJOR>-devel-<EMBREE_VERSION>-1.noarch.rpm
    sudo rpm --install embree<EMBREE_VERSION_MAJOR>-examples-<EMBREE_VERSION>-1.x86_64.rpm

You also have to install the Intel® Threading Building Blocks (TBB)
using `yum`:

    sudo yum install tbb.x86_64 tbb-devel.x86_64

On Debian-based Linux distributions you first need to convert the RPM
filed into DEB files using the `alien` tool:

    sudo apt-get install alien dpkg-dev debhelper build-essential

    sudo alien embree<EMBREE_VERSION_MAJOR>-lib-<EMBREE_VERSION>-1.x86_64.rpm
    sudo alien embree<EMBREE_VERSION_MAJOR>-devel-<EMBREE_VERSION>-1.noarch.rpm
    sudo alien embree<EMBREE_VERSION_MAJOR>-examples-<EMBREE_VERSION>-1.x86_64.rpm

    sudo dpkg -i embree<EMBREE_VERSION_MAJOR>-lib_<EMBREE_VERSION>-2_amd64.deb
    sudo dpkg -i embree<EMBREE_VERSION_MAJOR>-devel_<EMBREE_VERSION>-2_all.deb
    sudo dpkg -i embree<EMBREE_VERSION_MAJOR>-examples_<EMBREE_VERSION>-2_amd64.deb

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
`/usr/bin/embree<EMBREE_VERSION_MAJOR>` folder. Specify the full path to
the tutorials to start them.

To uninstall Embree, just execute the following:

    sudo rpm --erase embree<EMBREE_VERSION_MAJOR>-lib-<EMBREE_VERSION>-1.x86_64
    sudo rpm --erase embree<EMBREE_VERSION_MAJOR>-devel-<EMBREE_VERSION>-1.noarch
    sudo rpm --erase embree<EMBREE_VERSION_MAJOR>-examples-<EMBREE_VERSION>-1.x86_64

Linux tar.gz Files
------------------

The Linux version of Embree is also delivered as a `tar.gz` file:
[embree-<EMBREE_VERSION>.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.linux.tar.gz). Unpack this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the
C shell) to set up the environment properly:

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
[embree-<EMBREE_VERSION>.x86_64.dmg](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.dmg). This
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

macOS tar.gz file
-----------------

The macOS version of Embree is also delivered as a `tar.gz` file:
[embree-<EMBREE_VERSION>.x86_64.macosx.tar.gz](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.macosx.tar.gz). Unpack this file using `tar` and source the provided `embree-vars.sh` (if you
are using the bash shell) or `embree-vars.csh` (if you are using the
C shell) to set up the environment properly:

    tar xzf embree-<EMBREE_VERSION>.x64.macosx.tar.gz
    source embree-<EMBREE_VERSION>.x64.macosx/embree-vars.sh

If you want to ship Embree with your application, please use the Embree
library of the provided `tar.gz` file. The library name of that Embree
library is of the form `@rpath/libembree.<EMBREE_VERSION_MAJOR>.dylib`
(and similar also for the included TBB library). This ensures that you
can add a relative `RPATH` to your application that points to the location
where Embree (and TBB) can be found, e.g. `@loader_path/../lib`.

