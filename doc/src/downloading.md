Downloading Embree
------------------

For Windows we provide Embree as ZIP files linked against the Visual Studio 2015/2017 (VC14) runtime:

[embree-<EMBREE_VERSION>.x64.vc14.windows.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x64.vc14.windows.zip)  

For Linux we provide Embree as a `tar.gz` file:

[embree-<EMBREE_VERSION>.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.linux.tar.gz)  

For macOS we provide Embree a ZIP file:

[embree-<EMBREE_VERSION>.x86_64.macosx.zip](https://github.com/embree/embree/releases/download/v<EMBREE_VERSION>/embree-<EMBREE_VERSION>.x86_64.macosx.zip)

The source code of the latest Embree version can be downloaded here:

[embree-<EMBREE_VERSION>.zip](https://github.com/embree/embree/archive/v<EMBREE_VERSION>.zip)  
[embree-<EMBREE_VERSION>.tar.gz](https://github.com/embree/embree/archive/v<EMBREE_VERSION>.tar.gz)

You can access old Embree releases at [https://github.com/embree/embree/releases](https://github.com/embree/embree/releases).

Alternatively you can also use `git` to get the source code of Embree v<EMBREE_VERSION>

    $ git clone https://github.com/embree/embree.git embree
    $ cd embree
    $ git checkout v<EMBREE_VERSION>

You can also check out the source code of Embree with subversion:

    $ svn checkout https://github.com/embree/embree.git/tags/v<EMBREE_VERSION> embree
    $ cd embree

If you encounter bugs please report them to the [GitHub Issue
Tracker](https://github.com/embree/embree/issues) for Embree.
