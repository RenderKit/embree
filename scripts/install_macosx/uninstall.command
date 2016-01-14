#!/bin/bash
echo Uninstalling Embree ...
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
cd $DIR
cd ..
cd ..
pkgutil --only-files --files com.intel.embree-@EMBREE_VERSION@| tr '\n' '\0' | xargs -n 1 -0 sudo rm -v
pkgutil --only-dirs --files com.intel.embree-@EMBREE_VERSION@ | grep -e 'opt/local/include/\|opt/local/lib/\|Applications/' | tail -r | tr '\n' '\0' | xargs -n 1 -0 sudo rm -vd
sudo pkgutil --forget com.intel.embree-@EMBREE_VERSION@
