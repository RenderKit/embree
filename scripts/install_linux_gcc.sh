## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

#!/bin/bash

dest=$HOME/devtools-2
arch=`arch`
major=`rpm -q --qf "%{VERSION}" $(rpm -q --whatprovides redhat-release)`
source_url=http://people.centos.org/tru/devtools-2/$major/$arch/RPMS
usr_path="$HOME/devtools-2/opt/rh/devtoolset-2/root/usr"
packages="\
devtoolset-2-binutils \
devtoolset-2-gcc \
devtoolset-2-libstdc++-devel \
devtoolset-2-gcc-c++ \
"

echo Installing gcc in $dest
pushd . > /dev/null
mkdir -p $dest
cd $dest

echo .fetching RPM list
wget -cq $source_url/repodata/primary.xml.gz
gunzip -c primary.xml.gz > primary.xml

for package in $packages ; do
	echo .looking for $package
	p=`grep -A 29 "^<package type=\"rpm\"><name>$package</name>" primary.xml | grep -B 29 '</package>' | sed -e "s@.*<location href=\"\($package.\{3,30\}\.rpm\)\"/>.*@\1@;ty;d;:y" | sort -g | tail -n 1`
	echo ...fetching $p
	wget -cq $source_url/$p
	echo ...installing $p
	rpm2cpio $p | cpio -idu --quiet
done

popd > /dev/null

export PATH="$usr_path/bin":$PATH
export LD_LIBRARY_PATH="$usr_path/lib64":$LD_LIBRARY_PATH

echo Installation finished. To use gcc of devtoolset-2 be sure to keep \""$usr_path/bin"\" in your \$PATH and \""$usr_path/lib64"\" in your \$LD_LIBRARY_PATH.
