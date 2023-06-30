REM Copyright 2019-2021 Intel Corporation
REM SPDX-License-Identifier: Apache-2.0

set EMBREE_VERSION=4.2.0

copy build\embree-%EMBREE_VERSION%.x64.windows-embree.zip tests\integration\test_embree_release\embree.zip /Y
cd tests/integration/test_embree_release
dir
mkdir embree_install
tar -xf embree.zip -C embree_install
cd embree_install
dir
cd ..

set EMBREE_CONFIG_DIR=.\embree_install\lib\cmake\embree-%EMBREE_VERSION%
set PATH=.\embree_install\bin;%PATH%

echo "%EMBREE_CONFIG_DIR%"

