#!/bin/tcsh

## ======================================================================== ##
## Copyright 2009-2020 Intel Corporation                                    ##
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

pushd . > /dev/null
set SCRIPT_PATH=($_)
set SCRIPT_PATH="$SCRIPT_PATH[2]"
if ( -l "${SCRIPT_PATH}" ) then
  while( -l "${SCRIPT_PATH}" ) do cd `dirname "$SCRIPT_PATH"`; SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
endif
cd "`dirname "$SCRIPT_PATH"`" > /dev/null
set SCRIPT_PATH=`pwd`
popd > /dev/null

if (!($?CPATH)) then
  setenv CPATH
endif

if (!($?LIBRARY_PATCH)) then
  setenv LIBRARY_PATH
endif

if (!($?DYLD_LIBRARY_PATH)) then
  setenv DYLD_LIBRARY_PATH
endif

setenv CPATH "$SCRIPT_PATH/@CMAKE_INSTALL_INCLUDEDIR@":${CPATH}

setenv LIBRARY_PATH "$SCRIPT_PATH/@CMAKE_INSTALL_LIBDIR@":${LIBRARY_PATH}

setenv DYLD_LIBRARY_PATH "$SCRIPT_PATH/@CMAKE_INSTALL_LIBDIR@":${DYLD_LIBRARY_PATH}

