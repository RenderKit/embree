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

SET(PTEX_ROOT ${PROJECT_SOURCE_DIR}/ptex CACHE STRING "Root of Ptex installation")

FIND_PATH   (PTEX_INCLUDE_DIRS Ptexture.h PATHS ${PTEX_ROOT}/include ${PTEX_ROOT}/src/ptex )
FIND_LIBRARY(PTEX_LIBRARIES    Ptex       PATHS ${PTEX_ROOT}/lib     ${PTEX_ROOT}/build/ptex )
 
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PTEX DEFAULT_MSG PTEX_INCLUDE_DIRS PTEX_LIBRARIES)

MARK_AS_ADVANCED(PTEX_INCLUDE_DIRS)
MARK_AS_ADVANCED(PTEX_LIBRARIES)

