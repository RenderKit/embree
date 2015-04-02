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

FIND_PATH(EMBREE_INCLUDE_DIR embree2/rtcore.h)
FIND_LIBRARY(EMBREE_LIBRARY NAMES embree)
FIND_LIBRARY(EMBREE_LIBRARY_XEONPHI NAMES embree_xeonphi)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(EMBREE DEFAULT_MSG EMBREE_INCLUDE_DIR EMBREE_LIBRARY)

MARK_AS_ADVANCED(EMBREE_INCLUDE_DIR)
MARK_AS_ADVANCED(EMBREE_LIBRARY)
MARK_AS_ADVANCED(EMBREE_LIBRARY_XEONPHI)
