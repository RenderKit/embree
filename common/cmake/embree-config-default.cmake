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

SET(EMBREE_VERSION @EMBREE_VERSION@)
SET(EMBREE_VERSION_MAJOR @EMBREE_VERSION_MAJOR@)
SET(EMBREE_VERSION_MINOR @EMBREE_VERSION_MINOR@)
SET(EMBREE_VERSION_PATCH @EMBREE_VERSION_PATCH@)
SET(EMBREE_VERSION_NOTE "@EMBREE_VERSION_NOTE@")

SET(EMBREE_ISA @XEON_ISA@)
SET(EMBREE_XEON_PHI_SUPPORT @ENABLE_XEON_PHI_SUPPORT@)
SET(EMBREE_BUILD_TYPE @CMAKE_BUILD_TYPE@)
SET(EMBREE_RAY_PACKETS @RTCORE_RAY_PACKETS@)
SET(EMBREE_ISPC_SUPPORT @ENABLE_ISPC_SUPPORT@)
SET(EMBREE_STATIC_LIB @ENABLE_STATIC_LIB@)
SET(EMBREE_TUTORIALS @ENABLE_TUTORIALS@)

SET(EMBREE_RAY_MASK @RTCORE_RAY_MASK@)
SET(EMBREE_STAT_COUNTERS @RTCORE_STAT_COUNTERS@)
SET(EMBREE_BACKFACE_CULLING @RTCORE_BACKFACE_CULLING@)
SET(EMBREE_INTERSECTION_FILTER @RTCORE_INTERSECTION_FILTER@)
SET(EMBREE_INTERSECTION_FILTER_RESTORE @RTCORE_INTERSECTION_FILTER_RESTORE@)
SET(EMBREE_BUFFER_STRIDE @RTCORE_BUFFER_STRIDE@)
SET(EMBREE_EXPORT_ALL_SYMBOLS @RTCORE_EXPORT_ALL_SYMBOLS@)
SET(EMBREE_ENABLE_RAYSTREAM_LOGGER @RTCORE_ENABLE_RAYSTREAM_LOGGER@) 
SET(EMBREE_IGNORE_INVALID_RAYS @RTCORE_IGNORE_INVALID_RAYS@)
SET(EMBREE_TASKING_SYSTEM @RTCORE_TASKING_SYSTEM@)
