## ======================================================================== ##
## Copyright 2009-2017 Intel Corporation                                    ##
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

SET(EMBREE_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../../@CMAKE_INSTALL_INCLUDEDIR@)
SET(EMBREE_LIBRARY ${CMAKE_CURRENT_LIST_DIR}/../../@CMAKE_SHARED_LIBRARY_PREFIX@embree@CMAKE_SHARED_LIBRARY_SUFFIX@.@EMBREE_CONFIG_VERSION@)

SET(EMBREE_TASKING_TBB      @TASKING_TBB@)
SET(EMBREE_USE_PACKAGED_TBB @EMBREE_ZIP_MODE@)

IF (${EMBREE_TASKING_TBB} AND ${EMBREE_USE_PACKAGED_TBB})
  SET(EMBREE_TBB_LIBRARY ${CMAKE_CURRENT_LIST_DIR}/../../@CMAKE_SHARED_LIBRARY_PREFIX@tbb@CMAKE_SHARED_LIBRARY_SUFFIX@.2)
  SET(EMBREE_TBB_LIBRARY_MALLOC ${CMAKE_CURRENT_LIST_DIR}/../../@CMAKE_SHARED_LIBRARY_PREFIX@tbbmalloc@CMAKE_SHARED_LIBRARY_SUFFIX@.2)
ELSE()
  SET(EMBREE_USE_PACKAGED_TBB OFF)
  UNSET(EMBREE_TBB_LIBRARY)
  UNSET(EMBREE_TBB_LIBRARY_MALLOC)
ENDIF()

SET(EMBREE_LIBRARIES
  ${EMBREE_LIBRARY}
  ${EMBREE_TBB_LIBRARY}
  ${EMBREE_TBB_LIBRARY_MALLOC}
)

MARK_AS_ADVANCED(embree_DIR)

INCLUDE(${CMAKE_CURRENT_LIST_DIR}/embree-config-default.cmake)
