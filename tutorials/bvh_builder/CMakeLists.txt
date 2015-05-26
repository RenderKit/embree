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

SET(ENABLE_ISPC_SUPPORT OFF)

IF (TASKING_INTERNAL)
  ADD_DEFINITIONS(-DTASKING_TBB_INTERNAL)
ELSE()
  ADD_DEFINITIONS(-DTASKING_TBB) # FIXME: should not be necessary
  SET(ADDITIONAL_LIBRARIES ${TBB_LIBRARIES})
ENDIF()

INCLUDE(tutorial)
ADD_TUTORIAL(bvh_builder)
