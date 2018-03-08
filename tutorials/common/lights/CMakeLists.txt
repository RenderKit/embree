## ======================================================================== ##
## Copyright 2009-2018 Intel Corporation                                    ##
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

ADD_LIBRARY(lights STATIC
  light.cpp
  ambient_light.cpp
  directional_light.cpp
  point_light.cpp
  quad_light.cpp
  spot_light.cpp
)
TARGET_LINK_LIBRARIES(lights sys math)
SET_PROPERTY(TARGET lights PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET lights APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_ISPC_SUPPORT)
  ADD_ISPC_LIBRARY(lights_ispc STATIC
    light.ispc
    ambient_light.ispc
    directional_light.ispc
    point_light.ispc
    quad_light.ispc
    spot_light.ispc
  )
  TARGET_LINK_LIBRARIES(lights_ispc sys math)
  SET_TARGET_PROPERTIES(lights_ispc PROPERTIES LINKER_LANGUAGE CXX)
  SET_PROPERTY(TARGET lights_ispc PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET lights_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
ENDIF()
