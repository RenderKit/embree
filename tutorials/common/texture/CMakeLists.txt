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

ADD_LIBRARY(texture STATIC
  texture2d.cpp
)
TARGET_LINK_LIBRARIES(texture sys math)
SET_PROPERTY(TARGET texture PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET texture APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_ISPC_SUPPORT)
  ADD_ISPC_LIBRARY(texture_ispc STATIC
    texture2d.ispc
  )
  TARGET_LINK_LIBRARIES(texture_ispc sys math)
  SET_TARGET_PROPERTIES(texture_ispc PROPERTIES LINKER_LANGUAGE CXX)
  SET_PROPERTY(TARGET texture_ispc PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET texture_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
ENDIF()
