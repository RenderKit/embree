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

IF (TASKING_INTERNAL)
  ADD_DEFINITIONS(-DTASKING_TBB_INTERNAL)
ELSE()
  ADD_DEFINITIONS(-DTASKING_TBB) # FIXME: should not be necessary
ENDIF()

ADD_EXECUTABLE(bvh_access bvh_access.cpp)
TARGET_LINK_LIBRARIES(bvh_access embree)
SET_PROPERTY(TARGET bvh_access PROPERTY FOLDER tutorials/single)
INSTALL(TARGETS bvh_access DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
