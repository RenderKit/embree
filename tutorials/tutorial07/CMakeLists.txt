## ======================================================================== ##
## Copyright 2009-2014 Intel Corporation                                    ##
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

IF (__XEON__ AND BUILD_TUTORIALS)
  ADD_EXECUTABLE(tutorial07 tutorial07.cpp tutorial07_device.cpp)
  TARGET_LINK_LIBRARIES(tutorial07 embree tutorial image transport tutorial_device image)
ENDIF()

IF (__XEON__ AND BUILD_TUTORIALS_ISPC)
  ADD_ISPC_EXECUTABLE(tutorial07_ispc tutorial07.cpp tutorial07_device.ispc)
  TARGET_LINK_LIBRARIES(tutorial07_ispc embree tutorial image transport tutorial_device_ispc)
ENDIF()

IF (NOT __XEON__ AND __HOST__)
  ADD_EXECUTABLE(tutorial07_xeonphi tutorial07.cpp)
  TARGET_LINK_LIBRARIES(tutorial07_xeonphi tutorial image transport_host)
ENDIF()

IF (NOT __XEON__ AND NOT __HOST__)
  INCLUDE (icc_xeonphi)
  ADD_ISPC_EXECUTABLE(tutorial07_xeonphi_device tutorial07_device.ispc)
  TARGET_LINK_LIBRARIES(tutorial07_xeonphi_device embree_xeonphi transport_device tutorial_xeonphi_device_ispc)
ENDIF()
