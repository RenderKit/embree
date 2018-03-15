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

# additional parameters (beyond the name) are treated as additional dependencies
# if ADDITIONAL_LIBRARIES is set these will be included during linking

MACRO (ADD_TUTORIAL TUTORIAL_NAME)
  ADD_EXECUTABLE(${TUTORIAL_NAME} ${TUTORIAL_NAME}.cpp ${TUTORIAL_NAME}_device.cpp ${ARGN})
  TARGET_LINK_LIBRARIES(${TUTORIAL_NAME} embree image tutorial noise ${ADDITIONAL_LIBRARIES})
  SET_PROPERTY(TARGET ${TUTORIAL_NAME} PROPERTY FOLDER tutorials/single)
  SET_PROPERTY(TARGET ${TUTORIAL_NAME} APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
  INSTALL(TARGETS ${TUTORIAL_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
  SIGN_TARGET(${TUTORIAL_NAME})
ENDMACRO ()

MACRO (ADD_TUTORIAL_ISPC TUTORIAL_NAME)
  IF (EMBREE_ISPC_SUPPORT)
    ADD_EMBREE_ISPC_EXECUTABLE(${TUTORIAL_NAME}_ispc ${TUTORIAL_NAME}.cpp ${TUTORIAL_NAME}_device.ispc)
    TARGET_LINK_LIBRARIES(${TUTORIAL_NAME}_ispc embree image tutorial_ispc noise noise_ispc)
    SET_PROPERTY(TARGET ${TUTORIAL_NAME}_ispc PROPERTY FOLDER tutorials/ispc)
    SET_PROPERTY(TARGET ${TUTORIAL_NAME}_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
    INSTALL(TARGETS ${TUTORIAL_NAME}_ispc DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
    SIGN_TARGET(${TUTORIAL_NAME}_ispc)
  ENDIF()
ENDMACRO ()
