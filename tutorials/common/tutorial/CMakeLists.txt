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

IF (__XEON__)

IF (TASKING_INTERNAL)
  ADD_DEFINITIONS(-DTASKING_TBB_INTERNAL)
ELSE()
  ADD_DEFINITIONS(-DTASKING_TBB)
ENDIF()

IF (WIN32)
# FindGLUT.cmake is broken for Windows in CMake until 3.0: does not support PATH
  FIND_PATH(GLUT_INCLUDE_DIR NAMES GL/glut.h PATHS ../freeglut/include)
  # detect and select Win32 or x64
  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ARCH x64)
  ELSE()
    SET(ARCH Win32)
  ENDIF()
  FIND_LIBRARY(GLUT_glut_LIBRARY freeglut ../freeglut/${ARCH})
  SET(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
  MARK_AS_ADVANCED(
    GLUT_INCLUDE_DIR
    GLUT_glut_LIBRARY
  )
ELSE (WIN32)
  FIND_PACKAGE(GLUT REQUIRED)
ENDIF (WIN32)

FIND_PACKAGE(OpenGL REQUIRED)

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
ADD_LIBRARY(tutorial STATIC glutdisplay.cpp scene.cpp)
TARGET_LINK_LIBRARIES(tutorial sys lexers scenegraph ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
SET_PROPERTY(TARGET tutorial PROPERTY FOLDER tutorials/common)

IF (WIN32)
  GET_FILENAME_COMPONENT(GLUT_DIR ${GLUT_glut_LIBRARY} PATH)
  ADD_CUSTOM_COMMAND(TARGET tutorial POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GLUT_DIR}/freeglut.dll $<TARGET_FILE_DIR:tutorial>
    COMMENT "Copy FreeGlut DLL" VERBATIM
  )
  INSTALL(PROGRAMS ${GLUT_DIR}/freeglut.dll DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
ENDIF (WIN32)

ADD_LIBRARY(tutorial_device STATIC tutorial_device.cpp)
TARGET_LINK_LIBRARIES(tutorial_device tutorial)
SET_PROPERTY(TARGET tutorial_device PROPERTY FOLDER tutorials/common)

IF (ENABLE_ISPC_SUPPORT)
  ADD_ISPC_LIBRARY(tutorial_device_ispc STATIC tutorial_device.ispc)
  TARGET_LINK_LIBRARIES(tutorial_device_ispc tutorial)
  SET_TARGET_PROPERTIES(tutorial_device_ispc PROPERTIES LINKER_LANGUAGE C)
  SET_PROPERTY(TARGET tutorial_device_ispc PROPERTY FOLDER tutorials/common)
ENDIF()

ELSE (__XEON__)

  INCLUDE (icc_xeonphi)
  ADD_ISPC_LIBRARY(tutorial_xeonphi_device_ispc STATIC tutorial_device.ispc)
  SET_TARGET_PROPERTIES(tutorial_xeonphi_device_ispc PROPERTIES LINKER_LANGUAGE C)
  SET_PROPERTY(TARGET tutorial_xeonphi_device_ispc PROPERTY FOLDER tutorials/common)

ENDIF (__XEON__)
