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

IF (WIN32)

  # detect and select Win32 or x64
  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ARCH x64)
  ELSE()
    SET(ARCH Win32)
  ENDIF()

  # detect Visual Studio version
  IF (MSVC12)
    SET(VCVER vc12)
  ELSE()
    SET(VCVER vc14)
  ENDIF()

  FIND_PATH(GLFW_INCLUDE_DIRS NAMES GLFW/glfw3.h PATHS common/glfw/include)
  FIND_LIBRARY(GLFW_LIBRARY glfw3 common/glfw/${ARCH}/${VCVER})
  SET(GLFW_STATIC_LIBRARIES ${GLFW_LIBRARY})
  MARK_AS_ADVANCED(
    GLFW_INCLUDE_DIR
    GLFW_LIBRARY
  )

ELSE (WIN32)
  
  FIND_PACKAGE(PkgConfig REQUIRED)
  PKG_SEARCH_MODULE(GLFW QUIET glfw3)
  IF (NOT GLFW_FOUND)
    MESSAGE(FATAL_ERROR "GLFW not found! Please either install GLFW or disable EMBREE_TUTORIALS.")
  ENDIF()

  LINK_DIRECTORIES(${GLFW_LIBRARY_DIRS})
ENDIF (WIN32)

FIND_PACKAGE(OpenGL)
IF (NOT OPENGL_FOUND)
  MESSAGE(FATAL_ERROR "OpenGL not found! Please either install OpenGL or disable EMBREE_TUTORIALS.")
ENDIF()

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLFW_INCLUDE_DIRS})

ADD_SUBDIRECTORY(common)

ADD_SUBDIRECTORY(verify)
ADD_SUBDIRECTORY(triangle_geometry)
ADD_SUBDIRECTORY(dynamic_scene)
ADD_SUBDIRECTORY(user_geometry)
ADD_SUBDIRECTORY(viewer)
ADD_SUBDIRECTORY(instanced_geometry)
ADD_SUBDIRECTORY(intersection_filter)
ADD_SUBDIRECTORY(pathtracer)
ADD_SUBDIRECTORY(hair_geometry)
ADD_SUBDIRECTORY(subdivision_geometry)
ADD_SUBDIRECTORY(displacement_geometry)
ADD_SUBDIRECTORY(grid_geometry)
ADD_SUBDIRECTORY(bvh_builder)
ADD_SUBDIRECTORY(lazy_geometry)
ADD_SUBDIRECTORY(bvh_access)
ADD_SUBDIRECTORY(motion_blur_geometry)
ADD_SUBDIRECTORY(interpolation)
ADD_SUBDIRECTORY(convert)
ADD_SUBDIRECTORY(curve_geometry)
ADD_SUBDIRECTORY(point_geometry)
ADD_SUBDIRECTORY(buildbench)

IF (EMBREE_RAY_PACKETS)
  ADD_SUBDIRECTORY(viewer_stream)
  ADD_SUBDIRECTORY(viewer_anim)
ENDIF()


