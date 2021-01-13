## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

include(CMakeDependentOption)

ADD_SUBDIRECTORY(common/image)
CMAKE_DEPENDENT_OPTION(EMBREE_TUTORIALS_GLFW  "Enables GLFW usage in tutorials. When disabled tutorial can only render to disk." ON "EMBREE_TUTORIALS" OFF)

IF (EMBREE_TUTORIALS)

IF (EMBREE_TUTORIALS_GLFW)

  IF (NOT WIN32 OR EMBREE_EXTERNAL_GLFW)
  
    FIND_PACKAGE(glfw3 REQUIRED)
    SET(GLFW_LIBRARY glfw)
    SET(GLFW_INCLUDE_DIRS)
    
  ELSE()
  
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
    MARK_AS_ADVANCED(
      GLFW_INCLUDE_DIR
      GLFW_LIBRARY
    )
  ENDIF()

  FIND_PACKAGE(OpenGL)
  IF (NOT OPENGL_FOUND)
    MESSAGE(FATAL_ERROR "OpenGL not found! Please either install OpenGL or disable EMBREE_TUTORIALS.")
  ENDIF()

  ADD_DEFINITIONS(-DUSE_GLFW)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLFW_INCLUDE_DIRS})

ENDIF()

ADD_SUBDIRECTORY(common)

ADD_SUBDIRECTORY(verify)
ADD_SUBDIRECTORY(triangle_geometry)
ADD_SUBDIRECTORY(dynamic_scene)
ADD_SUBDIRECTORY(user_geometry)
ADD_SUBDIRECTORY(viewer)
ADD_SUBDIRECTORY(instanced_geometry)

if ("${EMBREE_MAX_INSTANCE_LEVEL_COUNT}" GREATER "1")
  ADD_SUBDIRECTORY(multi_instanced_geometry)
endif()

ADD_SUBDIRECTORY(intersection_filter)
ADD_SUBDIRECTORY(pathtracer)
ADD_SUBDIRECTORY(hair_geometry)
ADD_SUBDIRECTORY(subdivision_geometry)
ADD_SUBDIRECTORY(displacement_geometry)
ADD_SUBDIRECTORY(grid_geometry)
ADD_SUBDIRECTORY(bvh_builder)
ADD_SUBDIRECTORY(lazy_geometry)
ADD_SUBDIRECTORY(bvh_access)
ADD_SUBDIRECTORY(minimal)
ADD_SUBDIRECTORY(motion_blur_geometry)
ADD_SUBDIRECTORY(interpolation)
ADD_SUBDIRECTORY(convert)
ADD_SUBDIRECTORY(curve_geometry)
ADD_SUBDIRECTORY(point_geometry)
ADD_SUBDIRECTORY(buildbench)
ADD_SUBDIRECTORY(collide)
ADD_SUBDIRECTORY(closest_point)
ADD_SUBDIRECTORY(voronoi)
ADD_SUBDIRECTORY(next_hit)
ADD_SUBDIRECTORY(multiscene_geometry)
ADD_SUBDIRECTORY(quaternion_motion_blur)

IF (EMBREE_RAY_PACKETS)
  ADD_SUBDIRECTORY(viewer_stream)
  ADD_SUBDIRECTORY(viewer_anim)
ENDIF()

ADD_SUBDIRECTORY(embree_tests)

ENDIF()

