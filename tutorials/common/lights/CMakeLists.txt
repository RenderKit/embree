## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

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

IF (EMBREE_SYCL_SUPPORT)
  ADD_LIBRARY(lights_sycl STATIC
    light.cpp
    ambient_light.cpp
    directional_light.cpp
    point_light.cpp
    quad_light.cpp
    spot_light.cpp
    )
  TARGET_LINK_LIBRARIES(lights_sycl sys math)
  SET_PROPERTY(TARGET lights_sycl PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET lights_sycl APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST} ${CMAKE_CXX_FLAGS_SYCL}")
  TARGET_COMPILE_DEFINITIONS(lights_sycl PUBLIC EMBREE_SYCL_TUTORIAL)
ENDIF()

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
