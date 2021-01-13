## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

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
