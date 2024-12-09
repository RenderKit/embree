## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(alloc_tutorial STATIC alloc.cpp)
TARGET_LINK_LIBRARIES(alloc_tutorial sys)
SET_PROPERTY(TARGET alloc_tutorial PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET alloc_tutorial APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_SYCL_SUPPORT)
  ADD_LIBRARY(alloc_tutorial_sycl STATIC alloc.cpp)
  TARGET_LINK_LIBRARIES(alloc_tutorial_sycl sys)
  SET_PROPERTY(TARGET alloc_tutorial_sycl PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET alloc_tutorial_sycl APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST} ${CMAKE_CXX_FLAGS_SYCL}")
  TARGET_COMPILE_DEFINITIONS(alloc_tutorial_sycl PUBLIC EMBREE_SYCL_TUTORIAL)
ENDIF()