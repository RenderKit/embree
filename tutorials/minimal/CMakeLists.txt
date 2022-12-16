## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PROJECT(minimal)
CMAKE_MINIMUM_REQUIRED(VERSION 3.2.0)

IF (NOT EMBREE_PROJECT_COMPILATION)
  FIND_PACKAGE(embree REQUIRED)
ENDIF()

IF (EMBREE_SYCL_SUPPORT)
  SET(CMAKE_CXX_STANDARD 17)
ENDIF()

ADD_EXECUTABLE(embree_minimal minimal.cpp)
TARGET_LINK_LIBRARIES(embree_minimal embree)

SET_PROPERTY(TARGET embree_minimal PROPERTY FOLDER tutorials/single)
INSTALL(TARGETS embree_minimal DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DESTINATION "src" COMPONENT examples)
IF (COMMAND SIGN_TARGET)
  SIGN_TARGET(embree_minimal)
ENDIF()

IF (EMBREE_SYCL_SUPPORT)
  
  ADD_EXECUTABLE(embree_minimal_sycl minimal_sycl.cpp)
  TARGET_LINK_LIBRARIES(embree_minimal_sycl embree)
  SET_PROPERTY(TARGET embree_minimal_sycl APPEND PROPERTY COMPILE_FLAGS "-fsycl -Xclang -fsycl-allow-func-ptr -fsycl-targets=spir64")
  SET_PROPERTY(TARGET embree_minimal_sycl APPEND PROPERTY LINK_FLAGS    "-fsycl -fsycl-targets=spir64")
  
  SET_PROPERTY(TARGET embree_minimal_sycl PROPERTY FOLDER tutorials/sycl)
  INSTALL(TARGETS embree_minimal_sycl DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
  IF (COMMAND SIGN_TARGET)
    SIGN_TARGET(embree_minimal_sycl)
  ENDIF()
  
ENDIF()
