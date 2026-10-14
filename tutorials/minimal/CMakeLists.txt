## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PROJECT(minimal)
CMAKE_MINIMUM_REQUIRED(VERSION 3.13)

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

ADD_EMBREE_GENERIC_CPP_TEST(minimal embree_minimal)

IF (EMBREE_SYCL_SUPPORT)
  
  ADD_EXECUTABLE(embree_minimal_sycl minimal_sycl.cpp)
  TARGET_LINK_LIBRARIES(embree_minimal_sycl embree ze_wrapper ${CMAKE_DL_LIBS})
  TARGET_COMPILE_OPTIONS(embree_minimal_sycl PRIVATE -fsycl -Xclang -fsycl-allow-func-ptr -fsycl-targets=spir64)
  TARGET_LINK_OPTIONS(embree_minimal_sycl PRIVATE -fsycl -fsycl-targets=spir64)
  
  CHECK_CXX_COMPILER_FLAG("-Wno-sycl-undefined-func-in-image" HAVE_WNO_SYCL_UNDEFINED_FUNC_IN_IMAGE)
  IF (HAVE_WNO_SYCL_UNDEFINED_FUNC_IN_IMAGE)
    TARGET_LINK_OPTIONS(embree_minimal_sycl PRIVATE -Wno-sycl-undefined-func-in-image)
  ENDIF()

  SET_PROPERTY(TARGET embree_minimal_sycl PROPERTY FOLDER tutorials/sycl)
  INSTALL(TARGETS embree_minimal_sycl DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
  IF (COMMAND SIGN_TARGET)
    SIGN_TARGET(embree_minimal_sycl)
  ENDIF()
  
  ADD_EMBREE_GENERIC_SYCL_TEST(minimal embree_minimal)
ENDIF()
