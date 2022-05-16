## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PROJECT(minimal)
CMAKE_MINIMUM_REQUIRED(VERSION 3.2.0)

IF (NOT EMBREE_PROJECT_COMPILATION)
  FIND_PACKAGE(embree REQUIRED)
ENDIF()

IF (EMBREE_DPCPP_SUPPORT)
  SET(CMAKE_CXX_STANDARD 17)
ENDIF()

ADD_EXECUTABLE(minimal minimal.cpp)
TARGET_LINK_LIBRARIES(minimal embree)
SET_PROPERTY(TARGET minimal PROPERTY FOLDER tutorials/single)
INSTALL(TARGETS minimal DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
IF (COMMAND SIGN_TARGET)
  SIGN_TARGET(minimal)
ENDIF()

IF (EMBREE_DPCPP_SUPPORT)
  
  # with current public drivers, JIT works under Windows and AOT under Linux
  IF (NOT DEFINED EMBREE_DPCPP_AOT_DEVICES)
    IF (WIN32)
      SET(EMBREE_DPCPP_AOT_DEVICES "none")
    ELSE()
      SET(EMBREE_DPCPP_AOT_DEVICES "dg2")
    ENDIF()
  ENDIF()

  # JIT mode compile flags
  IF(EMBREE_DPCPP_AOT_DEVICES STREQUAL "none")
    SET(SYCL_COMPILE_FLAGS "-fsycl -Xclang -fsycl-allow-func-ptr -fsycl-targets=spir64")
    SET(SYCL_LINK_FLAGS "-lsycl -fsycl -fsycl-targets=spir64 -Xsycl-target-backend=spir64 \" -options \\\"-cl-intel-force-global-mem-allocation -cl-intel-no-local-to-generic\\\" \"")

  # AOT mode compile flags
  ELSE()
    SET(SYCL_COMPILE_FLAGS "-fsycl -Xclang -fsycl-allow-func-ptr -fsycl-targets=spir64_gen")
    SET(SYCL_LINK_FLAGS "-lsycl -fsycl -fsycl-targets=spir64_gen -Xsycl-target-backend=spir64_gen \"-device ${EMBREE_DPCPP_AOT_DEVICES} -revision_id 4 -options \\\"-cl-intel-force-global-mem-allocation -cl-intel-no-local-to-generic\\\" \"")
  ENDIF()

  GET_FILENAME_COMPONENT(SYCL_COMPILER_DIR ${CMAKE_CXX_COMPILER} PATH)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${SYCL_COMPILER_DIR}/../include/sycl -isystem ${SYCL_COMPILER_DIR}/../include/")
      
  ADD_EXECUTABLE(minimal_sycl minimal_sycl.cpp)
  TARGET_LINK_LIBRARIES(minimal_sycl embree)
  SET_PROPERTY(TARGET minimal_sycl APPEND PROPERTY COMPILE_FLAGS ${SYCL_COMPILE_FLAGS})
  SET_PROPERTY(TARGET minimal_sycl APPEND PROPERTY LINK_FLAGS    ${SYCL_LINK_FLAGS})
  TARGET_COMPILE_DEFINITIONS(minimal_sycl PUBLIC EMBREE_SYCL_TUTORIAL)
  SET_PROPERTY(TARGET minimal_sycl PROPERTY FOLDER tutorials/sycl)
  INSTALL(TARGETS minimal_sycl DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
  INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DESTINATION "src" COMPONENT examples)
  IF (COMMAND SIGN_TARGET)
    SIGN_TARGET(minimal_sycl)
  ENDIF()
ENDIF()

