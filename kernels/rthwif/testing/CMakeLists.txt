## Copyright 2009-2022 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PROJECT(rthwif_testing)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

SET(CMAKE_CXX_STANDARD 17)

ADD_EXECUTABLE(rthwif_cornell_box rthwif_cornell_box.cpp ../rthwif_production.cpp)
SET_PROPERTY(TARGET rthwif_cornell_box APPEND PROPERTY COMPILE_FLAGS "-fsycl -fsycl-targets=spir64 -DEMBREE_DPCPP_SUPPORT -DEMBREE_DPCPP_IMPLICIT_DISPATCH_GLOBALS")
SET_PROPERTY(TARGET rthwif_cornell_box APPEND PROPERTY LINK_FLAGS "-fsycl")

ADD_EXECUTABLE(rthwif_test rthwif_test.cpp ../rthwif_production.cpp)
TARGET_LINK_LIBRARIES(rthwif_test embree embree_rthwif)  # FIXME: remove
SET_PROPERTY(TARGET rthwif_test APPEND PROPERTY COMPILE_FLAGS "-fsycl -fsycl-targets=spir64 -DEMBREE_DPCPP_SUPPORT -DEMBREE_DPCPP_IMPLICIT_DISPATCH_GLOBALS")
SET_PROPERTY(TARGET rthwif_test APPEND PROPERTY LINK_FLAGS "-fsycl")

IF (EMBREE_DPCPP_IMPLICIT_DISPATCH_GLOBALS OR NOT DEFINED EMBREE_DPCPP_IMPLICIT_DISPATCH_GLOBALS)

  SET(OPTION BUILD_TESTING ON)
  INCLUDE(CTest)
  
  ADD_TEST(NAME rthwif_cornell_box
           COMMAND rthwif_cornell_box
           WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
         
  ADD_TEST(NAME rthwif_test_triangles_committed_hit        COMMAND rthwif_test --triangles-committed-hit)
  ADD_TEST(NAME rthwif_test_triangles_potential_hit        COMMAND rthwif_test --triangles-potential-hit)
  ADD_TEST(NAME rthwif_test_triangles_anyhit_shader_commit COMMAND rthwif_test --triangles-anyhit-shader-commit)
  ADD_TEST(NAME rthwif_test_triangles_anyhit_shader_reject COMMAND rthwif_test --triangles-anyhit-shader-reject)

ENDIF()
