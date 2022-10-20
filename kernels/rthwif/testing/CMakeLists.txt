## Copyright 2009-2022 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PROJECT(rthwif_testing)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

SET(CMAKE_CXX_STANDARD 17)

IF (NOT DEFINED EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS)
  OPTION(EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS "Using L0 allocated Dispatch Globals" ON)
ENDIF()

IF (NOT EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS)
  ADD_DEFINITIONS("-DEMBREE_SYCL_ALLOC_DISPATCH_GLOBALS")
ENDIF()

IF (NOT EMBREE_SYCL_GPU_BVH_BUILDER)
  ADD_DEFINITIONS("-DEMBREE_SYCL_GPU_BVH_BUILDER")
ENDIF()

ADD_EXECUTABLE(rthwif_cornell_box rthwif_cornell_box.cpp ../rthwif_production.cpp)
SET_PROPERTY(TARGET rthwif_cornell_box APPEND PROPERTY COMPILE_FLAGS "-fsycl -fsycl-targets=spir64 -DEMBREE_SYCL_SUPPORT")
SET_PROPERTY(TARGET rthwif_cornell_box APPEND PROPERTY LINK_FLAGS "-fsycl -lze_loader")

ADD_EXECUTABLE(rthwif_test rthwif_test.cpp ../rthwif_production.cpp)
TARGET_LINK_LIBRARIES(rthwif_test embree_rthwif sys tbb)
TARGET_LINK_LIBRARIES(rthwif_test embree_rthwif_gpu sys tbb)

SET_PROPERTY(TARGET rthwif_test APPEND PROPERTY COMPILE_FLAGS "-fsycl -fsycl-targets=spir64 -DEMBREE_SYCL_SUPPORT")
SET_PROPERTY(TARGET rthwif_test APPEND PROPERTY LINK_FLAGS "-fsycl -lze_loader")

SET(OPTION BUILD_TESTING ON)
INCLUDE(CTest)

IF (EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS OR NOT DEFINED EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS)

  ADD_TEST(NAME rthwif_cornell_box
           COMMAND rthwif_cornell_box --compare "${EMBREE_MODEL_DIR}/reference/cornell_box_reference.tga"
           WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

ENDIF()

ADD_TEST(NAME rthwif_test_builder_triangles_expected      COMMAND rthwif_test --build_test_triangles   --build_mode_expected)
ADD_TEST(NAME rthwif_test_builder_procedurals_expected    COMMAND rthwif_test --build_test_procedurals --build_mode_expected)
ADD_TEST(NAME rthwif_test_builder_instances_expected      COMMAND rthwif_test --build_test_instances   --build_mode_expected)
ADD_TEST(NAME rthwif_test_builder_mixed_expected          COMMAND rthwif_test --build_test_mixed       --build_mode_expected)

ADD_TEST(NAME rthwif_test_benchmark_triangles             COMMAND rthwif_test --benchmark_triangles)
ADD_TEST(NAME rthwif_test_benchmark_procedurals           COMMAND rthwif_test --benchmark_procedurals)

ADD_TEST(NAME rthwif_test_builder_triangles_worst_case      COMMAND rthwif_test --build_test_triangles   --build_mode_worst_case)
ADD_TEST(NAME rthwif_test_builder_procedurals_worst_case    COMMAND rthwif_test --build_test_procedurals --build_mode_worst_case)
ADD_TEST(NAME rthwif_test_builder_instances_worst_case      COMMAND rthwif_test --build_test_instances   --build_mode_worst_case)
ADD_TEST(NAME rthwif_test_builder_mixed_worst_case          COMMAND rthwif_test --build_test_mixed       --build_mode_worst_case)

ADD_TEST(NAME rthwif_test_triangles_committed_hit        COMMAND rthwif_test --no-instancing --triangles-committed-hit)
ADD_TEST(NAME rthwif_test_triangles_potential_hit        COMMAND rthwif_test --no-instancing --triangles-potential-hit)
ADD_TEST(NAME rthwif_test_triangles_anyhit_shader_commit COMMAND rthwif_test --no-instancing --triangles-anyhit-shader-commit)
ADD_TEST(NAME rthwif_test_triangles_anyhit_shader_reject COMMAND rthwif_test --no-instancing --triangles-anyhit-shader-reject)
ADD_TEST(NAME rthwif_test_procedurals_committed_hit      COMMAND rthwif_test --no-instancing --procedurals-committed-hit)

ADD_TEST(NAME rthwif_test_hwinstancing_triangles_committed_hit        COMMAND rthwif_test --hw-instancing --triangles-committed-hit)
ADD_TEST(NAME rthwif_test_hwinstancing_triangles_potential_hit        COMMAND rthwif_test --hw-instancing --triangles-potential-hit)
ADD_TEST(NAME rthwif_test_hwinstancing_triangles_anyhit_shader_commit COMMAND rthwif_test --hw-instancing --triangles-anyhit-shader-commit)
ADD_TEST(NAME rthwif_test_hwinstancing_triangles_anyhit_shader_reject COMMAND rthwif_test --hw-instancing --triangles-anyhit-shader-reject)
ADD_TEST(NAME rthwif_test_hwinstancing_procedurals_committed_hit      COMMAND rthwif_test --hw-instancing --procedurals-committed-hit)

ADD_TEST(NAME rthwif_test_swinstancing_triangles_committed_hit        COMMAND rthwif_test --sw-instancing --triangles-committed-hit)
ADD_TEST(NAME rthwif_test_swinstancing_triangles_potential_hit        COMMAND rthwif_test --sw-instancing --triangles-potential-hit)
ADD_TEST(NAME rthwif_test_swinstancing_triangles_anyhit_shader_commit COMMAND rthwif_test --sw-instancing --triangles-anyhit-shader-commit)
ADD_TEST(NAME rthwif_test_swinstancing_triangles_anyhit_shader_reject COMMAND rthwif_test --sw-instancing --triangles-anyhit-shader-reject)
ADD_TEST(NAME rthwif_test_swinstancing_procedurals_committed_hit      COMMAND rthwif_test --sw-instancing --procedurals-committed-hit)

