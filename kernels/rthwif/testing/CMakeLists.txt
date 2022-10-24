## Copyright 2009-2022 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_EXECUTABLE(rthwif_cornell_box rthwif_cornell_box.cpp ../rthwif_production.cpp)
SET_PROPERTY(TARGET rthwif_cornell_box APPEND PROPERTY COMPILE_FLAGS "-fsycl -fsycl-targets=spir64 -DEMBREE_SYCL_SUPPORT")
SET_PROPERTY(TARGET rthwif_cornell_box APPEND PROPERTY LINK_FLAGS "-fsycl -lze_loader")

ADD_EXECUTABLE(rthwif_test rthwif_test.cpp ../rthwif_production.cpp)
TARGET_LINK_LIBRARIES(rthwif_test embree_rthwif sys simd tbb)
SET_PROPERTY(TARGET rthwif_test APPEND PROPERTY COMPILE_FLAGS "-fsycl -fsycl-targets=spir64 -DEMBREE_SYCL_SUPPORT")
SET_PROPERTY(TARGET rthwif_test APPEND PROPERTY LINK_FLAGS "-fsycl -lze_loader")

IF (EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS)

  IF (DEFINED EMBREE_MODEL_DIR)
    SET(CORNELL_BOX_REFERENCE "${EMBREE_MODEL_DIR}/reference/cornell_box_reference.tga")
  ELSE()
    SET(CORNELL_BOX_REFERENCE "${CMAKE_CURRENT_SOURCE_DIR}/cornell_box_reference.tga")
  ENDIF()
  
  ADD_TEST(NAME rthwif_cornell_box
           COMMAND rthwif_cornell_box --compare "${CORNELL_BOX_REFERENCE}"
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

