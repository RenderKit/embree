## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(viewer)
ADD_TUTORIAL_ISPC(viewer)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_coherent viewer viewer --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_incoherent viewer viewer --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_quad_coherent viewer viewer -convert-triangles-to-quads)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_quad_incoherent viewer viewer -convert-triangles-to-quads --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_grid_coherent viewer viewer -convert-triangles-to-grids)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_grid_incoherent viewer viewer -convert-triangles-to-grids --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_low_quality_scene viewer viewer -rtcore quality=low)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_medium_quality_scene viewer viewer -rtcore quality=medium)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_high_quality_scene viewer viewer -rtcore quality=high)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_robust_scene viewer viewer -rtcore accel_flags=robust)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_compact_scene viewer viewer -rtcore accel_flags=compact)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 2)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_coherent viewer viewer --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_incoherent viewer viewer --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_quad_coherent viewer viewer -convert-triangles-to-quads)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_quad_incoherent viewer viewer -convert-triangles-to-quads --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_grid_coherent viewer viewer -convert-triangles-to-grids)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_grid_incoherent viewer viewer -convert-triangles-to-grids --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_low_quality_scene viewer viewer -rtcore quality=low)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_medium_quality_scene viewer viewer -rtcore quality=medium)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_high_quality_scene viewer viewer -rtcore quality=high)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_robust_scene viewer viewer -rtcore accel_flags=robust)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_compact_scene viewer viewer -rtcore accel_flags=compact)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-subdiv.txt     viewer viewer viewer)
  ADD_EMBREE_MODELS_TEST(test-models-curves.txt     viewer viewer viewer)
  ADD_EMBREE_MODELS_TEST(test-models-intensity2.txt viewer viewer viewer)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 2)
  ADD_EMBREE_MODELS_TEST(test-models-intensity3.txt viewer viewer viewer)
ENDIF()

CMAKE_HOST_SYSTEM_INFORMATION(RESULT memory QUERY TOTAL_PHYSICAL_MEMORY)
IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 3 AND ${memory} GREATER 10000)
  ADD_EMBREE_MODELS_TEST(test-models-intensity4.txt viewer viewer viewer)
ENDIF()

IF (EMBREE_MIN_WIDTH AND EMBREE_TESTING_INTENSITY GREATER 1)

  ADD_EMBREE_MODEL_TEST(viewer_min_width_spheres viewer_min_width_spheres viewer "" points/min_width_spheres.ecs)
  ADD_EMBREE_MODEL_TEST(viewer_min_width_discs viewer_min_width_discs viewer "" points/min_width_discs.ecs)
  ADD_EMBREE_MODEL_TEST(viewer_min_width_oriented_discs viewer_min_width_oriented_discs viewer "" points/min_width_oriented_discs.ecs)

  ADD_EMBREE_MODEL_TEST(viewer_min_width_flat_linear_curves viewer_min_width_flat_linear_curves viewer "" curves/min_width_flat_linear_curves.ecs)
  ADD_EMBREE_MODEL_TEST(viewer_min_width_round_linear_curves viewer_min_width_round_linear_curves viewer "" curves/min_width_round_linear_curves.ecs)
  ADD_EMBREE_MODEL_TEST(viewer_min_width_flat_bezier_curves viewer_min_width_flat_bezier_curves viewer "" curves/min_width_flat_bezier_curves.ecs)
  ADD_EMBREE_MODEL_TEST(viewer_min_width_round_bezier_curves viewer_min_width_round_bezier_curves viewer "" curves/min_width_round_bezier_curves.ecs)
  ADD_EMBREE_MODEL_TEST(viewer_min_width_oriented_hermite_curves viewer_min_width_oriented_hermite_curves viewer "" curves/min_width_oriented_hermite_curves.ecs)
  
ENDIF()
