## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
  ADD_DEFINITIONS("-DUSE_SPECIALIZATION_CONSTANTS")
ENDIF()

INCLUDE(tutorial)
ADD_TUTORIAL(viewer viewer_device_debug.cpp)
ADD_TUTORIAL_ISPC(viewer viewer_device_debug.ispc)
ADD_TUTORIAL_SYCL(viewer viewer_device_debug.cpp)

SET(viewer_tests 
  "models/curves/curve0.ecs"
  "models/curves/curve1.ecs"
  "models/curves/curve2.ecs"
  "models/curves/curve3.ecs"
  "models/curves/curve4.ecs"
  "models/curves/curve5.ecs"
  "models/curves/hair0.ecs"
  "models/curves/hair1.ecs"
  "models/curves/oriented_curve0.ecs"
  "models/curves/oriented_curve1.ecs"
  "models/curves/oriented_curve2.ecs"
  "models/curves/oriented_curve3.ecs"
  "models/curves/oriented_curve4.ecs"
  "models/curves/oriented_bspline_curve_twisted.ecs"
  "models/curves/oriented_hermite_curve_twisted.ecs"
  "models/curves/round_line_segments_0.ecs"
  "models/curves/round_line_segments_1.ecs"
  "models/curves/round_line_segments_2.ecs"
  "models/curves/round_line_segments_3.ecs"
  "models/curves/round_line_segments_4.ecs"
  "models/curves/round_line_segments_5.ecs"
  "models/curves/round_line_segments_6.ecs"
  "models/curves/round_line_segments_7.ecs"
  "models/curves/round_line_segments_8.ecs"
  "models/curves/round_line_segments_9.ecs"
  "models/curves/min_width_flat_linear_curves.ecs"
  "models/curves/min_width_round_linear_curves.ecs"
  "models/curves/min_width_flat_bezier_curves.ecs"
  "models/curves/min_width_round_bezier_curves.ecs"
  "models/curves/min_width_oriented_hermite_curves.ecs"

  "models/furball/furball_bezier_flat.ecs"
  "models/furball/furball_bezier_normaloriented.ecs"
  "models/furball/furball_bezier_round.ecs"
  "models/furball/furball_bspline_flat.ecs"
  "models/furball/furball_bspline_normaloriented.ecs"
  "models/furball/furball_bspline_round.ecs"
  "models/furball/furball_hermite_flat.ecs"
  "models/furball/furball_hermite_normaloriented.ecs"
  #"models/furball/furball_hermite_round.ecs"
  "models/furball/furball_catmulrom_flat.ecs"
  "models/furball/furball_catmulrom_normaloriented.ecs"
  "models/furball/furball_catmulrom_round.ecs"
  "models/furball/furball_linear_flat.ecs"
  "models/furball/furball_linear_round.ecs"
  "models/furball/spheres_grids.ecs"
  "models/furball/spheres_points.ecs"
  "models/furball/spheres_quads.ecs"
  "models/furball/spheres_triangles.ecs"

  "models/msmblur/curves_msmblur.ecs"
  "models/msmblur/curves_msmblur2.ecs"
  "models/msmblur/lines_msmblur.ecs"
  "models/msmblur/lines_msmblur2.ecs"
  "models/msmblur/mblur_time_range_triangle.ecs"
  "models/msmblur/mblur_time_range_quad.ecs"
  "models/msmblur/mblur_time_range_grid.ecs"
  "models/msmblur/mblur_time_range_curve.ecs"
  "models/msmblur/mblur_time_range_line.ecs"
  "models/msmblur/mblur_time_range_instancing.ecs"
  "models/msmblur/mblur_time_range_sphere.ecs"
  "models/msmblur/mblur_time_range_disc.ecs"
  "models/msmblur/mblur_time_range_oriented_disc.ecs"
  "models/msmblur/linear_instance_linear_triangle.ecs"

  "models/points/points.ecs"
  "models/points/min_width_spheres.ecs"
  "models/points/min_width_discs.ecs"
  "models/points/min_width_oriented_discs.ecs"

  "models/quaternion_motion_blur/quaternion_quad.ecs"

  "models/subdiv/subdiv0.ecs"
  "models/subdiv/subdiv1.ecs"
  "models/subdiv/subdiv3.ecs"
  "models/subdiv/subdiv4.ecs"
  "models/subdiv/subdiv5.ecs"
  "models/subdiv/subdiv6.ecs"
  "models/subdiv/subdiv7.ecs"
  "models/subdiv/subdiv8.ecs"
  "models/subdiv/subdiv9.ecs"
  "models/subdiv/subdiv_no_boundary.ecs"
  "models/subdiv/subdiv_smooth_boundary.ecs"
  "models/subdiv/subdiv_pin_corners.ecs"
  "models/subdiv/subdiv_pin_boundary.ecs"
  "models/subdiv/subdiv_pin_all.ecs"
  "models/subdiv/cylinder.ecs"
)


foreach(t ${viewer_tests})
  STRING(REPLACE "/" "_" testname "${t}")
  ADD_EMBREE_TEST_ECS("viewer_${testname}" embree_viewer ECS "${t}" INTENSITY 2)
endforeach()

foreach(t ${viewer_tests})
  STRING(REPLACE "/" "_" testname "${t}")
  ADD_EMBREE_TEST_ECS("viewer_coherent_${testname}" embree_viewer ECS "${t}" INTENSITY 2 ARGS --coherent)
endforeach()

foreach(t ${viewer_tests})
  STRING(REPLACE "/" "_" testname "${t}")

  # hair 0 and 1 are special with quads and grids
  if ((t STREQUAL "models/curves/hair0.ecs") OR (t STREQUAL "models/curves/hair1.ecs"))
    ADD_EMBREE_TEST_ECS("viewer_quad_coherent_${testname}" embree_viewer ECS "${t}" REFERENCE_SUFFIX "_quad" INTENSITY 2 ARGS --coherent -convert-triangles-to-quads)
  else()
    ADD_EMBREE_TEST_ECS("viewer_quad_coherent_${testname}" embree_viewer ECS "${t}" INTENSITY 2 ARGS --coherent -convert-triangles-to-quads)
  endif()
endforeach()

foreach(t ${viewer_tests})
  STRING(REPLACE "/" "_" testname "${t}")

  # hair 0 and 1 are special with quads and grids
  if ((t STREQUAL "models/curves/hair0.ecs") OR (t STREQUAL "models/curves/hair1.ecs"))
    ADD_EMBREE_TEST_ECS("viewer_grid_coherent_${testname}" embree_viewer ECS "${t}" REFERENCE_SUFFIX "_grid" INTENSITY 2 ARGS --coherent -convert-triangles-to-grids)
  else()
    ADD_EMBREE_TEST_ECS("viewer_grid_coherent_${testname}" embree_viewer ECS "${t}" INTENSITY 2 ARGS --coherent -convert-triangles-to-grids)
  endif()
endforeach()