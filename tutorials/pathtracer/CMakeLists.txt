## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
  ADD_DEFINITIONS("-DUSE_SPECIALIZATION_CONSTANTS")
ENDIF()

INCLUDE(tutorial)
ADD_TUTORIAL(pathtracer)
ADD_TUTORIAL_ISPC(pathtracer)
ADD_TUTORIAL_SYCL(pathtracer)


SET(pathtracer_tests
  "models/curves/curve0.ecs"
  "models/curves/curve1.ecs"
  "models/curves/curve2.ecs"
  "models/curves/curve3.ecs"
  "models/curves/curve4.ecs"
  #"models/curves/curve5.ecs"
  "models/curves/oriented_curve0.ecs"
  "models/curves/oriented_curve1.ecs"
  "models/curves/oriented_curve2.ecs"
  "models/curves/oriented_curve3.ecs"
  "models/curves/oriented_curve4.ecs"
  "models/curves/oriented_bspline_curve_twisted.ecs"
  "models/curves/oriented_hermite_curve_twisted.ecs"

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

  "models/points/points.ecs"

  "models/quaternion_motion_blur/quaternion_quad.ecs"
)


# EMBREE_SYCL_AOT_DEVICES != none   => see issue #1209
foreach(t ${pathtracer_tests})
  STRING(REPLACE "/" "_" testname "${t}")
  ADD_EMBREE_TEST_ECS("pathtracer_${testname}" embree_pathtracer ECS "${t}" INTENSITY 2 CONDITION "EMBREE_SYCL_AOT_DEVICES != none")
endforeach()

foreach(t ${pathtracer_tests})
  STRING(REPLACE "/" "_" testname "${t}")
  ADD_EMBREE_TEST_ECS("pathtracer_coherent_${testname}" embree_pathtracer ECS "${t}" INTENSITY 3 CONDITION "EMBREE_SYCL_AOT_DEVICES != none" ARGS --coherent)
endforeach()