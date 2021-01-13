## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(next_hit)

MACRO (ALL_HITS_TEST name model max_next_hits)
  ADD_TEST(NAME next_hit_${name}_${max_next_hits} COMMAND next_hit -c ${EMBREE_MODEL_DIR}/${model} -c ${EMBREE_MODEL_DIR}/${model} --verify --max_next_hits ${max_next_hits} --max_total_hits 1024 ${ARGN} --shader default -o test.tga)
ENDMACRO ()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  IF (CMAKE_SIZEOF_VOID_P EQUAL 8) # FIXME: these cause out of memory under Win32
    ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 1)
    ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 2)
    ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 3)
    ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 4)
    ALL_HITS_TEST(quads_robust_crown crown/crown.ecs 1 --convert-triangles-to-quads)
    ALL_HITS_TEST(grids_robust_crown crown/crown.ecs 1 --convert-triangles-to-grids)
  ENDIF()
  ALL_HITS_TEST(flat_curve_robust_tighten tighten/tighten.ecs 1)
  ALL_HITS_TEST(flat_curve_robust curves/hair0.ecs 1)
#  ALL_HITS_TEST(round_curve_robust curves/hair0.ecs 1 --convert-flat-to-round-curves)
  ALL_HITS_TEST(oriented_curve_robust curves/oriented_curve0.ecs 1)
  ALL_HITS_TEST(flat_linear_curve_robust curves/hair0.ecs 1 --convert-bezier-to-lines)
ENDIF()

