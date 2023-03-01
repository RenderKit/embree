## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
  ADD_DEFINITIONS("-DUSE_SPECIALIZATION_CONSTANTS")
ENDIF()

INCLUDE(tutorial)
ADD_TUTORIAL(next_hit)
ADD_TUTORIAL_SYCL(next_hit)

MACRO (ALL_HITS_TEST name max_next_hits)
  # uses the default cornell box 
  ADD_TEST(NAME next_hit_${name}_${max_next_hits} COMMAND embree_next_hit --verify --max_next_hits ${max_next_hits} --max_total_hits 1024 ${ARGN} -o test.tga)

  IF (EMBREE_SYCL_SUPPORT)
    ADD_TEST(NAME next_hit_sycl_${name}_${max_next_hits} COMMAND embree_next_hit_sycl --verify --max_next_hits ${max_next_hits} --max_total_hits 1024 ${ARGN} -o test.tga)
  ENDIF()
ENDMACRO ()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ALL_HITS_TEST(triangle_robust_cornell_box 1)
  ALL_HITS_TEST(triangle_robust_cornell_box 2)
  ALL_HITS_TEST(triangle_robust_cornell_box 3)
  ALL_HITS_TEST(triangle_robust_cornell_box 4)
  ALL_HITS_TEST(quads_robust_cornell_box 1 --convert-triangles-to-quads)
  ALL_HITS_TEST(grids_robust_cornell_box 1 --convert-triangles-to-grids)
ENDIF()
