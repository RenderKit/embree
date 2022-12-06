## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
  ADD_DEFINITIONS("-DUSE_SPECIALIZATION_CONSTANTS")
ENDIF()

INCLUDE(tutorial)
ADD_TUTORIAL(pathtracer)
ADD_TUTORIAL_ISPC(pathtracer)
ADD_TUTORIAL_SYCL(pathtracer)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt pathtracer_coherent pathtracer pathtracer --coherent)
#  ADD_EMBREE_MODELS_TEST(test-models-intensity2.txt pathtracer pathtracer pathtracer)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 2)
#  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt pathtracer_coherent pathtracer pathtracer --coherent)  # FIXME: triggers IGC bug with pathtracer of curve0 model
#  ADD_EMBREE_MODELS_TEST(test-models-intensity3.txt pathtracer pathtracer pathtracer)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 3)
  
  ADD_EMBREE_MODELS_TEST(test-models-intensity4.txt pathtracer pathtracer pathtracer)

  set_tests_properties(pathtracer_sophie_sophie PROPERTIES DISABLED True)
  set_tests_properties(pathtracer_sophie_mblur_sophie_mblur PROPERTIES DISABLED True)
  
  IF (EMBREE_ISPC_SUPPORT)
    set_tests_properties(pathtracer_sophie_sophie_ispc PROPERTIES DISABLED True)
    set_tests_properties(pathtracer_sophie_mblur_sophie_mblur_ispc PROPERTIES DISABLED True)
  ENDIF()

  IF (EMBREE_SYCL_SUPPORT)
    set_tests_properties(pathtracer_sophie_sophie_sycl PROPERTIES DISABLED True)
    set_tests_properties(pathtracer_sophie_mblur_sophie_mblur_sycl PROPERTIES DISABLED True)
  ENDIF()

ENDIF()

