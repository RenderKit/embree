## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(pathtracer)
ADD_TUTORIAL_ISPC(pathtracer)
ADD_TUTORIAL_SYCL(pathtracer)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt pathtracer_coherent pathtracer pathtracer --coherent)
#  ADD_EMBREE_MODELS_TEST(test-models-intensity2.txt pathtracer pathtracer pathtracer)

   IF (EMBREE_DPCPP_PVC)  # FIXME: fails on PVC
     set_tests_properties(pathtracer_coherent_crown_crown_sycl PROPERTIES DISABLED True)
   ENDIF()
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 2)
#  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt pathtracer_coherent pathtracer pathtracer --coherent)  # FIXME: triggers IGC bug with pathtracer of curve0 model
#  ADD_EMBREE_MODELS_TEST(test-models-intensity3.txt pathtracer pathtracer pathtracer)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 3)
  ADD_EMBREE_MODELS_TEST(test-models-intensity4.txt pathtracer pathtracer pathtracer)

  IF (EMBREE_DPCPP_SUPPORT AND EMBREE_DPCPP_MBLUR)
    set_tests_properties(pathtracer_sophie_mblur_sophie_mblur_sycl PROPERTIES DISABLED True)
  ENDIF()
ENDIF()
