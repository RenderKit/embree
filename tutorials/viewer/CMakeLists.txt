## ======================================================================== ##
## Copyright 2009-2018 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

INCLUDE(tutorial)
ADD_TUTORIAL(viewer)
ADD_TUTORIAL_ISPC(viewer)

IF (EMBREE_TESTING_INTENSITY GREATER 1)
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

IF (EMBREE_TESTING_INTENSITY GREATER 2)
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

IF (EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-subdiv.txt     viewer viewer viewer)
  ADD_EMBREE_MODELS_TEST(test-models-intensity2.txt viewer viewer viewer)
ENDIF()

IF (EMBREE_TESTING_INTENSITY GREATER 2)
  ADD_EMBREE_MODELS_TEST(test-models-intensity3.txt viewer viewer viewer)
ENDIF()

CMAKE_HOST_SYSTEM_INFORMATION(RESULT memory QUERY TOTAL_PHYSICAL_MEMORY)
IF (EMBREE_TESTING_INTENSITY GREATER 3 AND ${memory} GREATER 10000)
  ADD_EMBREE_MODELS_TEST(test-models-intensity4.txt viewer viewer viewer)
ENDIF()
