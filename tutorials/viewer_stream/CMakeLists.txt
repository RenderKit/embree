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
ADD_TUTORIAL(viewer_stream)
ADD_TUTORIAL_ISPC(viewer_stream)

IF (EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_coherent viewer viewer_stream --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_incoherent viewer viewer_stream --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_quad_coherent viewer viewer_stream -convert-triangles-to-quads)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_quad_incoherent viewer viewer_stream -convert-triangles-to-quads --incoherent)
ENDIF()

IF (EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_coherent viewer viewer_stream --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_incoherent viewer viewer_stream --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_quad_coherent viewer viewer_stream -convert-triangles-to-quads)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_quad_incoherent viewer viewer_stream -convert-triangles-to-quads --incoherent)
ENDIF()

IF (EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-subdiv.txt viewer_stream viewer viewer_stream)
  ADD_EMBREE_MODELS_TEST(test-models-intensity2.txt viewer_stream viewer viewer_stream)
ENDIF()

IF (EMBREE_TESTING_INTENSITY GREATER 3)
  ADD_EMBREE_MODELS_TEST(test-models-intensity3.txt viewer_stream viewer viewer_stream)
ENDIF()

CMAKE_HOST_SYSTEM_INFORMATION(RESULT memory QUERY TOTAL_PHYSICAL_MEMORY)
IF (EMBREE_TESTING_INTENSITY GREATER 3 AND ${memory} GREATER 10000)
  ADD_EMBREE_MODELS_TEST(test-models-intensity4.txt viewer_stream viewer viewer_stream)
ENDIF()
