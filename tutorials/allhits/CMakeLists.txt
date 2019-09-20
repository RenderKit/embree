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
ADD_TUTORIAL(allhits)

MACRO (ALL_HITS_TEST name model num_hits      )
  ADD_TEST(NAME allhits_${name}_${num_hits} COMMAND allhits -c ${EMBREE_MODEL_DIR}/${model} -c ${EMBREE_MODEL_DIR}/${model} --verify --num_hits ${num_hits} --rtcore scene_flags=robust ${ARGN} --shader default -o test.tga)
ENDMACRO ()

IF (EMBREE_TESTING_INTENSITY GREATER 1)
  ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 1)
  ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 2)
  ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 3)
  ALL_HITS_TEST(triangle_robust_crown crown/crown.ecs 4)
  ALL_HITS_TEST(quads_robust_crown crown/crown.ecs 1 --convert_triangles_to_quads)
  ALL_HITS_TEST(grids_robust_crown crown/crown.ecs 1 --convert_triangles_to_grids)
  ALL_HITS_TEST(flat_curve_robust_tighten tighten/tighten.ecs 1)
  ALL_HITS_TEST(flat_curve_robust curves/hair0.ecs 1)
  ALL_HITS_TEST(round_curve_robust curves/hair0.ecs 1 --convert-flat-to-round-curves)
  ALL_HITS_TEST(oriented_curve_robust curves/oriented_curve0.ecs 1)
  ALL_HITS_TEST(flat_linear_curve_robust curves/hair0.ecs 1 --convert-bezier-to-lines)
ENDIF()

