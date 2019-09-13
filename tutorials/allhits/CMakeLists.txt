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

IF (EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_TEST(NAME allhits_fast_crown_1 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 1 -o allhits_fast_crown_1.tga)
  ADD_TEST(NAME allhits_fast_crown_2 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 2 -o allhits_fast_crown_2.tga)
  ADD_TEST(NAME allhits_fast_crown_3 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 3 -o allhits_fast_crown_3.tga)
  ADD_TEST(NAME allhits_fast_crown_4 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 4 -o allhits_fast_crown_4.tga)
  ADD_TEST(NAME allhits_robust_crown_1 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 1 --rtcore scene_flags=robust -o allhits_robust_crown_1.tga)
  ADD_TEST(NAME allhits_robust_crown_2 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 2 --rtcore scene_flags=robust -o allhits_robust_crown_2.tga)
  ADD_TEST(NAME allhits_robust_crown_3 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 3 --rtcore scene_flags=robust -o allhits_robust_crown_3.tga)
  ADD_TEST(NAME allhits_robust_crown_4 COMMAND allhits -c ${EMBREE_MODEL_DIR}/crown/crown.ecs --verify --num_hits 4 --rtcore scene_flags=robust -o allhits_robust_crown_4.tga)
ENDIF()

