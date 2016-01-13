## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
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

ENABLE_TESTING()
INCLUDE(CTest)

MACRO (ADD_EMBREE_TEST name executable)
  ADD_TEST(NAME triangle_geometry
           WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
           COMMAND ${PROJECT_SOURCE_DIR}/scripts/invoke_test.py
                   --name ${name}
                   --modeldir /Users/swoop/Work/models/embree-models
                   --model default
                   --execute ${PROJECT_BINARY_DIR}/${executable})

#SET_TESTS_PROPERTIES(triangle_geometry PROPERTIES ATTACHED_FILES "triangle_geometry.tga")
ENDMACRO()

