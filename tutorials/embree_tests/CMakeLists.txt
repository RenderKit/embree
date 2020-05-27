## ======================================================================== ##
## Copyright 2009-2020 Intel Corporation                                    ##
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


ADD_EXECUTABLE(embree_tests ../../kernels/embree.rc 
  embree_tests.cpp 
  kernels/kernels_tests.cpp)
TARGET_LINK_LIBRARIES(embree_tests tasking sys math scenegraph embree)
SET_PROPERTY(TARGET embree_tests PROPERTY FOLDER tutorials)
SET_PROPERTY(TARGET embree_tests APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS embree_tests DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
SIGN_TARGET(embree_tests)

IF (BUILD_TESTING)

  ADD_TEST(NAME embree_tests COMMAND embree_tests)
  SET_TESTS_PROPERTIES(embree_tests PROPERTIES TIMEOUT 7000)

ENDIF()
