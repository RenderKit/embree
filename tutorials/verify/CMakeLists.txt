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

IF(NOT EMBREE_API_NAMESPACE)
  ADD_LIBRARY(c99_compile_test STATIC api_c99.c)
ENDIF()

ADD_EXECUTABLE(verify verify.cpp ../common/tutorial/application.cpp)
TARGET_LINK_LIBRARIES(verify sys math scenegraph embree)
SET_PROPERTY(TARGET verify PROPERTY FOLDER tutorials)
SET_PROPERTY(TARGET verify APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS verify DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
SIGN_TARGET(verify)

IF (EMBREE_TESTING_INTENSITY GREATER 0)

  IF (EMBREE_TESTING_INTENSITY GREATER 1)
    ADD_TEST(NAME verify COMMAND verify --no-colors --intensity 2)
  ELSE()
    ADD_TEST(NAME verify COMMAND verify --no-colors)
  ENDIF()
  
  SET_TESTS_PROPERTIES(verify PROPERTIES TIMEOUT 7000)

  IF (EMBREE_TESTING_MEMCHECK)
    ADD_MEMCHECK_TEST(verify_memcheck verify 
       --no-colors --intensity 0.1 
       --skip .*memory_consumption.*
       --skip .*regression_.*_build_join  # causes some issues with TBB
       --skip .*SSE4.*              # to run faster
       --skip .*AVX.*               # valgrind does not support AVX
       --skip .*AVX2.*              # valgrind does not support AVX2
       --skip .*AVX512KNL.*         # valgrind does not support AVX512KNL
       --skip .*AVX512SKX.*         # valgrind does not support AVX512SKX
    )
    set_tests_properties(verify_memcheck PROPERTIES TIMEOUT 10800)
  ENDIF()

  IF (EMBREE_TESTING_BENCHMARK)
    ADD_TEST(NAME verify_benchmarks COMMAND verify
      --no-colors --cdash
      --benchmark-tolerance 0.05
      --database "${EMBREE_TESTING_BENCHMARK_DATABASE}"
      --run .*benchmarks.*
      --skip .*_120.* --skip .*_1k.* --skip .*_10k.* --skip .*100k.*  # skip all smaller build benchmarks
      --run .*embree_reported_memory.*
    )
    set_tests_properties(verify_benchmarks PROPERTIES TIMEOUT 10800)
  ENDIF()
ENDIF()
