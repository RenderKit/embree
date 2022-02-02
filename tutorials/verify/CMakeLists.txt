## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF(NOT EMBREE_API_NAMESPACE)
  ADD_LIBRARY(c99_compile_test STATIC api_c99.c)
ENDIF()

ADD_EXECUTABLE(embree_verify ../../kernels/embree.rc verify.cpp ../common/tutorial/application.cpp ../../kernels/common/geometry.cpp)
TARGET_LINK_LIBRARIES(embree_verify sys math scenegraph embree tasking)
SET_PROPERTY(TARGET embree_verify PROPERTY FOLDER tutorials)
SET_PROPERTY(TARGET embree_verify APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS embree_verify DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
SIGN_TARGET(embree_verify)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)

  IF (EMBREE_TESTING_INTENSITY GREATER 1)
    ADD_TEST(NAME embree_verify COMMAND embree_verify --no-colors --intensity 2)
  ELSE()
    ADD_TEST(NAME embree_verify COMMAND embree_verify --no-colors)
  ENDIF()
  
  SET_TESTS_PROPERTIES(embree_verify PROPERTIES TIMEOUT 7000)

  IF (EMBREE_TESTING_MEMCHECK)
    ADD_MEMCHECK_TEST(embree_verify_memcheck embree_verify 
       --no-colors --intensity 0.1 
       --skip .*memory_consumption.*
       --skip .*regression_.*_build_join  # causes some issues with TBB
       --skip .*SSE4.*              # to run faster
       --skip .*AVX.*               # valgrind does not support AVX
       --skip .*AVX2.*              # valgrind does not support AVX2
       --skip .*AVX512.*         # valgrind does not support AVX512
    )
    set_tests_properties(embree_verify_memcheck PROPERTIES TIMEOUT 15000)
  ENDIF()

  IF (EMBREE_TESTING_BENCHMARK)
    ADD_TEST(NAME embree_verify_benchmarks COMMAND embree_verify
      --no-colors --cdash
      --benchmark-tolerance 0.05
      --database "${EMBREE_TESTING_BENCHMARK_DATABASE}"
      --run .*benchmarks.*
      --skip .*_120.* --skip .*_1k.* --skip .*_10k.* --skip .*100k.*  # skip all smaller build benchmarks
      --run .*embree_reported_memory.*
    )
    set_tests_properties(embree_verify_benchmarks PROPERTIES TIMEOUT 10800)
  ENDIF()
ENDIF()
