## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_EXECUTABLE(embree_tests ../../kernels/embree.rc
  embree_tests.cpp
  kernels/kernels_tests.cpp
  common/common_tests.cpp)
TARGET_LINK_LIBRARIES(embree_tests tasking sys math scenegraph embree)
SET_PROPERTY(TARGET embree_tests PROPERTY FOLDER tutorials)
SET_PROPERTY(TARGET embree_tests APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS embree_tests DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
SIGN_TARGET(embree_tests)

IF (BUILD_TESTING)

  ADD_TEST(NAME embree_tests COMMAND embree_tests)
  SET_TESTS_PROPERTIES(embree_tests PROPERTIES TIMEOUT 7000)

ENDIF()
