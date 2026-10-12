## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_EXECUTABLE(embree_regression_tests ../../kernels/embree.rc embree_regression_tests.cpp)
TARGET_LINK_LIBRARIES(embree_regression_tests embree)
SET_PROPERTY(TARGET embree_regression_tests PROPERTY FOLDER tutorials)
SET_PROPERTY(TARGET embree_regression_tests APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS embree_regression_tests DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
SIGN_TARGET(embree_regression_tests)

ADD_EMBREE_TEST_ECS(embree_regression_tests embree_regression_tests NO_REFERENCE NO_ISPC NO_SYCL)
SET_EMBREE_TEST_PROPERTIES(embree_regression_tests PROPERTIES TIMEOUT 7000)
