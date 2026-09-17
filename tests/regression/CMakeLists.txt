# Copyright 2009-2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

add_executable(embree_regression_morton_builder_clamp morton_builder_clamp_regression.cpp)
target_link_libraries(embree_regression_morton_builder_clamp PRIVATE embree)
set_property(TARGET embree_regression_morton_builder_clamp PROPERTY FOLDER tests/regression)

if (EMBREE_GEOMETRY_USER)
  add_executable(embree_regression_oriented_user_geometry oriented_user_geometry_regression.cpp)
  target_link_libraries(embree_regression_oriented_user_geometry PRIVATE embree)
  set_property(TARGET embree_regression_oriented_user_geometry PROPERTY FOLDER tests/regression)
endif()

if (BUILD_TESTING)
  add_test(NAME regression_morton_builder_clamp COMMAND embree_regression_morton_builder_clamp)
  if (EMBREE_GEOMETRY_USER)
    add_test(NAME regression_oriented_user_geometry COMMAND embree_regression_oriented_user_geometry)
  endif()
endif()
