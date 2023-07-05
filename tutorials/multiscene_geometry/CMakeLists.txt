## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

SET(EMBREE_ISPC_SUPPORT OFF)

INCLUDE(tutorial)
ADD_TUTORIAL(multiscene_geometry)
#ADD_TUTORIAL_ISPC(multiscene_geometry)
ADD_TUTORIAL_SYCL(multiscene_geometry)


ADD_EMBREE_TEST_ECS(multiscene_geometry embree_multiscene_geometry ARGS --compare-threshold 40)  # because of AVX512 in fedora32