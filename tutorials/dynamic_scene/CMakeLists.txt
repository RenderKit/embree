## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(dynamic_scene)
ADD_TUTORIAL_ISPC(dynamic_scene)
ADD_TUTORIAL_SYCL(dynamic_scene)

ADD_EMBREE_TEST_ECS(dynamic_scene embree_dynamic_scene)