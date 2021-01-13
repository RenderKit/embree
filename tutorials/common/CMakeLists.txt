## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_TUTORIALS_GLFW)
  ADD_SUBDIRECTORY(imgui)
ENDIF()

ADD_SUBDIRECTORY(tutorial)
ADD_SUBDIRECTORY(scenegraph)
ADD_SUBDIRECTORY(lights)
ADD_SUBDIRECTORY(texture)

