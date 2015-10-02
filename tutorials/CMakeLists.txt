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

ADD_SUBDIRECTORY(common)

SET (__XEON__ 1)
ADD_SUBDIRECTORY(triangle_geometry)
ADD_SUBDIRECTORY(dynamic_scene)
ADD_SUBDIRECTORY(user_geometry)
ADD_SUBDIRECTORY(viewer)
ADD_SUBDIRECTORY(instanced_geometry)
ADD_SUBDIRECTORY(intersection_filter)
ADD_SUBDIRECTORY(pathtracer)
ADD_SUBDIRECTORY(hair_geometry)
ADD_SUBDIRECTORY(subdivision_geometry)
ADD_SUBDIRECTORY(displacement_geometry)
ADD_SUBDIRECTORY(bvh_builder)
ADD_SUBDIRECTORY(lazy_geometry)
ADD_SUBDIRECTORY(bvh_access)
ADD_SUBDIRECTORY(motion_blur_geometry)
ADD_SUBDIRECTORY(interpolation)
ADD_SUBDIRECTORY(convert)

IF (ENABLE_XEON_PHI_SUPPORT AND RTCORE_RAY_PACKETS)
  SET (__XEON__ 0)
  SET (__HOST__ 1)
  ADD_SUBDIRECTORY(triangle_geometry triangle_geometry_host)
  ADD_SUBDIRECTORY(dynamic_scene dynamic_scene_host)
  ADD_SUBDIRECTORY(user_geometry user_geometry_host)
  ADD_SUBDIRECTORY(viewer viewer_host)
  ADD_SUBDIRECTORY(instanced_geometry instanced_geometry_host)
  ADD_SUBDIRECTORY(intersection_filter intersection_filter_host)
  ADD_SUBDIRECTORY(pathtracer pathtracer_host)
  ADD_SUBDIRECTORY(hair_geometry hair_geometry_host)
  ADD_SUBDIRECTORY(subdivision_geometry subdivision_geometry_host)
  ADD_SUBDIRECTORY(displacement_geometry displacement_geometry_host)
  ADD_SUBDIRECTORY(motion_blur_geometry motion_blur_geometry_host)
  ADD_SUBDIRECTORY(interpolation interpolation_host)
  SET (__HOST__ 0)
  ADD_SUBDIRECTORY(triangle_geometry triangle_geometry_device)
  ADD_SUBDIRECTORY(dynamic_scene dynamic_scene_device)
  ADD_SUBDIRECTORY(user_geometry user_geometry_device)
  ADD_SUBDIRECTORY(viewer viewer_device)
  ADD_SUBDIRECTORY(instanced_geometry instanced_geometry_device)
  ADD_SUBDIRECTORY(intersection_filter intersection_filter_device)
  ADD_SUBDIRECTORY(pathtracer pathtracer_device)
  ADD_SUBDIRECTORY(hair_geometry hair_geometry_device)
  ADD_SUBDIRECTORY(subdivision_geometry subdivision_geometry_device)
  ADD_SUBDIRECTORY(displacement_geometry displacement_geometry_device)
  ADD_SUBDIRECTORY(motion_blur_geometry motion_blur_geometry_device)
  ADD_SUBDIRECTORY(interpolation interpolation_device)
ENDIF()

# propagate menu links up
SET(CPACK_NSIS_MENU_LINKS ${CPACK_NSIS_MENU_LINKS} PARENT_SCOPE)

