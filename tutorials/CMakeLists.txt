## ======================================================================== ##
## Copyright 2009-2017 Intel Corporation                                    ##
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

ADD_SUBDIRECTORY(verify)
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
ADD_SUBDIRECTORY(curve_geometry)
ADD_SUBDIRECTORY(buildbench)

IF (EMBREE_RAY_PACKETS)
ADD_SUBDIRECTORY(viewer_stream)
ADD_SUBDIRECTORY(viewer_anim)
ENDIF()

# propagate menu links up
SET(CPACK_NSIS_MENU_LINKS ${CPACK_NSIS_MENU_LINKS} PARENT_SCOPE)

