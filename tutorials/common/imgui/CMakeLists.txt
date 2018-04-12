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

ADD_LIBRARY(imgui STATIC
  imgui.cpp
  imgui_impl_glfw_gl2.cpp
  imgui_demo.cpp
  imgui_draw.cpp
)

TARGET_LINK_LIBRARIES(imgui ${OPENGL_LIBRARIES})
SET_PROPERTY(TARGET imgui PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET imgui APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
