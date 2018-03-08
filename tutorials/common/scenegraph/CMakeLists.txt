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

ADD_LIBRARY(scenegraph STATIC
    xml_parser.cpp
    xml_loader.cpp
    xml_writer.cpp
    obj_loader.cpp
    ply_loader.cpp
    corona_loader.cpp
    texture.cpp
    scenegraph.cpp
    geometry_creation.cpp)

TARGET_LINK_LIBRARIES(scenegraph sys math lexers image embree)
SET_PROPERTY(TARGET scenegraph PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET scenegraph APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
