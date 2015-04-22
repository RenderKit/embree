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

PROJECT(embreeinfo)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

FIND_PACKAGE(embree 2.0 REQUIRED)
INCLUDE_DIRECTORIES(${EMBREE_INCLUDE_DIRS})

ADD_EXECUTABLE(embreeinfo embreeinfo.cpp)
TARGET_LINK_LIBRARIES(embreeinfo ${EMBREE_LIBRARY})
