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

ADD_EXECUTABLE(convert convert.cpp distribution1d.cpp distribution2d.cpp)
TARGET_LINK_LIBRARIES(convert scenegraph image tasking)
SET_PROPERTY(TARGET convert PROPERTY FOLDER tutorials/single)
SET_PROPERTY(TARGET convert APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS convert DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
SIGN_TARGET(convert)
