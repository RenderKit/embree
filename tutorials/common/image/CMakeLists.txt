## ======================================================================== ##
## Copyright 2009-2013 Intel Corporation                                    ##
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

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

SET(USE_OPENEXR  OFF CACHE BOOL "Use OpenEXR library to load images.")
IF (USE_OPENEXR)
  FIND_PACKAGE(OpenEXR)
  IF(OPENEXR_FOUND)
    ADD_DEFINITIONS(-DUSE_OPENEXR)
    INCLUDE_DIRECTORIES(${OPENEXR_INCLUDE_PATHS})
  ENDIF(OPENEXR_FOUND)
ENDIF()

SET(USE_IMAGE_MAGICK  OFF CACHE BOOL "Use ImageMagick library to load images.")
IF (USE_IMAGE_MAGICK)
  FIND_PACKAGE(ImageMagick COMPONENTS Magick++)
  IF(IMAGEMAGICK_FOUND)
    ADD_DEFINITIONS(-DUSE_IMAGEMAGICK -DMAGICKCORE_HDRI_ENABLE=0 -DMAGICKCORE_QUANTUM_DEPTH=8)
    INCLUDE_DIRECTORIES(${ImageMagick_Magick++_INCLUDE_DIR})
  ENDIF(IMAGEMAGICK_FOUND)
ENDIF()

ADD_LIBRARY(image STATIC
  image.cpp
  ppm.cpp
  pfm.cpp
  tga.cpp
  exr.cpp
  magick.cpp
)
TARGET_LINK_LIBRARIES(image sys ${OPENEXR_LIBRARIES} ${ImageMagick_Magick++_LIBRARY})
