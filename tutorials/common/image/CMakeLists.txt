## ======================================================================== ##
## Copyright 2009-2014 Intel Corporation                                    ##
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

SET(ADDITIONAL_LIBRARIES)

SET(USE_IMAGE_MAGICK OFF CACHE BOOL "Enable BMP, GIF, PNG, TGA, TIFF image codecs.")
IF (USE_IMAGE_MAGICK)
  FIND_PACKAGE(ImageMagick COMPONENTS Magick++ REQUIRED)
  IF (IMAGEMAGICK_FOUND)
    ADD_DEFINITIONS(-DUSE_IMAGEMAGICK)
    INCLUDE_DIRECTORIES(${ImageMagick_Magick++_INCLUDE_DIR})
    SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${ImageMagick_Magick++_LIBRARY})
  ENDIF (IMAGEMAGICK_FOUND)
ENDIF (USE_IMAGE_MAGICK)

SET(USE_LIBJPEG ON CACHE BOOL "Enable JPEG image codec.")
IF (USE_LIBJPEG)
  FIND_PACKAGE(JPEG REQUIRED)
  IF (JPEG_FOUND)
    ADD_DEFINITIONS(-DUSE_LIBJPEG)
    INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
    SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${JPEG_LIBRARIES})
  ENDIF (JPEG_FOUND)
ENDIF (USE_LIBJPEG)

SET(USE_OPENEXR OFF CACHE BOOL "Enable OpenEXR image codec.")
IF (USE_OPENEXR)
  FIND_PACKAGE(OpenEXR REQUIRED)
  IF (OPENEXR_FOUND)
    ADD_DEFINITIONS(-DUSE_OPENEXR)
    INCLUDE_DIRECTORIES(${OPENEXR_INCLUDE_PATHS})
    SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${OPENEXR_LIBRARIES})
  ENDIF (OPENEXR_FOUND)
ENDIF (USE_OPENEXR)

ADD_LIBRARY(image STATIC
  exr.cpp
  image.cpp
  jpeg.cpp
  magick.cpp
  pfm.cpp
  ppm.cpp
  tga.cpp
)

TARGET_LINK_LIBRARIES(image sys ${ADDITIONAL_LIBRARIES})

