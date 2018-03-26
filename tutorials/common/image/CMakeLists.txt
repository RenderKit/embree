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

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})
INCLUDE(CMakeDependentOption)

SET(ADDITIONAL_SOURCES)

FIND_PACKAGE(OpenImageIO)
MARK_AS_ADVANCED(
  OPENIMAGEIO_ROOT
)
CMAKE_DEPENDENT_OPTION(EMBREE_TUTORIALS_OPENIMAGEIO "Enables BMP, GIF, PNG, TGA, TIFF image codecs." OFF "OPENIMAGEIO_FOUND" OFF)
IF (EMBREE_TUTORIALS_OPENIMAGEIO)
  ADD_DEFINITIONS(-DUSE_OPENIMAGEIO)
  INCLUDE_DIRECTORIES(${OPENIMAGEIO_INCLUDE_DIRS})
  SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${OPENIMAGEIO_LIBRARIES})
  SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} oiio.cpp)
ENDIF (EMBREE_TUTORIALS_OPENIMAGEIO)

FIND_PACKAGE(JPEG)
CMAKE_DEPENDENT_OPTION(EMBREE_TUTORIALS_LIBJPEG "Enables JPEG image codec." ON "JPEG_FOUND" OFF)
IF (EMBREE_TUTORIALS_LIBJPEG)
  ADD_DEFINITIONS(-DEMBREE_TUTORIALS_LIBJPEG)
  INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
  SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${JPEG_LIBRARIES})
  SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} jpeg.cpp)
ENDIF (EMBREE_TUTORIALS_LIBJPEG)

FIND_PACKAGE(PNG)
CMAKE_DEPENDENT_OPTION(EMBREE_TUTORIALS_LIBPNG "Enables PNG image codecs." ON "PNG_FOUND" OFF)
IF (EMBREE_TUTORIALS_LIBPNG)
  ADD_DEFINITIONS(-DEMBREE_TUTORIALS_LIBPNG)
  INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIR})
  SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${PNG_LIBRARIES})
ENDIF (EMBREE_TUTORIALS_LIBPNG)

ADD_LIBRARY(image STATIC
    image.cpp
    pfm.cpp
    ppm.cpp
    tga.cpp
    png.cpp
    ${ADDITIONAL_SOURCES}
  )

TARGET_LINK_LIBRARIES(image sys math ${ADDITIONAL_LIBRARIES})
SET_PROPERTY(TARGET image PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET image APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
