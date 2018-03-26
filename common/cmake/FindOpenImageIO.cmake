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

IF (NOT OPENIMAGEIO_ROOT)
  SET(OPENIMAGEIO_ROOT $ENV{OPENIMAGEIO_ROOT})
ENDIF()
IF (NOT OPENIMAGEIO_ROOT)
  SET(OPENIMAGEIO_ROOT $ENV{OPENIMAGEIOROOT})
ENDIF()

# detect changed OPENIMAGEIO_ROOT
IF (NOT OPENIMAGEIO_ROOT STREQUAL OPENIMAGEIO_ROOT_LAST)
  UNSET(OPENIMAGEIO_INCLUDE_DIR CACHE)
  UNSET(OPENIMAGEIO_LIBRARY CACHE)
ENDIF()

set(OPENIMAGEIO_LIB_SUFFIX "")
IF (WIN32)
  IF (MSVC14)
    SET(OPENIMAGEIO_LIB_SUFFIX "vc2015")
  ELSEIF (MSVC12)
    SET(OPENIMAGEIO_LIB_SUFFIX "vc2013")
  ELSEIF (MSVC11)
    SET(OPENIMAGEIO_LIB_SUFFIX "vc2012")
  ELSEIF (MINGW)
    IF (X64)
      SET(OPENIMAGEIO_LIB_SUFFIX "mingw-w64")
    # Who's ever going to build for 32bit??
    ELSE ()
      SET(OPENIMAGEIO_LIB_SUFFIX "mingw-w64")
    ENDIF()
  ENDIF()
ENDIF ()

FIND_PATH(OPENIMAGEIO_ROOT include/OpenImageIO/imageio.h
  DOC "Root of OpenImageIO installation"
  HINTS ${OPENIMAGEIO_ROOT}
  PATHS
    ${PROJECT_SOURCE_DIR}/oiio
    /usr/local
    /usr
    /
)

FIND_PATH(OPENIMAGEIO_INCLUDE_DIR OpenImageIO/imageio.h PATHS ${OPENIMAGEIO_ROOT}/include NO_DEFAULT_PATH)
SET(OPENIMAGEIO_HINTS
  HINTS
    ${OPENIMAGEIO_ROOT}
  PATH_SUFFIXES
    /lib
    /lib64
    /lib-${OPENIMAGEIO_LIB_SUFFIX}
  )
SET(OPENIMAGEIO_PATHS PATHS /usr/lib /usr/lib64 /lib /lib64)
FIND_LIBRARY(OPENIMAGEIO_LIBRARY OpenImageIO ${OPENIMAGEIO_HINTS} ${OPENIMAGEIO_PATHS})

SET(OPENIMAGEIO_ROOT_LAST ${OPENIMAGEIO_ROOT} CACHE INTERNAL "Last value of OPENIMAGEIO_ROOT to detect changes")

SET(OPENIMAGEIO_ERROR_MESSAGE "OpenImageIO not found in your environment. You can 1) install
                              via your OS package manager, or 2) install it
                              somewhere on your machine and point OPENIMAGEIO_ROOT to it.")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenImageIO
  ${OPENIMAGEIO_ERROR_MESSAGE}
  OPENIMAGEIO_INCLUDE_DIR OPENIMAGEIO_LIBRARY
)

IF (OPENIMAGEIO_FOUND)
  SET(OPENIMAGEIO_INCLUDE_DIRS ${OPENIMAGEIO_INCLUDE_DIR})
  SET(OPENIMAGEIO_LIBRARIES ${OPENIMAGEIO_LIBRARY})
ENDIF()

MARK_AS_ADVANCED(OPENIMAGEIO_INCLUDE_DIR)
MARK_AS_ADVANCED(OPENIMAGEIO_LIBRARY)
