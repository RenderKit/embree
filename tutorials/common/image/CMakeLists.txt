## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})
INCLUDE(CMakeDependentOption)

SET(ADDITIONAL_SOURCES)

ADD_LIBRARY(image STATIC
    image.cpp
    pfm.cpp
    ppm.cpp
    tga.cpp
    stb.cpp
    exr.cpp
    ${ADDITIONAL_SOURCES}
  )

TARGET_LINK_LIBRARIES(image sys math ${ADDITIONAL_LIBRARIES})
SET_PROPERTY(TARGET image PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET image APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
