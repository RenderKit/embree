## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0


set(EMBREE_LEVEL_ZERO_API_HEADER_DIR "" CACHE PATH "directory pointing to the level zero api header (ze_api.h). If not specified (default) Embree will download ze_api.h automatically." )
SET(EMBREE_ZE_LOADER_RUNTIME_LINK_NAME_LINUX "libze_loader.so.1" CACHE STRING "Name of the ze_loader lib that is looked for at runtime on Linux")
SET(EMBREE_ZE_LOADER_RUNTIME_LINK_NAME_WINDOWS "ze_loader.dll" CACHE STRING "Name of the ze_loader lib that is looked for at runtime on Windows")

MARK_AS_ADVANCED(EMBREE_LEVEL_ZERO_API_HEADER_DIR)
MARK_AS_ADVANCED(EMBREE_ZE_LOADER_RUNTIME_LINK_NAME_LINUX)
MARK_AS_ADVANCED(EMBREE_ZE_LOADER_RUNTIME_LINK_NAME_WINDOWS)

if (EMBREE_LEVEL_ZERO_API_HEADER_DIR)
  MESSAGE("-- Use level zero API header from ${EMBREE_LEVEL_ZERO_API_HEADER_DIR}")
else()
  MESSAGE("-- Download level zero API header")
  set(EMBREE_LEVEL_ZERO_API_HEADER_DIR ${CMAKE_BINARY_DIR}/_deps/level_zero)
  file(DOWNLOAD
    https://github.com/oneapi-src/level-zero/raw/v1.10/include/ze_api.h
    ${EMBREE_LEVEL_ZERO_API_HEADER_DIR}/ze_api.h)
endif()

ADD_LIBRARY(ze_wrapper STATIC ze_wrapper.cpp)
TARGET_INCLUDE_DIRECTORIES(ze_wrapper INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)
TARGET_INCLUDE_DIRECTORIES(ze_wrapper PUBLIC ${EMBREE_LEVEL_ZERO_API_HEADER_DIR})
TARGET_COMPILE_DEFINITIONS(ze_wrapper PRIVATE ZE_LOADER_NAME_LINUX="${EMBREE_ZE_LOADER_RUNTIME_LINK_NAME_LINUX}")
TARGET_COMPILE_DEFINITIONS(ze_wrapper PRIVATE ZE_LOADER_NAME_WINDOWS="${EMBREE_ZE_LOADER_RUNTIME_LINK_NAME_WINDOWS}")
TARGET_LINK_LIBRARIES(ze_wrapper PUBLIC embree_rthwif)
SET_PROPERTY(TARGET ze_wrapper PROPERTY FOLDER common)
SET_PROPERTY(TARGET ze_wrapper APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_STATIC_LIB)
  INSTALL(TARGETS ze_wrapper EXPORT ze_wrapper-targets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)
  INSTALL(EXPORT ze_wrapper-targets DESTINATION "${EMBREE_CMAKEEXPORT_DIR}" COMPONENT devel)
ENDIF()
