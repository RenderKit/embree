## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(rthwif STATIC rthwif_embree_builder.cpp builder/qbvh6.cpp builder/statistics.cpp)
TARGET_LINK_LIBRARIES(rthwif PRIVATE tasking)

#ADD_LIBRARY(rthwif_sycl STATIC rthwif_embree.cpp rthwif_production.cpp)
#SET_TARGET_PROPERTIES(rthwif_sycl PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_SYCL})

IF (EMBREE_STATIC_LIB)
  INSTALL(TARGETS rthwif EXPORT rthwif-targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT devel)
  INSTALL(EXPORT rthwif-targets DESTINATION ${EMBREE_CMAKEEXPORT_DIR} COMPONENT devel)

#  INSTALL(TARGETS rthwif_sycl EXPORT rthwif_sycl-targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT devel)
#  INSTALL(EXPORT rthwif_sycl-targets DESTINATION ${EMBREE_CMAKEEXPORT_DIR} COMPONENT devel)
ENDIF()
