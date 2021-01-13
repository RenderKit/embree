## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (TASKING_INTERNAL)
  ADD_LIBRARY(tasking STATIC taskschedulerinternal.cpp)
ELSEIF (TASKING_TBB)
  ##############################################################
  # Find TBB
  ##############################################################
  if (NOT ${EMBREE_TBB_ROOT} STREQUAL "")
    set(TBB_ROOT ${EMBREE_TBB_ROOT})
  endif()
  FIND_PACKAGE(TBB REQUIRED tbb)

  ##############################################################
  # Create tasking target and link against TBB. Also set include directory
  # information on tasking target to provide the "algorithms" object library
  # with the TBB header files
  ##############################################################
  ADD_LIBRARY(tasking STATIC taskschedulertbb.cpp)
  TARGET_LINK_LIBRARIES(tasking PUBLIC TBB)
  TARGET_INCLUDE_DIRECTORIES(tasking PUBLIC "${TBB_INCLUDE_DIRS}")

  include(installTBB)

ELSEIF (TASKING_PPL)
  ADD_LIBRARY(tasking STATIC taskschedulerppl.cpp)
  TARGET_LINK_LIBRARIES(tasking PUBLIC ${PPL_LIBRARIES})
ENDIF()

SET_PROPERTY(TARGET tasking PROPERTY FOLDER common)
SET_PROPERTY(TARGET tasking APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_STATIC_LIB)
  INSTALL(TARGETS tasking EXPORT tasking-targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT devel)
  INSTALL(EXPORT tasking-targets DESTINATION ${EMBREE_CMAKEEXPORT_DIR} COMPONENT devel)
ENDIF()
