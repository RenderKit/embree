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
    set(TBB_DIR ${EMBREE_TBB_ROOT}/lib/cmake/tbb)
  endif()

  ADD_LIBRARY(tasking STATIC taskschedulertbb.cpp)
  
  if (TARGET TBB::tbb)
    message("reuse existing TBB::tbb target")
    TARGET_LINK_LIBRARIES(tasking PUBLIC TBB::tbb)
  else()
    # Try getting TBB via config first
    find_package(TBB 2021.1 QUIET COMPONENTS tbb CONFIG)
    if (TBB_FOUND)
      message("found tbb via config")
      TARGET_LINK_LIBRARIES(tasking PUBLIC TBB::tbb)
    else()
      message("not found tbb via config")
      # If not found try getting older TBB via module (FindTBB.cmake)
      unset(TBB_DIR CACHE)
      find_package(TBB 4.1 REQUIRED tbb)
      if (TBB_FOUND)
        message("found tbb via FindTBB.cmake")
        TARGET_LINK_LIBRARIES(tasking PUBLIC TBB)
        TARGET_INCLUDE_DIRECTORIES(tasking PUBLIC "${TBB_INCLUDE_DIRS}")
      else()
        message("not found tbb via FindTBB.cmake")
      endif()
    endif()
    if (TBB_FOUND)
      message("found tbb")
    else()
      message("not found tbb")
    endif()
  endif()

  ###############################################################
  ## Create tasking target and link against TBB. Also set include directory
  ## information on tasking target to provide the "algorithms" object library
  ## with the TBB header files
  ###############################################################
  #TARGET_LINK_LIBRARIES(tasking PUBLIC TBB)
  #TARGET_INCLUDE_DIRECTORIES(tasking PUBLIC "${TBB_INCLUDE_DIRS}")

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
