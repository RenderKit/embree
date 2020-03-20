## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0


macro(find_tbb TARGET
               ROOT_VAR 
               INCLUDE_DIR_VAR
               BIN_DIR_VAR
               LIB_DIR_VAR
               TBB_LIBRARY_VAR
               TBBMALLOC_LIBRARY_VAR
               TBB_LIBRARY_NAME
               TBBMALLOC_LIBRARY_NAME)

  if(${ROOT_VAR})
    message(STATUS "Looking for ${TARGET} (${TBB_LIBRARY_NAME}, ${TBBMALLOC_LIBRARY_NAME})"
                   " in ${${ROOT_VAR}}")
  else()
    message(STATUS "Looking for ${TARGET} (${TBB_LIBRARY_NAME}, ${TBBMALLOC_LIBRARY_NAME})"
                   " in standard locations")
  endif()

  if(WIN32)
    # workaround for parentheses in variable name / CMP0053
    set(PROGRAMFILESx86 "PROGRAMFILES(x86)")
    set(PROGRAMFILES32 "$ENV{${PROGRAMFILESx86}}")
    if(NOT PROGRAMFILES32)
      set(PROGRAMFILES32 "$ENV{PROGRAMFILES}")
    endif()
    if(NOT PROGRAMFILES32)
      set(PROGRAMFILES32 "C:/Program Files (x86)")
    endif()
    find_path(${ROOT_VAR} include/tbb/tbb.h
      DOC "Root of ${TBB_LIBRARY_NAME} installation"
      PATHS ${PROJECT_SOURCE_DIR}/tbb
      NO_DEFAULT_PATH
    )
    find_path(${ROOT_VAR} include/tbb/tbb.h
      DOC "Root of ${TBB_LIBRARY_NAME} installation"
      PATHS
        ${PROJECT_SOURCE_DIR}/../tbb
        "${PROGRAMFILES32}/IntelSWTools/compilers_and_libraries/windows/tbb"
        "${PROGRAMFILES32}/Intel/Composer XE/tbb"
        "${PROGRAMFILES32}/Intel/compilers_and_libraries/windows/tbb"
    )

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(TBB_ARCH intel64)
    else()
      set(TBB_ARCH ia32)
    endif()

    if(MSVC10)
      set(TBB_VCVER vc10)
    elseif(MSVC11)
      set(TBB_VCVER vc11)
    elseif(MSVC12)
      set(TBB_VCVER vc12)
    else()
      set(TBB_VCVER vc14)
    endif()

    if(${ROOT_VAR} STREQUAL "")
      find_path(${INCLUDE_DIR_VAR} tbb/task_scheduler_init.h)
      find_path(${BIN_DIR_VAR} "${TBB_LIBRARY_NAME}.dll")
      find_library(${TBB_LIBRARY_VAR} ${TBB_LIBRARY_NAME})
      find_library(${TBBMALLOC_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_NAME})
    else()
      set(${INCLUDE_DIR_VAR} ${INCLUDE_DIR_VAR}-NOTFOUND)
      set(${BIN_DIR_VAR} ${BIN_DIR_VAR}-NOTFOUND)
      set(${TBB_LIBRARY_VAR} ${TBB_LIBRARY_VAR}-NOTFOUND)
      set(${TBBMALLOC_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_VAR}-NOTFOUND)

      find_path(${INCLUDE_DIR_VAR} tbb/task_scheduler_init.h PATHS ${${ROOT_VAR}}/include NO_DEFAULT_PATH)

      find_path(${BIN_DIR_VAR} "${TBB_LIBRARY_NAME}.dll"
        HINTS
          ${${ROOT_VAR}}/bin/${TBB_ARCH}/${TBB_VCVER}
          ${${ROOT_VAR}}/bin
          ${${ROOT_VAR}}/../redist/${TBB_ARCH}/tbb/${TBB_VCVER}
          ${${ROOT_VAR}}/../redist/${TBB_ARCH}_win/tbb/${TBB_VCVER}
        NO_DEFAULT_PATH
      )

      set(${LIB_DIR_VAR} ${${ROOT_VAR}}/lib/${TBB_ARCH}/${TBB_VCVER})
      
      find_library(${TBB_LIBRARY_VAR} ${TBB_LIBRARY_NAME} 
        PATHS 
          ${${LIB_DIR_VAR}} 
          ${${ROOT_VAR}}/lib 
        NO_DEFAULT_PATH)

      find_library(${TBBMALLOC_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_NAME} 
        PATHS 
          ${${LIB_DIR_VAR}} 
          ${${ROOT_VAR}}/lib
        NO_DEFAULT_PATH)
    endif()

  else()

    find_path(${ROOT_VAR} include/tbb/tbb.h
      DOC "Root of ${TBB_LIBRARY_NAME} installation"
      PATHS ${PROJECT_SOURCE_DIR}/tbb
      NO_DEFAULT_PATH
    )
    find_path(${ROOT_VAR} include/tbb/tbb.h
      DOC "Root of ${TBB_LIBRARY_NAME} installation"
      HINTS ${${ROOT_VAR}}
      PATHS
        ${PROJECT_SOURCE_DIR}/tbb
        /opt/intel/composerxe/tbb
        /opt/intel/compilers_and_libraries/tbb
        /opt/intel/tbb
    )

    if(${ROOT_VAR} STREQUAL "")
      find_path(${INCLUDE_DIR_VAR} tbb/task_scheduler_init.h)
      find_library(${TBB_LIBRARY_VAR} ${TBB_LIBRARY_NAME})
      find_library(${TBBMALLOC_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_NAME})
    elseif(EXISTS ${${ROOT_VAR}}/cmake/TBBBuild.cmake AND EXISTS ${${ROOT_VAR}}/src/tbb/tbb_version.h)
      option(TBB_STATIC_LIB "Build TBB as a static library (building TBB as a static library is NOT recommended)")
      if(TBB_STATIC_LIB)
        include(${${ROOT_VAR}}/cmake/TBBBuild.cmake)
        tbb_build(TBB_ROOT ${${ROOT_VAR}} CONFIG_DIR TBB_DIR MAKE_ARGS extra_inc=big_iron.inc)

        set(${INCLUDE_DIR_VAR} ${${ROOT_VAR}}/include)
        set(${TBB_LIBRARY_VAR} ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/lib${TBB_LIBRARY_NAME}.a)
        set(${TBBMALLOC_LIBRARY_VAR} ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/lib${TBBMALLOC_LIBRARY_NAME}.a)
      else()
        include(${${ROOT_VAR}}/cmake/TBBBuild.cmake)
        tbb_build(TBB_ROOT ${${ROOT_VAR}} CONFIG_DIR TBB_DIR)
        set(${INCLUDE_DIR_VAR} ${${ROOT_VAR}}/include)
        set(${TBB_LIBRARY_VAR} ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/lib${TBB_LIBRARY_NAME}.so.2)
        set(${TBBMALLOC_LIBRARY_VAR} ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/lib${TBBMALLOC_LIBRARY_NAME}.so.2)
      endif()
    else()
      set(${INCLUDE_DIR_VAR} ${INCLUDE_DIR_VAR}-NOTFOUND)
      set(${BIN_DIR_VAR} ${BIN_DIR_VAR}-NOTFOUND)
      set(${TBB_LIBRARY_VAR} ${TBB_LIBRARY_VAR}-NOTFOUND)
      set(${TBBMALLOC_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_VAR}-NOTFOUND)
      if(APPLE)
        find_path(${INCLUDE_DIR_VAR} tbb/task_scheduler_init.h PATHS ${${ROOT_VAR}}/include NO_DEFAULT_PATH)
        find_library(${TBB_LIBRARY_VAR} ${TBB_LIBRARY_NAME} PATHS ${${ROOT_VAR}}/lib NO_DEFAULT_PATH)
        find_library(${TBBMALLOC_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_NAME} PATHS ${${ROOT_VAR}}/lib NO_DEFAULT_PATH)
      else()
        find_path(${INCLUDE_DIR_VAR} tbb/task_scheduler_init.h PATHS ${${ROOT_VAR}}/include NO_DEFAULT_PATH)
        file(GLOB ALL_HINTS PATHS ${${ROOT_VAR}}/lib/intel64/gcc*)
        list(REVERSE ALL_HINTS)
        set(ALL_HINTS 
          HINTS 
            ${ALL_HINTS}
            ${${ROOT_VAR}}/lib 
            ${${ROOT_VAR}}/lib/x86_64-linux-gnu
            ${${ROOT_VAR}}/lib64 
            ${${ROOT_VAR}}/libx86_64-linux-gnu
          NO_DEFAULT_PATH)
        find_library(${TBB_LIBRARY_VAR}       ${TBB_LIBRARY_NAME}       ${ALL_HINTS})
        find_library(${TBBMALLOC_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_NAME} ${ALL_HINTS})
      endif()
    endif()
  endif()

  include(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(${TARGET} 
    DEFAULT_MSG ${INCLUDE_DIR_VAR} ${TBB_LIBRARY_VAR} ${TBBMALLOC_LIBRARY_VAR})

  # Do not display these variables in GUIs unless the advanced flag is on.
  mark_as_advanced(${INCLUDE_DIR_VAR})
  mark_as_advanced(${BIN_DIR_VAR})
  mark_as_advanced(${LIB_DIR_VAR})
  mark_as_advanced(${TBB_LIBRARY_VAR})
  mark_as_advanced(${TBBMALLOC_LIBRARY_VAR})

  # Create an imported library target for TBB.
  if(${TARGET}_FOUND)
    add_library(${TARGET}::tbb SHARED IMPORTED)
    set_target_properties(${TARGET}::tbb PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${${INCLUDE_DIR_VAR}}
      INTERFACE_COMPILE_DEFINITIONS "__TBB_NO_IMPLICIT_LINKAGE=1"
    )
    add_library(${TARGET}::tbbmalloc SHARED IMPORTED)
    set_target_properties(${TARGET}::tbbmalloc PROPERTIES
      INTERFACE_COMPILE_DEFINITIONS "__TBBMALLOC_NO_IMPLICIT_LINKAGE=1"
    )
    if(WIN32)
      set_target_properties(${TARGET}::tbb PROPERTIES
        IMPORTED_IMPLIB ${${TBB_LIBRARY_VAR}}
      )
      set_target_properties(${TARGET}::tbbmalloc PROPERTIES
        IMPORTED_IMPLIB ${${TBBMALLOC_LIBRARY_VAR}}
      )
    else()
      # Note: IMPORTED_NO_SONAME must be set or cmake will attempt
      #       to link to the full path of libtbb.so. Instead, we
      #       rely on the linker to find libtbb.so.2.
      set_target_properties(${TARGET}::tbb PROPERTIES
        IMPORTED_LOCATION ${${TBB_LIBRARY_VAR}}
        IMPORTED_NO_SONAME TRUE
      )
      set_target_properties(${TARGET}::tbbmalloc PROPERTIES
        IMPORTED_LOCATION ${${TBBMALLOC_LIBRARY_VAR}}
        IMPORTED_NO_SONAME TRUE
      )
    endif()
    get_filename_component(${LIB_DIR_VAR} ${${TBB_LIBRARY_VAR}} DIRECTORY)
    message(STATUS "FOUND ${TARGET} AT ${${LIB_DIR_VAR}}")
  endif()
endmacro()

#===============================================================================
# User configuration.
#===============================================================================

SET(EMBREE_TBB_ROOT "" CACHE PATH "Sets the root path of the release version of TBB.")
SET(EMBREE_TBB_DEBUG_ROOT "" CACHE PATH "Sets the root path of the debug version of TBB.")
SET(EMBREE_TBB_POSTFIX "" CACHE STRING "Link to tbb<EMBREE_TBB_POSTFIX>.(dll,lib,so) in release mode.")
SET(EMBREE_TBB_DEBUG_POSTFIX "_debug" CACHE STRING "Link to tbb<EMBREE_TBB_DEBUG_POSTFIX>.(dll,lib,so) in debug mode.")

if (NOT EMBREE_TBB_ROOT)
  set(EMBREE_TBB_ROOT $ENV{EMBREE_TBB_ROOT})
  if(NOT EMBREE_TBB_ROOT)
    set(EMBREE_TBB_ROOT $ENV{EMBREE_TBBROOT})
  endif()
endif()

# For the debug root we fall back to TBB_ROOT!
if (NOT EMBREE_TBB_DEBUG_ROOT)
  set(EMBREE_TBB_DEBUG_ROOT $ENV{EMBREE_TBB_DEBUG_ROOT})
  if(NOT EMBREE_TBB_DEBUG_ROOT)
    set(EMBREE_TBB_DEBUG_ROOT $ENV{EMBREE_TBBDEBUGROOT})
    if(NOT EMBREE_TBB_DEBUG_ROOT)
      set(EMBREE_TBB_DEBUG_ROOT ${EMBREE_TBB_ROOT})
    endif()
  endif()
endif()

if (NOT EMBREE_TBB_LIBRARY_NAME)
  set(EMBREE_TBB_LIBRARY_NAME $ENV{EMBREE_TBB_LIBRARY_NAME})
  if (NOT EMBREE_TBB_LIBRARY_NAME)
    set(EMBREE_TBB_LIBRARY_NAME "tbb${EMBREE_TBB_POSTFIX}")
  endif()
endif()

if (NOT EMBREE_TBBMALLOC_LIBRARY_NAME)
  set(EMBREE_TBBMALLOC_LIBRARY_NAME $ENV{EMBREE_TBBMALLOC_LIBRARY_NAME})
  if (NOT EMBREE_TBBMALLOC_LIBRARY_NAME)
    set(EMBREE_TBBMALLOC_LIBRARY_NAME "tbbmalloc${EMBREE_TBB_POSTFIX}")
  endif()
endif()

if (NOT EMBREE_TBB_DEBUG_LIBRARY_NAME)
  set(EMBREE_TBB_DEBUG_LIBRARY_NAME $ENV{EMBREE_TBB_DEBUG_LIBRARY_NAME})
  if (NOT EMBREE_TBB_DEBUG_LIBRARY_NAME)
    set(EMBREE_TBB_DEBUG_LIBRARY_NAME "tbb${EMBREE_TBB_DEBUG_POSTFIX}")
  endif()
endif()

if (NOT EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME)
  set(EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME $ENV{EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME})
  if (NOT EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME)
    set(EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME "tbbmalloc${EMBREE_TBB_DEBUG_POSTFIX}")
  endif()
endif()

find_tbb(TBB
  EMBREE_TBB_ROOT 
  EMBREE_TBB_INCLUDE_DIR 
  EMBREE_TBB_BIN_DIR 
  EMBREE_TBB_LIB_DIR 
  EMBREE_TBB_LIBRARY 
  EMBREE_TBBMALLOC_LIBRARY 
  ${EMBREE_TBB_LIBRARY_NAME}
  ${EMBREE_TBBMALLOC_LIBRARY_NAME})

find_tbb(TBB_DEBUG
  EMBREE_TBB_DEBUG_ROOT 
  EMBREE_TBB_DEBUG_INCLUDE_DIR 
  EMBREE_TBB_DEBUG_BIN_DIR 
  EMBREE_TBB_DEBUG_LIB_DIR 
  EMBREE_TBB_DEBUG_LIBRARY 
  EMBREE_TBBMALLOC_DEBUG_LIBRARY 
  ${EMBREE_TBB_DEBUG_LIBRARY_NAME}
  ${EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME})

#===============================================================================
# For linking, attempt to differentiate between release and debug builds,
# but fall back if possible to avoid breaking existing build pipelines.
#===============================================================================

if(TBB_FOUND AND TBB_DEBUG_FOUND)
  set(TBB_LIBRARIES
    optimized TBB::tbbmalloc
    optimized TBB::tbb       
    debug     TBB_DEBUG::tbbmalloc
    debug     TBB_DEBUG::tbb)
  set(TBB_INCLUDE_DIRS ${EMBREE_TBB_INCLUDE_DIR} ${EMBREE_TBB_DEBUG_INCLUDE_DIR})
elseif(TBB_FOUND)
  # This is a status message only because there is no performance penalty in
  # falling back to the release version of TBB. This fallback is quite likely 
  # to happen because users may use tbb as provided by their OS distribution.
  message(STATUS "Could not find TBB_DEBUG. Falling back to TBB in Debug mode. Consider setting EMBREE_TBB_DEBUG_ROOT or EMBREE_TBB_DEBUG_POSTIFX.")
  set(TBB_LIBRARIES TBB::tbbmalloc TBB::tbb)
  set(TBB_INCLUDE_DIRS ${EMBREE_TBB_INCLUDE_DIR})
elseif(TBB_DEBUG_FOUND)
  # Warn in this case because we are falling back to debug mode!
  message(WARNING "Could not find TBB. Falling back to TBB_DEBUG in Release mode. Consider setting EMBREE_TBB_ROOT or EMBREE_TBB_POSTIFX.")
  set(TBB_LIBRARIES TBB_DEBUG::tbbmalloc TBB_DEBUG::tbb)
  set(TBB_INCLUDE_DIRS ${EMBREE_TBB_DEBUG_INCLUDE_DIR})
else()
  message(FATAL_ERROR "Could not find TBB or TBB_DEBUG. Consider setting EMBREE_TBB_ROOT and EMBREE_TBB_DEBUG_ROOT.")
endif()

##############################################################
# Install TBB
##############################################################

IF (EMBREE_INSTALL_DEPENDENCIES)
  IF (TBB_FOUND)
    IF (WIN32)
      INSTALL(PROGRAMS ${EMBREE_TBB_BIN_DIR}/${EMBREE_TBB_LIBRARY_NAME}.dll ${EMBREE_TBB_BIN_DIR}/${EMBREE_TBBMALLOC_LIBRARY_NAME}.dll DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
      INSTALL(PROGRAMS ${EMBREE_TBB_LIB_DIR}/${EMBREE_TBB_LIBRARY_NAME}.lib ${EMBREE_TBB_LIB_DIR}/${EMBREE_TBBMALLOC_LIBRARY_NAME}.lib DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
    ELSEIF (APPLE)
      INSTALL(PROGRAMS ${EMBREE_TBB_LIBRARY} ${EMBREE_TBBMALLOC_LIBRARY} DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
    ELSE()
      INSTALL(PROGRAMS ${EMBREE_TBB_LIBRARY}.2 ${EMBREE_TBBMALLOC_LIBRARY}.2 DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
    ENDIF()
  ENDIF()
  IF (TBB_DEBUG_FOUND)
    IF (WIN32)
      INSTALL(PROGRAMS ${EMBREE_TBB_BIN_DIR}/${EMBREE_TBB_DEBUG_LIBRARY_NAME}.dll ${EMBREE_TBB_DEBUG_BIN_DIR}/${EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME}.dll DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
      INSTALL(PROGRAMS ${EMBREE_TBB_LIB_DIR}/${EMBREE_TBB_DEBUG_LIBRARY_NAME}.lib ${EMBREE_TBB_DEBUG_LIB_DIR}/${EMBREE_TBBMALLOC_DEBUG_LIBRARY_NAME}.lib DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
    ELSEIF (APPLE)
      INSTALL(PROGRAMS ${EMBREE_TBB_DEBUG_LIBRARY} ${EMBREE_TBBMALLOC_DEBUG_LIBRARY} DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
    ELSE()
      INSTALL(PROGRAMS ${EMBREE_TBB_DEBUG_LIBRARY}.2 ${EMBREE_TBBMALLOC_DEBUG_LIBRARY}.2 DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
    ENDIF()
  ENDIF()
ENDIF()

