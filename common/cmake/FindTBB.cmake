## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

#===============================================================================
# This script will attempt to find TBB and set up a TBB target.
#
# The user may specify a version and lists of required and optional components:
#
# find_package(TBB 2017.0 EXACT REQUIRED
#              tbb tbbmalloc
#              OPTIONAL_COMPONENTS tbbmalloc_proxy
#              QUIET)
#
# If this target exists already, the script will attempt to re-use it, but fail
# if version or components do not match the user-specified requirements.
#
# If all the required component targets (e.g. TBB::tbb) exist, the script will
# attempt to create a target TBB and link existing component targets to it.
# It will fail if the component target version does not match the user-specified
# requirements.
#
# The user may specify the following variables to help the search process:
# - TBB_ROOT
# - TBB_INCLUDE_DIR
#
# After the script has run successfully, there is a target TBB, as well as
# component targets TBB::<COMPONENT>, e.g. TBB::tbbmalloc.
#
# The targets will attempt to link to release versions of TBB in release mode,
# and debug versions in debug mode.
#
# In addition to the targets, the script defines:
#
# TBB_FOUND
# TBB_INCLUDE_DIRS
#
#===============================================================================

# We use INTERFACE libraries, which are only supported in 3.x
cmake_minimum_required(VERSION 3.1)

# These two are used to automatically find the root and include directories.
set(_TBB_INCLUDE_SUBDIR "include")
set(_TBB_HEADER "tbb/tbb.h")

# Initialize cache variable; but use existing non-cache variable as the default,
# and fall back to the environment variable.
if (NOT TBB_ROOT)
  set(TBB_ROOT "$ENV{TBB_ROOT}")
endif()

set(TBB_ROOT "${TBB_ROOT}" CACHE PATH "The root path of TBB.")

#===============================================================================
# Error messages that respect the user's wishes about peace and quiet.
#===============================================================================

function(rk_tbb_status)
  if (NOT TBB_FIND_QUIETLY)
    message(STATUS "${ARGV}")
  endif()
endfunction()

function(rk_tbb_warning)
  if (NOT TBB_FIND_QUIETLY)
    message(WARNING "${ARGV}")
  endif()
endfunction()

macro(rk_tbb_error)
  if (TBB_FIND_REQUIRED)
    message(FATAL_ERROR "${ARGV}")
  else()
    rk_tbb_warning("${ARGV}")
  endif()
  return()
endmacro()

#===============================================================================
# Extract a list of required and optional components.
#===============================================================================

macro(rk_tbb_list_components)
  # cmake provides the TBB_FIND_COMPONENTS and
  # TBB_FIND_REQUIRED_<C> variables based on the invocation
  # of find_package.
  if (TBB_FIND_COMPONENTS STREQUAL "")
    set(_REQUIRED_COMPONENTS "tbb")
    set(_OPTIONAL_COMPONENTS "tbbmalloc"
                             "tbbmalloc_proxy"
                             "tbbbind"
                             "tbbpreview")
  else()
    set(_REQUIRED_COMPONENTS "")
    set(_OPTIONAL_COMPONENTS "")
    foreach (C IN LISTS TBB_FIND_COMPONENTS)
      if (${TBB_FIND_REQUIRED_${C}})
        list(APPEND _REQUIRED_COMPONENTS ${C})
      else()
        list(APPEND _OPTIONAL_COMPONENTS ${C})
      endif()
    endforeach()
  endif()

  rk_tbb_status("Looking for TBB components ${_REQUIRED_COMPONENTS}"
                " (${_OPTIONAL_COMPONENTS})")
endmacro()

#===============================================================================
# List components that are available, and check if any REQUIRED components
# are missing.
#===============================================================================

macro(rk_tbb_check_components)
  set(_TBB_MISSING_COMPONENTS "")
  set(_TBB_AVAILABLE_COMPONENTS "")

  foreach (C IN LISTS _REQUIRED_COMPONENTS)
    if (TARGET TBB::${C})
      list(APPEND _TBB_AVAILABLE_COMPONENTS ${C})
    else()
      list(APPEND _TBB_MISSING_COMPONENTS ${C})
    endif()
  endforeach()

  foreach (C IN LISTS _OPTIONAL_COMPONENTS)
    if (TARGET TBB::${C})
      list(APPEND _TBB_AVAILABLE_COMPONENTS ${C})
    endif()
  endforeach()
endmacro()

#===============================================================================
# Check the version of the TBB root we found.
#===============================================================================

macro(rk_tbb_check_version)
  # Extract the version we found in our root.
  if(EXISTS "${TBB_INCLUDE_DIR}/oneapi/tbb/version.h")
    set(_TBB_VERSION_HEADER "oneapi/tbb/version.h")
  elseif(EXISTS "${TBB_INCLUDE_DIR}/tbb/tbb_stddef.h")
    set(_TBB_VERSION_HEADER "tbb/tbb_stddef.h")
  elseif(EXISTS "${TBB_INCLUDE_DIR}/tbb/version.h")
    set(_TBB_VERSION_HEADER "tbb/version.h")
  else()
    rk_tbb_error("Missing TBB version information. Could not find"
      "tbb/tbb_stddef.h or tbb/version.h in ${TBB_INCLUDE_DIR}")
  endif()
  file(READ ${TBB_INCLUDE_DIR}/${_TBB_VERSION_HEADER} VERSION_HEADER_CONTENT)
  string(REGEX MATCH "#define TBB_VERSION_MAJOR ([0-9]+)" DUMMY "${VERSION_HEADER_CONTENT}")
  set(TBB_VERSION_MAJOR ${CMAKE_MATCH_1})
  string(REGEX MATCH "#define TBB_VERSION_MINOR ([0-9]+)" DUMMY "${VERSION_HEADER_CONTENT}")
  set(TBB_VERSION_MINOR ${CMAKE_MATCH_1})
  set(TBB_VERSION "${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR}")
  set(TBB_VERSION_STRING "${TBB_VERSION}")

  # If the user provided information about required versions, check them!
  if (TBB_FIND_VERSION)
    if (${TBB_FIND_VERSION_EXACT} AND NOT
        TBB_VERSION VERSION_EQUAL ${TBB_FIND_VERSION})
      rk_tbb_error("Requested exact TBB version ${TBB_FIND_VERSION},"
        " but found ${TBB_VERSION}")
    elseif(TBB_VERSION VERSION_LESS ${TBB_FIND_VERSION})
      rk_tbb_error("Requested minimum TBB version ${TBB_FIND_VERSION},"
        " but found ${TBB_VERSION}")
    endif()
  endif()

  rk_tbb_status("Found TBB version ${TBB_VERSION} at ${TBB_ROOT}")
endmacro()

#===============================================================================
# Reuse existing targets.
# NOTE: This must be a macro, as we rely on return() to exit this script.
#===============================================================================

macro(rk_tbb_reuse_existing_target_components)
  rk_tbb_check_components()

  if (_TBB_MISSING_COMPONENTS STREQUAL "")
    rk_tbb_status("Found existing TBB component targets: ${_TBB_AVAILABLE_COMPONENTS}")

    # Get TBB_INCLUDE_DIR if not already set to check for the version of the
    # existing component targets (making the assumption that they all have
    # the same version)
    if (NOT TBB_INCLUDE_DIR)
      list(GET _TBB_AVAILABLE_COMPONENTS 0 first_target)
      get_target_property(TBB_INCLUDE_DIR TBB::${first_target} INTERFACE_INCLUDE_DIRECTORIES)
      foreach(TGT IN LISTS _TBB_AVAILABLE_COMPONENTS)
        get_target_property(_TGT_INCLUDE_DIR TBB::${TGT} INTERFACE_INCLUDE_DIRECTORIES)
        if (NOT _TGT_INCLUDE_DIR STREQUAL "${TBB_INCLUDE_DIR}")
          rk_tbb_error("Existing TBB component targets have inconsistent include directories.")
        endif()
      endforeach()
    endif()

    find_path(TBB_INCLUDE_DIR
      NAMES "${_TBB_HEADER}"
      PATHS "${TBB_INCLUDE_DIRS}")

    # Extract TBB_ROOT from the include path so that rk_tbb_check_version
    # prints the correct tbb location
    string(REPLACE "/${_TBB_INCLUDE_SUBDIR}" "" TBB_ROOT "${TBB_INCLUDE_DIR}")
    rk_tbb_check_version()

    # Add target TBB and link all available components
    if (NOT TARGET TBB)
      add_library(TBB INTERFACE)
      foreach(C IN LISTS _TBB_AVAILABLE_COMPONENTS)
        target_link_libraries(TBB INTERFACE TBB::${C})
      endforeach()
    endif()
    set(TBB_FOUND TRUE)
    set(TBB_INCLUDE_DIRS "${TBB_INCLUDE_DIR}")
    return()
  elseif ((TARGET TBB) OR (NOT _TBB_AVAILABLE_COMPONENTS STREQUAL ""))
    rk_tbb_error("Ignoring existing TBB targets because required components are missing: ${_TBB_MISSING_COMPONENTS}")
  endif()
endmacro()


#===============================================================================
# Find the root directory if a manual override is not specified.
# Sets TBB_ROOT in the parent scope, but does not check for failure.
#===============================================================================

function(rk_tbb_find_root)
  if (NOT TBB_ROOT OR TBB_ROOT STREQUAL "")
    set(TBB_HINTS "")
    set(TBB_PATHS "")

    if (WIN32)
      # workaround for parentheses in variable name / CMP0053
      set(PROGRAMFILESx86 "PROGRAMFILES(x86)")
      set(PROGRAMFILES32 "$ENV{${PROGRAMFILESx86}}")
      if(NOT PROGRAMFILES32)
        set(PROGRAMFILES32 "$ENV{PROGRAMFILES}")
      endif()
      if(NOT PROGRAMFILES32)
        set(PROGRAMFILES32 "C:/Program Files (x86)")
      endif()
      set(TBB_PATHS
          "${PROJECT_SOURCE_DIR}/../tbb"
          "${PROGRAMFILES32}/IntelSWTools/compilers_and_libraries/windows/tbb"
          "${PROGRAMFILES32}/Intel/Composer XE/tbb"
          "${PROGRAMFILES32}/Intel/compilers_and_libraries/windows/tbb")
    else()
      set(TBB_HINTS "/usr/local")
      set(TBB_PATHS
          "${PROJECT_SOURCE_DIR}/tbb"
          "/opt/intel/composerxe/tbb"
          "/opt/intel/compilers_and_libraries/tbb"
          "/opt/intel/compilers_and_libraries/linux/tbb"
          "/opt/intel/tbb")
    endif()

    set(TBB_ROOT "TBB_ROOT-NOTFOUND")
    find_path(TBB_ROOT
      NAMES "${_TBB_INCLUDE_SUBDIR}/${_TBB_HEADER}"
      HINTS ${TBB_HINTS}
      PATHS ${TBB_PATHS}
      NO_PACKAGE_ROOT_PATH)
  endif()
endfunction()

#===============================================================================
# Find the include directory if a manual override is not specified.
# Assumes TBB_ROOT to be set.
#===============================================================================

function(rk_tbb_find_include_directory)
  find_path(TBB_INCLUDE_DIR
    NAMES "${_TBB_HEADER}"
    HINTS "${TBB_ROOT}/${_TBB_INCLUDE_SUBDIR}"
    NO_PACKAGE_ROOT_PATH)
endfunction()

#===============================================================================
# Find a specific library and create a target for it.
#===============================================================================

function(rk_tbb_find_library COMPONENT_NAME BUILD_CONFIG)
  set(LIB_VAR "${COMPONENT_NAME}_LIBRARY_${BUILD_CONFIG}")
  set(BIN_DIR_VAR "${COMPONENT_NAME}_BIN_DIR_${BUILD_CONFIG}")
  set(DLL_VAR "${COMPONENT_NAME}_DLL_${BUILD_CONFIG}")
  if (BUILD_CONFIG STREQUAL "DEBUG")
    set(LIB_NAME "${COMPONENT_NAME}_debug")
  else()
    set(LIB_NAME "${COMPONENT_NAME}")
  endif()

  unset(LIB_PATHS)

  if (WIN32)
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

    set(LIB_PATHS
      ${TBB_ROOT}/lib/${TBB_ARCH}/${TBB_VCVER}
      ${TBB_ROOT}/lib
    )

    # On window, also search the DLL so that the client may install it.
    set(DLL_NAME "${LIB_NAME}.dll")

    # lib name with version suffix to handle oneTBB tbb12.dll
    set(LIB_NAME_VERSION "") 
    if (${COMPONENT_NAME} STREQUAL "tbb")
      if (BUILD_CONFIG STREQUAL "DEBUG")
        set(LIB_NAME_VERSION "tbb12_debug")
      else()
        set(LIB_NAME_VERSION "tbb12")
      endif()
    endif()
    set(DLL_NAME_VERSION "${LIB_NAME_VERSION}.dll")

    find_file(BIN_FILE
      NAMES ${DLL_NAME} ${DLL_NAME_VERSION}
      PATHS
        "${TBB_ROOT}/bin/${TBB_ARCH}/${TBB_VCVER}"
        "${TBB_ROOT}/bin"
        "${TBB_ROOT}/redist/${TBB_ARCH}/${TBB_VCVER}"
        "${TBB_ROOT}/../redist/${TBB_ARCH}/tbb/${TBB_VCVER}"
        "${TBB_ROOT}/../redist/${TBB_ARCH}_win/tbb/${TBB_VCVER}"
      NO_DEFAULT_PATH)
    get_filename_component(${BIN_DIR_VAR} ${BIN_FILE} DIRECTORY)
    set(${DLL_VAR} "${BIN_FILE}" CACHE PATH "${COMPONENT_NAME} ${BUILD_CONFIG} dll path")
  elseif(APPLE)
    set(LIB_PATHS ${TBB_ROOT}/lib)
  else()
    file(GLOB LIB_PATHS PATHS ${TBB_ROOT}/lib/intel64/gcc*)
    list(REVERSE LIB_PATHS)
    list(APPEND LIB_PATHS
      ${TBB_ROOT}/lib
      ${TBB_ROOT}/lib/x86_64-linux-gnu
      ${TBB_ROOT}/lib64
      ${TBB_ROOT}/libx86_64-linux-gnu)
  endif()

  # We prefer finding the versioned file on Unix so that the library path
  # variable will not point to a symlink. This makes installing TBB as a
  # dependency easier.
  if (UNIX)
    set(LIB_NAME lib${LIB_NAME}.so.2 ${LIB_NAME})
  endif()

  find_library(${LIB_VAR}
    NAMES ${LIB_NAME}
    PATHS ${LIB_PATHS}
    NO_DEFAULT_PATH)

  # Hide this variable if we found something, otherwise display it for
  # easy override.
  if(${LIB_VAR})
    mark_as_advanced(${LIB_VAR})
  endif()
  if(${BIN_DIR_VAR})
    mark_as_advanced(${BIN_DIR_VAR})
  endif()
  if(${DLL_VAR})
    mark_as_advanced(${DLL_VAR})
  endif()
endfunction()

#===============================================================================
# Find the given component.
# This macro attempts to find both release and debug versions, and falls back
# appropriately if only one can be found.
# On success, it creates a target ${TARGET}::${COMPONENT_NAME} and links
# it to the overall ${TARGET}.
#
# For more information on the variables set here, see
# https://cmake.org/cmake/help/v3.17/manual/cmake-developer.7.html#a-sample-find-module
#===============================================================================

function(rk_tbb_find_and_link_component COMPONENT_NAME)
  set(COMPONENT_TARGET "TBB::${COMPONENT_NAME}")

  rk_tbb_find_library("${COMPONENT_NAME}" RELEASE)
  rk_tbb_find_library("${COMPONENT_NAME}" DEBUG)

  if (${COMPONENT_NAME}_LIBRARY_RELEASE OR ${COMPONENT_NAME}_LIBRARY_DEBUG)
    # Note: We *must* use SHARED here rather than UNKNOWN as our
    #       IMPORTED_NO_SONAME trick a few lines down does not work with
    #       UNKNOWN.
    add_library(${COMPONENT_TARGET} SHARED IMPORTED)

    if (${COMPONENT_NAME}_LIBRARY_RELEASE)
      set_property(TARGET ${COMPONENT_TARGET} APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      if(WIN32)
        set_target_properties(${COMPONENT_TARGET} PROPERTIES
          IMPORTED_LOCATION_RELEASE "${${COMPONENT_NAME}_DLL_RELEASE}")
        set_target_properties(${COMPONENT_TARGET} PROPERTIES
          IMPORTED_IMPLIB_RELEASE "${${COMPONENT_NAME}_LIBRARY_RELEASE}")
      else()
        set_target_properties(${COMPONENT_TARGET} PROPERTIES
          IMPORTED_LOCATION_RELEASE "${${COMPONENT_NAME}_LIBRARY_RELEASE}")
      endif()
    endif()

    if (${COMPONENT_NAME}_LIBRARY_DEBUG)
      set_property(TARGET ${COMPONENT_TARGET} APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      if(WIN32)
        set_target_properties(${COMPONENT_TARGET} PROPERTIES
          IMPORTED_LOCATION_DEBUG "${${COMPONENT_NAME}_DLL_DEBUG}")
        set_target_properties(${COMPONENT_TARGET} PROPERTIES
          IMPORTED_IMPLIB_DEBUG "${${COMPONENT_NAME}_LIBRARY_DEBUG}")
      else()
        set_target_properties(${COMPONENT_TARGET} PROPERTIES
          IMPORTED_LOCATION_DEBUG "${${COMPONENT_NAME}_LIBRARY_DEBUG}")
      endif()
    endif()

    set_target_properties(${COMPONENT_TARGET} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${TBB_INCLUDE_DIR}"
      INTERFACE_COMPILE_DEFINITIONS "__TBB_NO_IMPLICIT_LINKAGE=1"
    )


    if(NOT WIN32)
      # Note: IMPORTED_NO_SONAME must be set or cmake will attempt
      #       to link to the full path of libtbb.so. Instead, we
      #       rely on the linker to find libtbb.so.2.
      set_target_properties(${COMPONENT_TARGET} PROPERTIES
        IMPORTED_NO_SONAME TRUE
      )
    endif()

    target_link_libraries(TBB INTERFACE ${COMPONENT_TARGET})
  endif()
endfunction()

#===============================================================================

# Note: The order of these is important.
#       Some of these macros create variables that are used in later calls.
rk_tbb_list_components()
rk_tbb_reuse_existing_target_components()

rk_tbb_find_root()
if (NOT EXISTS "${TBB_ROOT}")
  rk_tbb_error("Unable to find root directory ${TBB_ROOT}")
endif()
mark_as_advanced(TBB_ROOT) # Hide, we found something.

rk_tbb_find_include_directory()
if (NOT EXISTS "${TBB_INCLUDE_DIR}")
  rk_tbb_error("Unable to find include directory ${TBB_INCLUDE_DIR}")
endif()
mark_as_advanced(TBB_INCLUDE_DIR) # Hide, we found something.

rk_tbb_check_version()

add_library(TBB INTERFACE)

foreach(C IN LISTS _REQUIRED_COMPONENTS _OPTIONAL_COMPONENTS)
  rk_tbb_find_and_link_component(${C})
endforeach()

rk_tbb_check_components()
if (_TBB_MISSING_COMPONENTS)
  rk_tbb_error("Cannot find required components: "
               "${_TBB_MISSING_COMPONENTS}")
endif()

set(TBB_FOUND TRUE)
set(TBB_INCLUDE_DIRS "${TBB_INCLUDE_DIR}")
