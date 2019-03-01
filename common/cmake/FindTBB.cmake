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

IF (NOT TBB_ROOT)
  SET(TBB_ROOT $ENV{TBB_ROOT})
ENDIF()
IF (NOT TBB_ROOT)
  SET(TBB_ROOT $ENV{TBBROOT})
ENDIF()

IF (WIN32)
  # workaround for parentheses in variable name / CMP0053
  SET(PROGRAMFILESx86 "PROGRAMFILES(x86)")
  SET(PROGRAMFILES32 "$ENV{${PROGRAMFILESx86}}")
  IF (NOT PROGRAMFILES32)
    SET(PROGRAMFILES32 "$ENV{PROGRAMFILES}")
  ENDIF()
  IF (NOT PROGRAMFILES32)
    SET(PROGRAMFILES32 "C:/Program Files (x86)")
  ENDIF()
  FIND_PATH(EMBREE_TBB_ROOT include/tbb/tbb.h
    DOC "Root of TBB installation"
    PATHS ${PROJECT_SOURCE_DIR}/tbb
    NO_DEFAULT_PATH
  )
  FIND_PATH(EMBREE_TBB_ROOT include/tbb/tbb.h
    HINTS ${TBB_ROOT}
    PATHS
      ${PROJECT_SOURCE_DIR}/../tbb
      "${PROGRAMFILES32}/IntelSWTools/compilers_and_libraries/windows/tbb"
      "${PROGRAMFILES32}/Intel/Composer XE/tbb"
      "${PROGRAMFILES32}/Intel/compilers_and_libraries/windows/tbb"
  )

  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(TBB_ARCH intel64)
  ELSE()
    SET(TBB_ARCH ia32)
  ENDIF()

  # precompiled TBB packages currently only support VC12 and VC14
  IF (MSVC12)
    SET(TBB_VCVER vc12)
  ELSEIF(MSVC14)
    SET(TBB_VCVER vc14)
  ENDIF()

  SET(TBB_LIBDIR ${EMBREE_TBB_ROOT}/lib/${TBB_ARCH}/${TBB_VCVER})
  SET(TBB_BINDIR ${EMBREE_TBB_ROOT}/bin/${TBB_ARCH}/${TBB_VCVER})

  IF (EMBREE_TBB_ROOT STREQUAL "")
    FIND_PATH(TBB_INCLUDE_DIR tbb/task_scheduler_init.h)
    FIND_LIBRARY(TBB_LIBRARY tbb)
    FIND_LIBRARY(TBB_LIBRARY_MALLOC tbbmalloc)
  ELSE()
    SET(TBB_INCLUDE_DIR TBB_INCLUDE_DIR-NOTFOUND)
    SET(TBB_LIBRARY TBB_LIBRARY-NOTFOUND)
    SET(TBB_LIBRARY_MALLOC TBB_LIBRARY_MALLOC-NOTFOUND)
    FIND_PATH(TBB_INCLUDE_DIR tbb/task_scheduler_init.h PATHS ${EMBREE_TBB_ROOT}/include NO_DEFAULT_PATH)
    FIND_LIBRARY(TBB_LIBRARY tbb PATHS ${TBB_LIBDIR} ${EMBREE_TBB_ROOT}/lib NO_DEFAULT_PATH)
    FIND_LIBRARY(TBB_LIBRARY_MALLOC tbbmalloc PATHS ${TBB_LIBDIR} ${EMBREE_TBB_ROOT}/lib NO_DEFAULT_PATH)
  ENDIF()

ELSE ()

  FIND_PATH(EMBREE_TBB_ROOT include/tbb/tbb.h
    DOC "Root of TBB installation"
    PATHS ${PROJECT_SOURCE_DIR}/tbb
    NO_DEFAULT_PATH
  )
  FIND_PATH(EMBREE_TBB_ROOT include/tbb/tbb.h
    DOC "Root of TBB installation"
    HINTS ${TBB_ROOT}
    PATHS
      ${PROJECT_SOURCE_DIR}/tbb
      /opt/intel/composerxe/tbb
      /opt/intel/compilers_and_libraries/tbb
      /opt/intel/tbb
  )

  IF (EMBREE_TBB_ROOT STREQUAL "")
    FIND_PATH(TBB_INCLUDE_DIR tbb/task_scheduler_init.h)
    FIND_LIBRARY(TBB_LIBRARY tbb)
    FIND_LIBRARY(TBB_LIBRARY_MALLOC tbbmalloc)
    
  ELSEIF (EXISTS ${EMBREE_TBB_ROOT}/cmake/TBBBuild.cmake AND EXISTS ${EMBREE_TBB_ROOT}/src/tbb/tbb_version.h)
    OPTION(EMBREE_TBB_STATIC_LIB "Build TBB as a static library (building TBB as a static library is NOT recommended)")
    if (EMBREE_TBB_STATIC_LIB)
      include(${EMBREE_TBB_ROOT}/cmake/TBBBuild.cmake)
      tbb_build(TBB_ROOT ${EMBREE_TBB_ROOT} CONFIG_DIR TBB_DIR MAKE_ARGS extra_inc=big_iron.inc)
      SET(TBB_INCLUDE_DIR ${EMBREE_TBB_ROOT}/include)
      SET(TBB_LIBRARY ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/libtbb.a)
      SET(TBB_LIBRARY_MALLOC ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/libtbbmalloc.a)
    else()
      include(${EMBREE_TBB_ROOT}/cmake/TBBBuild.cmake)
      tbb_build(TBB_ROOT ${EMBREE_TBB_ROOT} CONFIG_DIR TBB_DIR)
      SET(TBB_INCLUDE_DIR ${EMBREE_TBB_ROOT}/include)
      SET(TBB_LIBRARY ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/libtbb.so.2)
      SET(TBB_LIBRARY_MALLOC ${PROJECT_BINARY_DIR}/tbb_cmake_build/tbb_cmake_build_subdir_release/libtbbmalloc.so.2)
    endif()
    
  ELSE()
    SET(TBB_INCLUDE_DIR TBB_INCLUDE_DIR-NOTFOUND)
    SET(TBB_LIBRARY TBB_LIBRARY-NOTFOUND)
    SET(TBB_LIBRARY_MALLOC TBB_LIBRARY_MALLOC-NOTFOUND)
    IF (APPLE)
      FIND_PATH(TBB_INCLUDE_DIR tbb/task_scheduler_init.h PATHS ${EMBREE_TBB_ROOT}/include NO_DEFAULT_PATH)
      FIND_LIBRARY(TBB_LIBRARY tbb PATHS ${EMBREE_TBB_ROOT}/lib NO_DEFAULT_PATH)
      FIND_LIBRARY(TBB_LIBRARY_MALLOC tbbmalloc PATHS ${EMBREE_TBB_ROOT}/lib NO_DEFAULT_PATH)
    ELSE()
      FIND_PATH(TBB_INCLUDE_DIR tbb/task_scheduler_init.h PATHS ${EMBREE_TBB_ROOT}/include NO_DEFAULT_PATH)
      SET(TBB_HINTS HINTS
          ${EMBREE_TBB_ROOT}/lib/intel64/gcc4.7
          ${EMBREE_TBB_ROOT}/lib/intel64/gcc4.4
          ${EMBREE_TBB_ROOT}/lib/intel64/gcc4.1
          ${EMBREE_TBB_ROOT}/lib
          ${EMBREE_TBB_ROOT}/lib64
        PATHS /usr/libx86_64-linux-gnu/)
      FIND_LIBRARY(TBB_LIBRARY tbb ${TBB_HINTS})
      FIND_LIBRARY(TBB_LIBRARY_MALLOC tbbmalloc ${TBB_HINTS})
    ENDIF()
  ENDIF()

ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TBB DEFAULT_MSG TBB_INCLUDE_DIR TBB_LIBRARY TBB_LIBRARY_MALLOC)

IF (TBB_FOUND)
  SET(TBB_INCLUDE_DIRS ${TBB_INCLUDE_DIR})
  SET(TBB_LIBRARIES ${TBB_LIBRARY} ${TBB_LIBRARY_MALLOC})
ENDIF()

MARK_AS_ADVANCED(TBB_INCLUDE_DIR)
MARK_AS_ADVANCED(TBB_LIBRARY)
MARK_AS_ADVANCED(TBB_LIBRARY_MALLOC)

##############################################################
# Install TBB
##############################################################

IF (EMBREE_INSTALL_DEPENDENCIES)
  IF (WIN32)
    INSTALL(PROGRAMS ${TBB_BINDIR}/tbb.dll ${TBB_BINDIR}/tbbmalloc.dll DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
    INSTALL(PROGRAMS ${TBB_LIBDIR}/tbb.lib ${TBB_LIBDIR}/tbbmalloc.lib DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
  ELSEIF (APPLE)
    INSTALL(PROGRAMS ${EMBREE_TBB_ROOT}/lib/libtbb.dylib ${EMBREE_TBB_ROOT}/lib/libtbbmalloc.dylib DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
  ELSE()
    INSTALL(PROGRAMS ${EMBREE_TBB_ROOT}/lib/intel64/gcc4.4/libtbb.so.2 ${EMBREE_TBB_ROOT}/lib/intel64/gcc4.4/libtbbmalloc.so.2 DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT lib)
  ENDIF()
ENDIF()

