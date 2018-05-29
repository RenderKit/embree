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

SET(FLAGS_SSE2  "/D__SSE__ /D__SSE2__")
SET(FLAGS_SSE42 "${FLAGS_SSE2} /D__SSE3__ /D__SSSE3__ /D__SSE4_1__ /D__SSE4_2__")
SET(FLAGS_AVX   "${FLAGS_SSE42} /arch:AVX")
SET(FLAGS_AVX2  "${FLAGS_SSE42} /arch:AVX2")

SET(COMMON_CXX_FLAGS "")
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /EHsc")        # catch C++ exceptions only and extern "C" functions never throw a C++ exception
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /MP")          # compile source files in parallel
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GR")          # enable runtime type information (on by default)
IF (EMBREE_STACK_PROTECTOR)
  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GS")          # protects against return address overrides
ELSE()
  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GS-")          # do not protect against return address overrides
ENDIF()
MACRO(DISABLE_STACK_PROTECTOR_FOR_FILE file)
  IF (EMBREE_STACK_PROTECTOR)
    SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "/GS-")
  ENDIF()
ENDMACRO()

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${COMMON_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DDEBUG")                     # enables assertions
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DTBB_USE_DEBUG")             # configures TBB in debug mode
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Oi")                         # inline intrinsic functions
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG")        # generate debug information
SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG")  # generate debug information

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${COMMON_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")                       # enable full optimizations
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi")                       # inline intrinsic functions
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Gy")                       # package individual functions

SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${COMMON_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /DTBB_USE_DEBUG")          # configures TBB in debug mode
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Ox")                      # enable full optimizations
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Oi")                      # inline intrinsic functions
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Gy")                      # package individual functions
SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG")        # generate debug information
SET(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG")  # generate debug information

SET(SECURE_LINKER_FLAGS "")
SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /NXCompat")    # compatible with data execution prevention (on by default)
SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /DynamicBase") # random rebase of executable at load time
IF (CMAKE_SIZEOF_VOID_P EQUAL 4)
  SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /SafeSEH")     # invoke known exception handlers (Win32 only, x64 exception handlers are safe by design)
ENDIF()
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SECURE_LINKER_FLAGS}")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${SECURE_LINKER_FLAGS}")

SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4217")  # locally defined symbol XXX imported in function YYY (happens as the ISPC API layer uses exported library functions)
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4049")  # warning LNK4049: locally defined symbol _rtcOccluded1M imported

INCLUDE(msvc_post)

