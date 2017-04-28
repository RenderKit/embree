## ======================================================================== ##
## Copyright 2009-2017 Intel Corporation                                    ##
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

IF (WIN32)

  SET(FLAGS_SSE2  "/QxSSE2")
  SET(FLAGS_SSE42 "/QxSSE4.2")
  SET(FLAGS_AVX   "/arch:AVX")
  SET(FLAGS_AVX2  "/QxCORE-AVX2")
  SET(FLAGS_AVX512KNL "/QxMIC-AVX512")
  SET(FLAGS_AVX512SKX "/QxCORE-AVX512")

  SET(COMMON_CXX_FLAGS "/EHsc /MP /GR /GS-")
  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /Qdiag-disable:11074 ")  # remark #11074: Inlining inhibited by limit max-size
  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /Qdiag-disable:11075 ")  # remark #11075: To get full report use -Qopt-report:4 -Qopt-report-phase ipo
  
  SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} ${COMMON_CXX_FLAGS}")
  SET(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        ${COMMON_CXX_FLAGS} /Ox /Oi /Gy /Qinline-max-total-size- /Qinline-factor=200 /Qvec- ")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${COMMON_CXX_FLAGS} /Ox /Oi /Gy /Qinline-max-total-size- /Qinline-factor=200 /Qvec- ")
  
  SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4217")  # locally defined symbol XXX imported in function YYY (happens as the ISPC API layer uses exported library functions)
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4049")  # warning LNK4049: locally defined symbol _rtcOccluded1M imported
  SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG")
  
  INCLUDE(msvc_post)

ELSE()

  IF (APPLE)
    SET(FLAGS_SSE2   "-xssse3") # in MacOSX ICC does not support SSE2
  ELSE()
    SET(FLAGS_SSE2   "-xsse2")
  ENDIF()
  SET(FLAGS_SSE42  "-xsse4.2")
  SET(FLAGS_AVX    "-xAVX")
  SET(FLAGS_AVX2   "-xCORE-AVX2")
  SET(FLAGS_AVX512KNL "-xMIC-AVX512")
  SET(FLAGS_AVX512SKX "-xCORE-AVX512")

  OPTION(EMBREE_IGNORE_CMAKE_CXX_FLAGS "When enabled Embree ignores default CMAKE_CXX_FLAGS." ON)
  IF (EMBREE_IGNORE_CMAKE_CXX_FLAGS)
    SET(CMAKE_CXX_FLAGS "")
  ENDIF()
  
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -std=c++11 -fvisibility-inlines-hidden -fvisibility=hidden -no-ansi-alias -fasm-blocks")
  SET(CMAKE_CXX_FLAGS_DEBUG          "-DDEBUG  -DTBB_USE_DEBUG -g -O0")
  SET(CMAKE_CXX_FLAGS_RELEASE        "-DNDEBUG                    -O3 -restrict -no-inline-max-total-size -inline-factor=200 -no-inline-max-per-compile -no-vec")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-DDEBUG  -DTBB_USE_DEBUG -g -O3 -restrict -no-inline-max-total-size -inline-factor=200 -no-inline-max-per-compile -no-vec")
  
  # Don't change linker flags used by Cray wrappers
  IF(NOT CMAKE_CXX_COMPILER_WRAPPER STREQUAL "CrayPrgEnv")
    # enable -static-intel and -no-intel-extensions to void to export ICC specific symbols from Embree
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-intel -no-intel-extensions")
  ENDIF()
  
  #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -debug inline-debug-info")
  #SET(CMAKE_EXE_LINKER_FLAGS "-g") 
  
  IF (APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.7 ") # we only use MacOSX 10.7 features
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++"            ) # link against C++11 stdlib
  ELSE(APPLE)
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
  ENDIF(APPLE)

ENDIF()
