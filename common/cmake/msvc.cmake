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

SET(FLAGS_SSE2  "/D__SSE__ /D__SSE2__")
SET(FLAGS_SSE42 "${FLAGS_SSE2} /D__SSE3__ /D__SSSE3__ /D__SSE4_1__ /D__SSE4_2__")
SET(FLAGS_AVX   "${FLAGS_SSE42} /arch:AVX")
SET(FLAGS_AVX2  "${FLAGS_SSE42} /arch:AVX2")

SET(COMMON_CXX_FLAGS "/EHsc /MP /GR /GS- ${FLAGS_SSE2}")

SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} ${COMMON_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        ${COMMON_CXX_FLAGS} /Ox /Oi /Gy")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${COMMON_CXX_FLAGS} /Ox /Oi /Gy")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4217")  # locally defined symbol XXX imported in function YYY (happens as the ISPC API layer uses exported library functions)
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4049")  # warning LNK4049: locally defined symbol _rtcOccluded1M imported
SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG")

INCLUDE(msvc_post)

