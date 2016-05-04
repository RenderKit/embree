## ======================================================================== ##
## Copyright 2009-2016 Intel Corporation                                    ##
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

IF (TASKING_INTERNAL)
  ADD_DEFINITIONS(-DTASKING_INTERNAL)
ELSE()
  ADD_DEFINITIONS(-DTASKING_TBB)
ENDIF()

IF (TARGET_SSE41)
  ADD_DEFINITIONS(-D__TARGET_SSE41__)
ENDIF()

IF (TARGET_AVX)
  ADD_DEFINITIONS(-D__TARGET_AVX__)
ENDIF()

IF (TARGET_AVX2)
  ADD_DEFINITIONS(-D__TARGET_AVX2__)
ENDIF()

IF (TARGET_AVX512KNL)
  ADD_DEFINITIONS(-D__TARGET_AVX512KNL__)
ENDIF()

ADD_EXECUTABLE(verify verify.cpp ../tutorials/common/tutorial/application.cpp)
TARGET_LINK_LIBRARIES(verify sys embree)
SET_PROPERTY(TARGET verify PROPERTY FOLDER tests)
ADD_TEST(NAME verify COMMAND verify)

ADD_EXECUTABLE(benchmark benchmark.cpp)
TARGET_LINK_LIBRARIES(benchmark sys embree)
SET_PROPERTY(TARGET benchmark PROPERTY FOLDER tests)
ADD_TEST(NAME benchmark COMMAND benchmark)

ADD_EXECUTABLE(retrace retrace.cpp)
TARGET_LINK_LIBRARIES(retrace sys simd embree)
SET_PROPERTY(TARGET retrace PROPERTY FOLDER tests)

INSTALL(TARGETS verify benchmark retrace DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)

SET(CPACK_NSIS_MENU_LINKS ${CPACK_NSIS_MENU_LINKS} "${CMAKE_INSTALL_BINDIR}/verify" "verify")
SET(CPACK_NSIS_MENU_LINKS ${CPACK_NSIS_MENU_LINKS} "${CMAKE_INSTALL_BINDIR}/benchmark" "benchmark")
SET(CPACK_NSIS_MENU_LINKS ${CPACK_NSIS_MENU_LINKS} "${CMAKE_INSTALL_BINDIR}/retrace" "retrace")
SET(CPACK_NSIS_MENU_LINKS ${CPACK_NSIS_MENU_LINKS} PARENT_SCOPE)
