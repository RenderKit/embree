## ======================================================================== ##
## Copyright 2017 Kitware, Inc.                                             ##
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

SET(CHECK_ISA_DIR ${CMAKE_CURRENT_LIST_DIR})
FUNCTION(CHECK_ISA_DEFAULT OUTVAR)
  SET(ISA_DEFAULT_BIN ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/check_isa_default.bin)
  SET(SRC ${CHECK_ISA_DIR}/check_isa.cpp)
  TRY_COMPILE(ISA_DEFAULT_COMPILE
    ${CMAKE_BINARY_DIR}
    ${SRC}
    COPY_FILE ${ISA_DEFAULT_BIN}
  )
  IF(NOT ISA_DEFAULT_COMPILE)
    SET(ISA_DEFAULT "SSE2")
    RETURN()
  ENDIF()
  FILE(STRINGS ${ISA_DEFAULT_BIN} ISA_DEFAULT REGEX "^ISA:")
  STRING(REPLACE "ISA:" "" ISA_DEFAULT "${ISA_DEFAULT}")
  SET(${OUTVAR} ${ISA_DEFAULT} PARENT_SCOPE)
ENDFUNCTION()
