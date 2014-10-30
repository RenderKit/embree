## ======================================================================== ##
## Copyright 2009-2014 Intel Corporation                                    ##
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

FIND_PATH( OPENSUBDIV_INCLUDE_DIR NAMES opensubdiv/version.h )
FIND_LIBRARY( OPENSUBDIV_LIBRARY NAMES osdCPU)

IF (OPENSUBDIV_INCLUDE_DIR AND OPENSUBDIV_LIBRARY)
  SET(OPENSUBDIV_FOUND TRUE)
ELSE ()
  MESSAGE(FATAL_ERROR "OpenSubdiv not found")
ENDIF ()

MARK_AS_ADVANCED(OPENSUBDIV_FOUND)
MARK_AS_ADVANCED(OPENSUBDIV_INCLUDE_DIR)
MARK_AS_ADVANCED(OPENSUBDIV_LIBRARY)
