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

SET(PACKAGE_VERSION 2.10.0)

IF (${PACKAGE_FIND_VERSION_MAJOR} EQUAL 2)
  IF (${PACKAGE_FIND_VERSION} VERSION_LESS 2.10.0)
    SET(PACKAGE_VERSION_COMPATIBLE 1)  
  ENDIF()
  IF (${PACKAGE_FIND_VERSION} VERSION_EQUAL 2.10.0)
    SET(PACKAGE_VERSION_EXACT 1)  
  ENDIF()
ELSE()
  SET(PACKAGE_VERSION_UNSUITABLE 1)
ENDIF()
