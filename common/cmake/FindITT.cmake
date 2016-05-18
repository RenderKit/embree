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

FIND_PATH   (ITT_INCLUDE_DIR ittnotify.h /opt/intel/inspector_xe/include)
FIND_LIBRARY(ITT_LIBRARY NAMES ittnotify PATHS /opt/intel/inspector_xe/lib)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ITT DEFAULT_MSG ITT_INCLUDE_DIR ITT_LIBRARY)

MARK_AS_ADVANCED(
  ITT_INCLUDE_DIR
  ITT_LIBRARY
)
