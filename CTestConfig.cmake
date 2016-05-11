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

set(CTEST_PROJECT_NAME "Embree")
set(CTEST_NIGHTLY_START_TIME "22:00:00 UTC")
set(TEST_MODELS_HASH d8e7eaa8092c8f5ef00a835d1d34862b4e029155)

IF (NOT CTEST_DROP_SITE)
  set(CTEST_DROP_METHOD "http")
  set(CTEST_DROP_SITE "cdash")
  set(CTEST_DROP_LOCATION "/CDash/submit.php?project=Embree")
  set(CTEST_DROP_SITE_CDASH TRUE)
endif()

list (APPEND CTEST_CUSTOM_WARNING_EXCEPTION "warning #1478")  # deprecated function used
list (APPEND CTEST_CUSTOM_WARNING_EXCEPTION "warning #10237") # -lcilkrts linked in dynamically, static library not available")
list (APPEND CTEST_CUSTOM_WARNING_EXCEPTION "-Wextern-initializer")
