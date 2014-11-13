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

SET(FLAGS_SSE2  "")
SET(FLAGS_SSE3  "")
SET(FLAGS_SSSE3 "")
SET(FLAGS_SSE41 " /DCONFIG_SSE41")
SET(FLAGS_SSE42 " /DCONFIG_SSE42")
SET(FLAGS_AVX   "/arch:AVX /DCONFIG_AVX")
SET(FLAGS_AVX2  "/arch:AVX2 /DCONFIG_AVX2")
SET(FLAGS_AVX512 "")

SET(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} /Ox /fp:fast /Qpar /Oi /Gy /GR-")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Ox /fp:fast /Qpar /Oi /Gy /GR-")