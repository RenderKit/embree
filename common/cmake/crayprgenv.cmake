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

SET(FLAGS_SSE2      "-target-cpu=xeon")
SET(FLAGS_SSE42     "-target-cpu=interlagos")
SET(FLAGS_AVX       "-target-cpu=sandybridge")
SET(FLAGS_AVX2      "-target-cpu=haswell")
SET(FLAGS_AVX512KNL "-target-cpu=mic-knl")
SET(FLAGS_AVX512SKX "-target-cpu=x86-skylake")
