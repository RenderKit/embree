## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
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

#IF (__XEON__)
#
#  ADD_LIBRARY(tasking STATIC
#    taskscheduler.cpp
#    taskscheduler_sys.cpp
#    tasksys.cpp
#  )

#  TARGET_LINK_LIBRARIES(tasking sys)
#  SET_PROPERTY(TARGET sys PROPERTY FOLDER kernels)

#ELSE ()
 
#  INCLUDE(icc_xeonphi)

#  ADD_LIBRARY(tasking_xeonphi STATIC
#    taskscheduler.cpp
#    taskscheduler_sys.cpp
#    taskscheduler_mic.cpp
#    tasksys.cpp
#  )

#  TARGET_LINK_LIBRARIES(tasking_xeonphi sys_xeonphi)

#ENDIF ()
