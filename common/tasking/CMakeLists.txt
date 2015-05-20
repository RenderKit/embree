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

IF (__XEON__)

# FIXME: have to add these CPP files directly in libembree as otherwise globals of the tasking system exist twice under VS2012 Win32

#  IF (TASKING_INTERNAL)
#    ADD_DEFINITIONS(-DTASKING_TBB_INTERNAL)
#  ENDIF()

#  IF (TASKING_TBB)
#    ADD_DEFINITIONS(-DTASKING_TBB)
#  ENDIF()

#  ADD_LIBRARY(tasking STATIC
#    taskscheduler_tbb.cpp
#    tasksys.cpp
#  )

#  IF (TASKING_TBB)
#    TARGET_LINK_LIBRARIES(tasking sys ${TBB_LIBRARIES})
#  ELSE()
#    TARGET_LINK_LIBRARIES(tasking sys)
#  ENDIF()

#  SET_PROPERTY(TARGET tasking PROPERTY FOLDER kernels)

ELSE ()

#  #IF (TASKING_INTERNAL)
#    ADD_DEFINITIONS(-DTASKING_LOCKSTEP)
#  #ELSE()
#  #  ADD_DEFINITIONS(-DTASKING_TBB)
#  #  INCLUDE_DIRECTORIES(${TBB_INCLUDE_DIRS_MIC})
#  #  SET(ADDITIONAL_LIBRARIES ${TBB_LIBRARIES_MIC})
#  #ENDIF()
 
#  INCLUDE(icc_xeonphi)

#  ADD_LIBRARY(tasking_xeonphi STATIC
#    taskscheduler_tbb.cpp # FIXME: remove
#    taskscheduler_mic.cpp
#    tasksys.cpp
#  )

#  TARGET_LINK_LIBRARIES(tasking_xeonphi sys_xeonphi)

ENDIF ()
