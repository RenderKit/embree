## ======================================================================== ##
## Copyright 2009-2017 Intel Corporation                                    ##
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

UNSET(TASKING_LIBRARIES CACHE)

IF (TASKING_INTERNAL)
  ADD_LIBRARY(tasking_obj OBJECT taskschedulerinternal.cpp)
  SET(TASKING_LIBRARIES CACHE INTERNAL "")
  ADD_LIBRARY(tasking STATIC $<TARGET_OBJECTS:tasking_obj>)
ENDIF()

IF (TASKING_TBB)
  ADD_LIBRARY(tasking_obj OBJECT taskschedulertbb.cpp)
  ADD_LIBRARY(tasking STATIC $<TARGET_OBJECTS:tasking_obj>)  
  SET(TASKING_LIBRARIES ${TBB_LIBRARIES} CACHE INTERNAL "")
  TARGET_LINK_LIBRARIES(tasking sys math ${TBB_LIBRARIES})
ENDIF()

IF (TASKING_PPL)
  ADD_LIBRARY(tasking_obj OBJECT taskschedulerppl.cpp)
  ADD_LIBRARY(tasking STATIC $<TARGET_OBJECTS:tasking_obj>)
  SET(TASKING_LIBRARIES ${PPL_LIBRARIES} CACHE INTERNAL "")
  TARGET_LINK_LIBRARIES(tasking sys math ${PPL_LIBRARIES})
ENDIF()

SET_PROPERTY(TARGET tasking PROPERTY FOLDER common)
