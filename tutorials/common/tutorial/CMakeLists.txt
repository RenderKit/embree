## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_TUTORIALS_GLFW)
  SET(IMGUI_LIBRARY imgui)
ENDIF()

ADD_LIBRARY(tutorial STATIC tutorial.cpp application.cpp scene.cpp tutorial_device.cpp scene_device.cpp)
TARGET_LINK_LIBRARIES(tutorial sys math lexers scenegraph lights embree ${IMGUI_LIBRARY} tasking ${OPENGL_LIBRARIES} ${GLFW_LIBRARY})
SET_PROPERTY(TARGET tutorial PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET tutorial APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

ADD_LIBRARY(noise STATIC noise.cpp)
SET_PROPERTY(TARGET noise PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET noise APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_ISPC_SUPPORT)
  ADD_ISPC_LIBRARY(tutorial_ispc STATIC tutorial.cpp application.cpp scene.cpp tutorial_device.ispc scene_device.cpp tasksys.cpp)
  TARGET_LINK_LIBRARIES(tutorial_ispc sys math lexers scenegraph lights_ispc embree ${IMGUI_LIBRARY} tasking ${OPENGL_LIBRARIES} ${GLFW_LIBRARY})
  SET_PROPERTY(TARGET tutorial_ispc PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET tutorial_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

  ADD_ISPC_LIBRARY(noise_ispc STATIC noise.ispc)
  SET_TARGET_PROPERTIES(noise_ispc PROPERTIES LINKER_LANGUAGE CXX)
  SET_PROPERTY(TARGET noise_ispc PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET noise_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
ENDIF()

IF (WIN32 AND NOT EMBREE_EXTERNAL_GLFW)

  GET_FILENAME_COMPONENT(GLFW_DIR ${GLFW_LIBRARY} PATH)
  ADD_CUSTOM_COMMAND(TARGET tutorial POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GLFW_DIR}/glfw3.dll $<TARGET_FILE_DIR:tutorial>
    COMMENT "Copying GLFW DLL" VERBATIM
  )
  IF (EMBREE_SIGN_FILE)
    ADD_CUSTOM_COMMAND(TARGET tutorial POST_BUILD
      COMMAND ${EMBREE_SIGN_FILE} $<TARGET_FILE_DIR:tutorial>/glfw3.dll
      COMMENT "Signing GLFW DLL" VERBATIM
    )
  ENDIF()
  INSTALL(PROGRAMS $<TARGET_FILE_DIR:tutorial>/glfw3.dll DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)

ENDIF()

ADD_CUSTOM_COMMAND(TARGET tutorial POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/tutorials/models $<TARGET_FILE_DIR:tutorial>/models
  COMMENT "Copying example models")


