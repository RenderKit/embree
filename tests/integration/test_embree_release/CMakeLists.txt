cmake_minimum_required(VERSION 3.19)

project(TestEmbreeRelease)

include(GNUInstallDirs)
include(FetchContent)

set(BUILD_TESTING OFF CACHE INTERNAL "")
set(CATCH_INSTALL_DOCS OFF CACHE INTERNAL "")
set(CATCH_INSTALL_EXTRAS OFF CACHE INTERNAL "")
set(CATCH_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(CATCH_BUILD_TESTING OFF CACHE INTERNAL "")

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.3.2
)
FetchContent_MakeAvailable(Catch2)

find_package(embree REQUIRED)

add_executable(test test.cpp)
target_link_libraries(test PRIVATE Catch2::Catch2WithMain embree)

INSTALL(TARGETS test DESTINATION "${CMAKE_INSTALL_BINDIR}")

IF(TARGET embree)
  GET_TARGET_PROPERTY(LIB_PATH embree IMPORTED_LOCATION_RELEASE)
  IF(WIN32)
    GET_FILENAME_COMPONENT(LIB_DIR "${LIB_PATH}" DIRECTORY)
    FILE(GLOB LIB_FILES ${LIB_DIR}/*.dll)
    foreach(LIB_FILE ${LIB_FILES})
      INSTALL(FILES "${LIB_FILE}" DESTINATION "${CMAKE_INSTALL_BINDIR}")
    endforeach()
  ELSE()
    GET_FILENAME_COMPONENT(LIB_DIR "${LIB_PATH}" DIRECTORY)
    FILE(GLOB LIB_FILES ${LIB_DIR}/*.so*)
    INSTALL(FILES ${LIB_FILES} DESTINATION "${CMAKE_INSTALL_BINDIR}")
  ENDIF()
ELSE()
  MESSAGE(SEND_ERROR "Target embree not found.")
ENDIF()
