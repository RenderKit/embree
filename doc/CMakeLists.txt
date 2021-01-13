## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.11)

execute_process(COMMAND git config --get remote.origin.url
                RESULT_VARIABLE result
                OUTPUT_VARIABLE intelstyle_url)
if (result)
  return()
endif()

string(REGEX REPLACE "\n$" "" intelstyle_url "${intelstyle_url}")
string(REPLACE "embree" "intelstyle" intelstyle_url "${intelstyle_url}")

message(STATUS "Fetch intelstyle repository ${intelstyle_url}")
include(FetchContent)
FetchContent_Declare(IntelStyle
                     GIT_REPOSITORY ${intelstyle_url}
                     SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/intelstyle)

FetchContent_GetProperties(IntelStyle)
if(NOT IntelStyle_POPULATED)
  FetchContent_Populate(IntelStyle)
endif()
