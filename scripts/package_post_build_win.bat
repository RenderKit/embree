rem ## Copyright 2009-2021 Intel Corporation
rem ## SPDX-License-Identifier: Apache-2.0

@echo off

set package_name=%1
set signfile=%2

REN %package_name%-embree.zip %package_name%.zip
REN %package_name%-embree-testing.zip %package_name%-testing.zip
DEL %package_name%-Unspecified.zip


IF %ERRORLEVEL% NEQ 0 (
  exit /b 1
)

IF %ERRORLEVEL% NEQ 0 (
  exit /b 1
)
