:: Copyright 2009-2021 Intel Corporation
:: SPDX-License-Identifier: Apache-2.0

@echo off

:: only account for failures in this script
set ERRORLEVEL=0

set package_name=%1

:: we should be in build folder
cmake --build . --target package -j8

:: rename packages
echo renaming %package_name%-embree.zip         -> %package_name%.zip
REN %package_name%-embree.zip %package_name%.zip
echo renaming %package_name%-embree-testing.zip -> %package_name%-testing.zip
REN %package_name%-embree-testing.zip %package_name%-testing.zip
IF EXIST %package_name%-Unspecified.zip (
  echo deleting %package_name%-Unspecified.zip
  DEL %package_name%-Unspecified.zip
)

IF %ERRORLEVEL% NEQ 0 (
  exit /b 1
)
