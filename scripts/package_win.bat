rem ## Copyright 2009-2021 Intel Corporation
rem ## SPDX-License-Identifier: Apache-2.0

@echo off

set build_type=%1
set outfile=%2
set outarch=%3  
set signfile=%4

if "%build_type%" == "" (
  set build_type=Release
)

cmake --build . --config %build_type% --target package --verbose

REN %outfile%.%outarch%.windows-embree.zip %outfile%.%outarch%.windows.zip
REN %outfile%.%outarch%.windows-embree-testing.zip %outfile%.%outarch%.windows-testing.zip
DEL %outfile%.%outarch%.windows-embree-Unspecified.zip


IF %ERRORLEVEL% NEQ 0 (
  exit /b 1
)

IF [%signfile%] NEQ [] (
  %signfile% %outfile%.%outarch%.windows.zip
  %signfile% %outfile%.%outarch%.windows-testing.zip
)

IF %ERRORLEVEL% NEQ 0 (
  exit /b 1
)
