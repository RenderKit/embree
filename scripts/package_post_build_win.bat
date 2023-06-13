rem ## Copyright 2009-2021 Intel Corporation
rem ## SPDX-License-Identifier: Apache-2.0

@echo off

set outfile=%1
set outarch=%2
set signfile=%3

REN %outfile%.%outarch%.windows-embree.zip %outfile%.%outarch%.windows.zip
REN %outfile%.%outarch%.windows-embree-testing.zip %outfile%.%outarch%.windows-testing.zip
DEL %outfile%.%outarch%.windows-Unspecified.zip


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
