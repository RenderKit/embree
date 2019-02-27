@echo off

set build_type=%1
set outfile=%2
set signfile=%3

if "%build_type%" == "" (
  set build_type=Release
)

cmake --build . --config %build_type% --target PACKAGE -- /m /nologo /verbosity:n

IF %ERRORLEVEL% NEQ 0 (
  exit /b 1
)

IF [%signfile%] NEQ [] (
  %signfile% %outfile%
)

IF %ERRORLEVEL% NEQ 0 (
  exit /b 1
)
