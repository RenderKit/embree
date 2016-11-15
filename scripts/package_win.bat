@echo off

setlocal

REM create package
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

set mode=%1
set version=%2
set platform=%3

IF %mode% == "ON" (
  embree_zip=embree-${EMBREE_VERSION}.%platform%.windows.zip
  @echo  "<DartMeasurementFile name=\"%embree_zip%\" type=\"file\">%embree_zip%</DartMeasurementFile>"
) ELSE (
  embree_installer=embree-${EMBREE_VERSION}.%platform%.exe
  @echo  "<DartMeasurementFile name=\"%embree_installer%\" type=\"file\">%embree_installer%</DartMeasurementFile>"
)

:abort
endlocal
:end
