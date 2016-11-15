@echo off

setlocal

REM create package
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

@echo "<DartMeasurementFile name="%1" type="file">%1</DartMeasurementFile>"

:abort
endlocal
:end
