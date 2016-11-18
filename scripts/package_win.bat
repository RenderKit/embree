@echo off

setlocal

cmake --build . --config %1 --target PACKAGE -- /m /nologo /verbosity:n

@echo "<DartMeasurement name="%2" type="text/string">%2</DartMeasurement>"

:abort
endlocal
:end
