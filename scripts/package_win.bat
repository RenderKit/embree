@echo off

set build_type=%2
if "%build_type%" == "" (
  set build_type=Release
)

cmake --build . --config %build_type% --target PACKAGE -- /m /nologo /verbosity:n

@echo "<DartMeasurement name="%1" type="text/string">%1</DartMeasurement>"

