@echo off

if "%~1" == "" (
  echo Usage: release_win.bat path-to-bin-folder
  goto end
)

setlocal
set destdir="%cd%\%~1"

mkdir build
cd build
del CMakeCache.txt rem make sure to use default settings
cmake -D CMAKE_INSTALL_PREFIX=%destdir% -G "Visual Studio 12 2013 Win64" ..
ICProjConvert150 embree.sln /IC /s /f
cmake --build . --config Release --target INSTALL -- /m
cd ..

endlocal
:end
