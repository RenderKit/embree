@echo off

if "%~1" == "" (
  echo Usage: release_win.bat path-to-bin-folder
  goto end
)

setlocal
set destroot=%cd%\%~1
set DESTDIR=

mkdir build
cd build
del CMakeCache.txt rem make sure to use default settings
cmake -D CMAKE_INSTALL_PREFIX=%destroot%\x64 -D COMPILER=ICC -G "Visual Studio 12 2013 Win64" ..
ICProjConvert150 embree.sln /IC /s /f
cmake --build . --config Release --target INSTALL -- /m
cmake --build . --config Release --target PACKAGE -- /m
copy embree*.exe %destroot%
cd ..

mkdir build32
cd build32
del CMakeCache.txt rem make sure to use default settings
cmake -D CMAKE_INSTALL_PREFIX=%destroot%\win32 -D COMPILER=ICC -G "Visual Studio 12 2013" ..
ICProjConvert150 embree.sln /IC /s /f
cmake --build . --config Release --target INSTALL -- /m
cmake --build . --config Release --target PACKAGE -- /m
copy embree*.exe %destroot%
cd ..

endlocal
:end
