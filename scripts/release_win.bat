@echo off

if "%~1" == "" (
  echo Usage: release_win.bat path-to-bin-folder
  goto end
)

setlocal
set destdir=%cd%\%~1

mkdir build
cd build
del CMakeCache.txt rem make sure to use default settings
cmake -D CMAKE_INSTALL_PREFIX=%destdir%\x64 -D COMPILER=ICC -G "Visual Studio 12 2013 Win64" ..
ICProjConvert150 embree.sln /IC /s /f
cmake --build . --config Release --target INSTALL -- /m
cmake --build . --config Release --target PACKAGE -- /m
copy embree*.exe %destdir%
cd ..

mkdir build32
cd build32
del CMakeCache.txt rem make sure to use default settings
cmake -D CMAKE_INSTALL_PREFIX=%destdir%\win32 -D COMPILER=ICC -G "Visual Studio 12 2013" ..
ICProjConvert150 embree.sln /IC /s /f
cmake --build . --config Release --target INSTALL -- /m
cmake --build . --config Release --target PACKAGE -- /m
copy embree*.exe %destdir%
cd ..

endlocal
:end
