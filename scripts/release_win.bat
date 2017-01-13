@echo off

setlocal
set TBB_PATH_LOCAL=%cd%\tbb

mkdir -p build_win32
cd build_win32
del CMakeCache.txt
del version.h

REM set release settings
cmake -L ^
-G "Visual Studio 12 2013" ^
-T "Intel C++ Compiler 16.0" ^
-D EMBREE_MAX_ISA=AVX2 ^
-D EMBREE_TUTORIALS_IMAGE_MAGICK=OFF ^
-D EMBREE_TUTORIALS_LIBJPEG=OFF  ^
-D EMBREE_TUTORIALS_LIBPNG=OFF ^
-D EMBREE_TBB_ROOT=%TBB_PATH_LOCAL% ^
..

REM compile
cmake --clean-first --build . --config Release --target PREINSTALL -- /m /nologo /verbosity:n

REM create ZIP files
cmake ^
-D EMBREE_ZIP_MODE=ON ^
-D CMAKE_INSTALL_INCLUDEDIR=include ^
-D CMAKE_INSTALL_LIBDIR=lib ^
-D CMAKE_INSTALL_DATAROOTDIR= ^
-D CMAKE_INSTALL_DOCDIR=doc ^
-D CMAKE_INSTALL_BINDIR=bin ^
..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

REM create installers
cmake ^
-D EMBREE_ZIP_MODE=OFF ^
-D CMAKE_INSTALL_INCLUDEDIR=include ^
-D CMAKE_INSTALL_LIBDIR=lib ^
-D CMAKE_INSTALL_DATAROOTDIR= ^
-D CMAKE_INSTALL_DOCDIR=doc ^
-D CMAKE_INSTALL_BINDIR=bin ^
..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

cd ..
mkdir build_x64
cd build_x64
del CMakeCache.txt # make sure to use default settings
del version.h

REM set release settings
cmake -L ^
-G "Visual Studio 12 2013 Win64" ^
-T "Intel C++ Compiler 16.0" ^
-D EMBREE_MAX_ISA=AVX2 ^
-D EMBREE_TBB_ROOT=%TBB_PATH_LOCAL% ^
-D ENABLE_XEON_PHI_SUPPORT=OFF ^
-D EMBREE_TUTORIALS_IMAGE_MAGICK=OFF ^
-D EMBREE_TUTORIALS_LIBJPEG=OFF  ^
-D EMBREE_TUTORIALS_LIBPNG=OFF ^
..

REM compile
cmake --clean-first --build . --config Release --target PREINSTALL -- /m /nologo /verbosity:n

REM create installers
cmake ^
-D EMBREE_ZIP_MODE=OFF ^
-D CMAKE_INSTALL_INCLUDEDIR=include ^
-D CMAKE_INSTALL_LIBDIR=lib ^
-D CMAKE_INSTALL_DATAROOTDIR= ^
-D CMAKE_INSTALL_DOCDIR=doc ^
-D CMAKE_INSTALL_BINDIR=bin ^
..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

REM create ZIP files
cmake ^
-D EMBREE_ZIP_MODE=ON ^
-D CMAKE_INSTALL_INCLUDEDIR=include ^
-D CMAKE_INSTALL_LIBDIR=lib ^
-D CMAKE_INSTALL_DATAROOTDIR= ^
-D CMAKE_INSTALL_DOCDIR=doc ^
-D CMAKE_INSTALL_BINDIR=bin ^
..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

del CMakeCache.txt 
cd ..

:abort
endlocal
:end
