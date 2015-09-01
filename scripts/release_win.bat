@echo off

setlocal
set TBB_PATH=%cd%\tbb

mkdir -p build_win32
cd build_win32
del CMakeCache.txt
del version.h

REM set release settings
cmake -L ^
-G "Visual Studio 12 2013" ^
-T "Intel C++ Compiler XE 15.0" ^
-A "Win32" ^
-D COMPILER=ICC ^
-D XEON_ISA=AVX2 ^
-D TBB_ROOT=%TBB_PATH% ^
-D ENABLE_XEON_PHI_SUPPORT=OFF ^
-D USE_IMAGE_MAGICK=OFF ^
-D USE_LIBJPEG=OFF  ^
-D USE_LIBPNG=OFF ^
-D USE_OPENEXR=OFF ^
..

REM compile
cmake --clean-first --build . --config Release --target PREINSTALL -- /m /nologo /verbosity:n

REM create installers
cmake -D ENABLE_INSTALLER=ON ..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

REM create ZIP files
cmake -D ENABLE_INSTALLER=OFF ..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

cd ..
mkdir build_x64
cd build_x64
del CMakeCache.txt # make sure to use default settings
del version.h

REM set release settings
cmake -L ^
-G "Visual Studio 12 2013 Win64" ^
-T "Intel C++ Compiler XE 15.0" ^
-A "x64" ^
-D COMPILER=ICC ^
-D XEON_ISA=AVX2 ^
-D TBB_ROOT=%TBB_PATH% ^
-D ENABLE_XEON_PHI_SUPPORT=OFF ^
-D USE_IMAGE_MAGICK=OFF ^
-D USE_LIBJPEG=OFF  ^
-D USE_LIBPNG=OFF ^
-D USE_OPENEXR=OFF ^
..

REM compile
cmake --clean-first --build . --config Release --target PREINSTALL -- /m /nologo /verbosity:n

REM create installers
cmake -D ENABLE_INSTALLER=ON ..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

REM create ZIP files
cmake -D ENABLE_INSTALLER=OFF ..
cmake --build . --config Release --target PACKAGE -- /m /nologo /verbosity:n

cd ..

:abort
endlocal
:end
