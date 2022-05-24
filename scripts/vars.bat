:: Copyright 2009-2021 Intel Corporation
:: SPDX-License-Identifier: Apache-2.0

@echo off

set "VARSDIR=%~dp0"
if not defined CMPLR_ROOT for /f "delims=" %%F in ("%VARSDIR%..") do set "CMPLR_ROOT=%%~fF"

set "SCRIPT_NAME=%~nx0"
set "VS_TARGET_ARCH="
set "INTEL_TARGET_ARCH="
set "INTEL_TARGET_PLATFORM=windows"
set "USE_INTEL_LLVM=0"

set "DPCPP_DIR="
set "GFX_DIR="

if /i "%1"=="" (
    echo "error set dpcpp version" & goto END
) else (
    set DPCPP_DIR=%1

)

shift

if /i "%1"=="" (
    echo "error set gfx version" & goto END
) else (
    set GFX_DIR=%1

)

shift

:ParseArgs
:: Parse the incoming arguments
if /i "%1"==""              goto CheckArgs
if /i "%1"=="ia32"          (set INTEL_TARGET_ARCH=ia32)     & (set TARGET_VS_ARCH=x86)     & shift & goto ParseArgs
if /i "%1"=="intel64"       (set INTEL_TARGET_ARCH=intel64)  & (set TARGET_VS_ARCH=amd64)   & shift & goto ParseArgs
if /i "%1"=="vs2017"        (set TARGET_VS=vs2017)           & shift & goto ParseArgs
if /i "%1"=="vs2019"        (set TARGET_VS=vs2019)           & shift & goto ParseArgs
if /i "%1"=="--include-intel-llvm"   (set USE_INTEL_LLVM=1)  & shift & goto ParseArgs
shift & goto ParseArgs

:CheckArgs
:: set correct defaults
if /i "%INTEL_TARGET_ARCH%"==""   (set INTEL_TARGET_ARCH=intel64) & (set TARGET_VS_ARCH=amd64)

:: Setup Intel Compiler environment directly if Visual Studio environment is ready.
if defined VSCMD_VER (
    if /i "%VSCMD_ARG_TGT_ARCH%"=="x86" (
        set INTEL_TARGET_ARCH=ia32
    ) else (
        set INTEL_TARGET_ARCH=intel64
    )
    echo "SetIntelEnv"
    goto SetIntelEnv
)

::detect installed VS
set "MSVS_VAR_SCRIPT="

:: The exact installation directory depends on both the version and offering of Visual Studio,
:: according to the following pattern: C:\Program Files (x86)\Microsoft Visual Studio\<version>\<offering>.
if defined VS2019INSTALLDIR (
    goto SetVCVars
)
if defined VS2017INSTALLDIR (
    goto SetVCVars
)

if /i "%TARGET_VS%"=="" (
    call :SetVS2019INSTALLDIR
    if not defined VS2019INSTALLDIR (
        call :SetVS2017INSTALLDIR
    )
    goto SetVCVars
)

if /i "%TARGET_VS%"=="vs2019" (
    if not defined VS2019INSTALLDIR (
        call :SetVS2019INSTALLDIR
    )
    goto SetVCVars
)

if /i "%TARGET_VS%"=="vs2017" (
    if not defined VS2017INSTALLDIR (
        call :SetVS2017INSTALLDIR
    )
    goto SetVCVars
)

::default, set the latest VS in global environment
:SetVCVars
if /i "%TARGET_VS%"=="" (
    ::vs2019
    if defined VS2019INSTALLDIR (
        if exist "%VS2019INSTALLDIR%\VC\Auxiliary\Build\vcvarsall.bat" (
            goto SetVS2019
        )
    )
    ::vs2017
    if defined VS2017INSTALLDIR (
        if exist "%VS2017INSTALLDIR%\VC\Auxiliary\Build\vcvarsall.bat" (
            goto SetVS2017
        )
    )
    call :NO_VS 2017 or 2019
    goto EndWithError
)

::VS2019
if /i "%TARGET_VS%"=="vs2019" (
    if defined VS2019INSTALLDIR (
        if exist "%VS2019INSTALLDIR%\VC\Auxiliary\Build\vcvarsall.bat" (
            goto SetVS2019
        )
    )
    call :NO_VS 2019
    goto EndWithError
)

::VS2017
if /i "%TARGET_VS%"=="vs2017" (
    if defined VS2017INSTALLDIR (
        if exist "%VS2017INSTALLDIR%\VC\Auxiliary\Build\vcvarsall.bat" (
            goto SetVS2017
        )
    )
    call :NO_VS 2017
    goto EndWithError
)

:SetVS2019
set "TARGET_VS=vs2019"
set MSVS_VAR_SCRIPT="%VS2019INSTALLDIR%\VC\Auxiliary\Build\vcvarsall.bat"
goto Setup

:SetVS2017
set "TARGET_VS=vs2017"
set MSVS_VAR_SCRIPT="%VS2017INSTALLDIR%\VC\Auxiliary\Build\vcvarsall.bat"
goto Setup

:Setup

echo "MSVS_VAR_SCRIPT %MSVS_VAR_SCRIPT%"

:: call visual studio VARs script
:: ============================================================================
if "%VSCMD_START_DIR%"=="" (
    if EXIST "%USERPROFILE%\Source" (
        set "VSCMD_START_DIR=%CD%"
    )
)

@call %MSVS_VAR_SCRIPT% %TARGET_VS_ARCH% 1>NUL

call :GetFullPath %MSVS_VAR_SCRIPT%\.. MSVS_VAR_SCRIPT_DIR
if /i "%INTEL_TARGET_ARCH%"=="ia32" (
    if defined VCToolsInstallDir (
        if exist "%VCToolsInstallDir%\bin\HostX64\x64" (
            set "PATH=%PATH%;%VCToolsInstallDir%\bin\HostX64\x64"
            goto set_dll_end
        )
    )
    if exist "%MSVS_VAR_SCRIPT_DIR%\bin\amd64" (
        set "PATH=%PATH%;%MSVS_VAR_SCRIPT_DIR%\bin\amd64"
        goto set_dll_end
    )
)
:set_dll_end

if defined VCToolsInstallDir (
    set "__MS_VC_INSTALL_PATH=%VCToolsInstallDir%"
)

:: setup intel compiler after visual studio environment ready
:: ============================================================================
:SetIntelEnv
if /i "%INTEL_TARGET_ARCH%"=="ia32" (
    set "INTEL_TARGET_ARCH_IA32=ia32"
) else (
    if defined INTEL_TARGET_ARCH_IA32 (set "INTEL_TARGET_ARCH_IA32=")
)

:: There should be only one OpenCL CPU / FGPA emu runtime is loaded.
if defined OCL_ICD_FILENAMES (
    set "OCL_ICD_FILENAMES="
)

:: OpenCL FPGA runtime
if exist "%CMPLR_ROOT%\%INTEL_TARGET_PLATFORM%\lib\oclfpga\fpgavars.bat" (
    call "%CMPLR_ROOT%\%INTEL_TARGET_PLATFORM%\lib\oclfpga\fpgavars.bat"
)

:: ===========================================================================
:: location of this bat file, without a trailing \ (backslash character)
set "ROOT=%~dp0"
set "ROOT=%ROOT:~0,-1%"

echo "using dpcpp compiler %DPCPP_DIR%"
echo "using gfx/driver %GFX_DIR%"

set "PATH=%DPCPP_DIR%\bin;%PATH%"
set "PATH=%DPCPP_DIR%\lib;%PATH%"
set "PATH=%GFX_DIR%;%PATH%"
set "PATH=%GFX_DIR%\Graphics;%PATH%"

set "CPATH=%DPCPP_DIR%\include;%CPATH%"
set "INCLUDE=%DPCPP_DIR%\include;%INCLUDE%"
set "LIB=%DPCPP_DIR%\lib;%LIB%"

goto End

:End
exit /B 0

:: ============================================================================
:NO_VS
echo.
if /i "%*"=="2017 or 2019" (
    echo ERROR: Visual Studio %* is not found in "C:\Program Files (x86)\Microsoft Visual Studio\<2017 or 2019>\<Edition>", please set VS2017INSTALLDIR or VS2019INSTALLDIR
    goto :EOF
)
if /i "%*"=="2019" (
    echo ERROR: Visual Studio %* is not found in "C:\Program Files (x86)\Microsoft Visual Studio\2019\<Edition>", please set VS2019INSTALLDIR
    goto :EOF
)
if /i "%*"=="2017" (
    echo ERROR: Visual Studio %* is not found in "C:\Program Files (x86)\Microsoft Visual Studio\2019\<Edition>", please set VS2017INSTALLDIR
    goto :EOF
)
:EndWithError
exit /B 1

:: ============================================================================
:GetFullPath
SET %2=%~f1
GOTO :EOF

:SetVS2019INSTALLDIR
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional" (
    set "VS2019INSTALLDIR=C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional"
    goto :EOF
)
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise" (
    set "VS2019INSTALLDIR=C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise"
    goto :EOF
)
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community" (
    set "VS2019INSTALLDIR=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community"
    goto :EOF
)
goto :EOF

:SetVS2017INSTALLDIR
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional" (
    set "VS2017INSTALLDIR=C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional"
    goto :EOF
)
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise" (
    set "VS2017INSTALLDIR=C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise"
    goto :EOF
)
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community" (
    set "VS2017INSTALLDIR=C:\Program Files (x86)\Microsoft Visual Studio\2017\Community"
    goto :EOF
)
goto :EOF
