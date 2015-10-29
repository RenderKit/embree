#!/bin/tcsh
pushd . > /dev/null
set SCRIPT_PATH=($_)
set SCRIPT_PATH="$SCRIPT_PATH[2]"
if ( -l "${SCRIPT_PATH}" ) then
  while( -l "${SCRIPT_PATH}" ) do cd `dirname "$SCRIPT_PATH"`; SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
endif
cd "`dirname "$SCRIPT_PATH"`" > /dev/null
set SCRIPT_PATH=`pwd`
popd > /dev/null

if (!($?CPATH)) then
  setenv CPATH
endif

if (!($?LIBRARY_PATCH)) then
  setenv LIBRARY_PATH
endif

if (!($?DYLD_LIBRARY_PATH)) then
  setenv DYLD_LIBRARY_PATH
endif

setenv CPATH "$SCRIPT_PATH/@CMAKE_INSTALL_INCLUDEDIR@":${CPATH}

setenv LIBRARY_PATH "$SCRIPT_PATH/@CMAKE_INSTALL_LIBDIR@":${LIBRARY_PATH}

setenv DYLD_LIBRARY_PATH "$SCRIPT_PATH/@CMAKE_INSTALL_LIBDIR@":${DYLD_LIBRARY_PATH}

