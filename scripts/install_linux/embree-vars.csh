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

setenv CPATH "$SCRIPT_PATH/include:${CPATH}"

setenv LIBRARY_PATH "$SCRIPT_PATH/lib:${LIBRARY_PATH}"

setenv LD_LIBRARY_PATH "$SCRIPT_PATH/lib:${LD_LIBRARY_PATH}"

setenv SINK_LD_LIBRARY_PATH "$SCRIPT_PATH/lib:${SINK_LD_LIBRARY_PATH}"
