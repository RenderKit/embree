#!/bin/tcsh

pushd . > /dev/null
SCRIPT_PATH="${BASH_SOURCE[0]}";
if ( -l "${SCRIPT_PATH}" ) then
  while( -l "${SCRIPT_PATH}" ) do cd `dirname "$SCRIPT_PATH"`; SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
fi
cd "`dirname "$SCRIPT_PATH"`" > /dev/null
SCRIPT_PATH=`pwd`;
popd > /dev/null

setenv CPATH "$SCRIPT_PATH/include:${CPATH}"

setenv LIBRARY_PATH "$SCRIPT_PATH/lib:${LIBRARY_PATH}"

setenv LD_LIBRARY_PATH "$SCRIPT_PATH/lib:${LD_LIBRARY_PATH}"

setenv SINK_LD_LIBRARY_PATH "$SCRIPT_PATH/lib:${SINK_LD_LIBRARY_PATH}"
