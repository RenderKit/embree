#!/bin/bash

pushd . > /dev/null
SCRIPT_PATH="${BASH_SOURCE[0]}";
if ([ -h "${SCRIPT_PATH}" ]) then
  while([ -h "${SCRIPT_PATH}" ]) do cd `dirname "$SCRIPT_PATH"`; SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
fi
cd "`dirname "$SCRIPT_PATH"`" > /dev/null
SCRIPT_PATH=`pwd`;
popd > /dev/null

$SCRIPT_PATH/benchmark.py run $1 $2_scene_group ./$2 --instancing scene_group
$SCRIPT_PATH/benchmark.py run $1 $2_scene_geometry ./$2 --instancing scene_geometry
$SCRIPT_PATH/benchmark.py run $1 $2_geometry ./$2 --instancing geometry
$SCRIPT_PATH/benchmark.py run $1 $2_geometry_ref ./$2 --instancing geometry --rtcore instancing_open_factor=1
$SCRIPT_PATH/benchmark.py print $1 $2_scene_group $2_scene_geometry $2_geometry_ref $2_geometry
