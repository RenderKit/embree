#!/bin/bash

#./pathtracer --instancing none --rtcore accel=bvh4.triangle4 --benchmark 4 4 $* > scene_noinst.log
#FPS=`sed scene_noinst.log -n -e "s/BENCHMARK_RENDER \(.*\)/\1/p"`
#echo "scene_noinstancing: FPS=$FPS"

./pathtracer --instancing scene_geometry --benchmark 4 4 $* > scene_geometry.log
FPS=`sed scene_geometry.log -n -e "s/BENCHMARK_RENDER_AVG \(.*\)/\1/p"`
echo "scene_geometry: FPS=$FPS"

./pathtracer --instancing scene_group --benchmark 4 4 $* > scene_group.log
FPS=`sed scene_group.log -n -e "s/BENCHMARK_RENDER_AVG \(.*\)/\1/p"`
echo "scene_group: FPS=$FPS"

for i in 1 2 4 8 12;
do
  ./pathtracer --instancing geometry --rtcore instancing_open_factor=$i --benchmark 4 4 $* > geom_inst_$i.log
  FPS=`sed geom_inst_$i.log -n -e "s/BENCHMARK_RENDER_AVG \(.*\)/\1/p"`
  PRIMS=`sed geom_inst_$i.log -n -e "s/BENCHMARK_INSTANCED_PRIMITIVES \(.*\)/\1/p"`
  INSTS=`sed geom_inst_$i.log -n -e "s/[\[\.]*BENCHMARK_INSTANCES \(.*\)/\1/p"`
  OINSTS=`sed geom_inst_$i.log -n -e "s/BENCHMARK_OPENED_INSTANCES \(.*\)/\1/p"`
  echo "geometry_instancing_$i: FPS=$FPS, INSTS=$INSTS, OINSTS=$OINSTS, PRIMS=$PRIMS" 
done
