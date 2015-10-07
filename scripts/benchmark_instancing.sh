#!/bin/bash
./pathtracer -instancing scene -benchmark 4 4 $* > scene_inst.log
echo "scene_instancing: "
grep scene_inst.log -e "BENCHMARK_RENDER"

for i in 10000 1000 500 250 100;
do
  ./pathtracer -instancing geometry -rtcore instancing_block_size=$i -benchmark 4 4 $* > geom_inst_$i.log
  echo "geometry_instancing_$i: "
  grep geom_inst_$i.log -e "BENCHMARK_RENDER"
done
