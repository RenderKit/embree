#!/bin/bash

for scene in ~/models/embree/instancing/instancing_tree.ecs  ~/models/embree/instancing/instancing_barbarian.ecs ~/models/embree/instancing/instancing_barbarian_mblur.ecs ~/models/embree/instancing/instancing002.ecs ~/models/embree/instancing/instancing003.ecs ~/models/rivl/xfrog.ecs ~/models/embree/instancing/instancing000.ecs ~/models/embree/instancing/instancing001.ecs;
do
  echo $scene
  ./pathtracer -c $scene -instancing geometry -benchmark 4 4 $* -rtcore instancing_open_factor=1 > scene_inst.log
  FPS=`sed scene_inst.log -n -e "s/BENCHMARK_RENDER \(.*\)/\1/p"`
  echo "  geom_instancing_ref: FPS=$FPS"

  ./pathtracer -c $scene -instancing geometry -benchmark 4 4 $* > geom_inst.log
  FPS=`sed geom_inst.log -n -e "s/BENCHMARK_RENDER \(.*\)/\1/p"`
  PRIMS=`sed geom_inst.log -n -e "s/BENCHMARK_INSTANCED_PRIMITIVES \(.*\)/\1/p"`
  INSTS=`sed geom_inst.log -n -e "s/[\[\.]*BENCHMARK_INSTANCES \(.*\)/\1/p"`
  OINSTS=`sed geom_inst.log -n -e "s/BENCHMARK_OPENED_INSTANCES \(.*\)/\1/p"`
  echo "  geometry_instancing: FPS=$FPS, INSTS=$INSTS, OINSTS=$OINSTS, PRIMS=$PRIMS" 
done
