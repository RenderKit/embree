#!/bin/bash -xe

wget -N -q -nv https://github.com/embree/models/releases/download/release/asian_dragon.zip
wget -N -q -nv https://github.com/embree/models/releases/download/release/crown.zip
unzip -u -q asian_dragon.zip
unzip -u -q crown.zip

if [ "$1" != "gpu" ]; then
  echo "VIEWER CPU"
  ./embree_viewer -c crown/crown.ecs --size 2048 2048 --benchmark 10 100 | grep avg
  ./embree_viewer -c asian_dragon/asian_dragon.ecs --size 2048 2048 --benchmark 10 100 | grep avg

  echo "PATHTRACER CPU"
  ./embree_pathtracer -c crown/crown.ecs --size 2048 2048 --benchmark 5 40 | grep avg
  ./embree_pathtracer -c asian_dragon/asian_dragon.ecs --size 2048 2048 --benchmark 5 40 | grep avg
fi

if [ "$1" != "gpu" ]; then
  echo "VIEWER CPU-ispc"
  ./embree_viewer_ispc -c crown/crown.ecs --size 2048 2048 --benchmark 10 100 | grep avg
  ./embree_viewer_ispc -c asian_dragon/asian_dragon.ecs --size 2048 2048 --benchmark 10 100 | grep avg


  echo "PATHTRACER CPU-ispc"
  ./embree_pathtracer_ispc -c crown/crown.ecs --size 2048 2048 --benchmark 5 40 | grep avg
  ./embree_pathtracer_ispc -c asian_dragon/asian_dragon.ecs --size 2048 2048 --benchmark 5 40 | grep avg

fi



if [ "$1" != "cpu" ]; then
  echo "VIEWER GPU"
  ./embree_viewer_sycl -c crown/crown.ecs --size 2048 2048 --benchmark 50 500 | grep avg
  ./embree_viewer_sycl -c asian_dragon/asian_dragon.ecs --size 2048 2048 --benchmark 50 500 | grep avg

  echo "PATHTRACER GPU"
  ./embree_pathtracer_sycl crown/crown.ecs --size 2048 2048 --benchmark 10 100 | grep avg
  ./embree_pathtracer_sycl asian_dragon/asian_dragon.ecs --size 2048 2048 --benchmark 10 100 | grep avg
fi


