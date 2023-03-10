#!/bin/bash -xe

export EMBREE_NO_SPLASH=1

models_dir=./embree-models/
git lfs install
git clone https://${RENDERKIT_GITHUB_TOKEN}@github.com/intel-sandbox/embree-models.git $models_dir

ecs_files=($(find $models_dir -name "*.ecs"))

#config
resolution="2048 2048"
cpubench="3"
gpubench="20 10"

if [ "$1" != "gpu" ]; then
  echo "VIEWER CPU"
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_viewer -c $ecs_file --size $resolution --benchmark $cpubench
  done

  echo "PATHTRACER CPU"
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_pathtracer -c $ecs_file --size $resolution --benchmark $cpubench
  done
fi

if [ "$1" != "gpu" ]; then
  echo "VIEWER CPU-ispc"
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_viewer_ispc -c $ecs_file --size $resolution --benchmark $cpubench
  done

  echo "PATHTRACER CPU-ispc"
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_pathtracer_ispc -c $ecs_file --size $resolution --benchmark $cpubench
  done
fi

if [ "$1" != "cpu" ]; then
  echo "VIEWER GPU"
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_viewer_sycl -c $ecs_file --size $resolution --benchmark $gpubench
  done

  echo "PATHTRACER GPU"
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_pathtracer_sycl -c $ecs_file --size $resolution --benchmark $gpubench
  done
fi


