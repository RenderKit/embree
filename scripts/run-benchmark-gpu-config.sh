#!/bin/bash

## Copyright 2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

run_benchmark()
{
  SCENE_NAME=${1}
  SCENE_FILE=${2}
  BENCHMARK_CONFIG=${3}
  echo "running ./build/viewer_sycl --benchmark_out=benchmark_results/Embree-Viewer-GPU-${SCENE_NAME}-${CONFIG}.json -c ${SCENE_FILE} ${BENCHMARK_CONFIG} "
  time ./build/viewer_sycl --benchmark_out=benchmark_results/Embree-Viewer-GPU-${SCENE_NAME}-${CONFIG}.json -c ${SCENE_FILE} ${BENCHMARK_CONFIG}
  echo "running ./build/pathtracer_sycl --benchmark_out=benchmark_results/Embree-Pathtracer-GPU-${SCENE_NAME}-${CONFIG}.json -c ${SCENE_FILE} ${BENCHMARK_CONFIG} "
  time ./build/pathtracer_sycl --benchmark_out=benchmark_results/Embree-Pathtracer-GPU-${SCENE_NAME}-${CONFIG}.json ${BENCHMARK} -c ${SCENE_FILE} ${BENCHMARK_CONFIG}
}

run_tutorial_benchmark()
{
  TUT=${1}
  echo "./build/${TUT}_sycl --benchmark_out=benchmark_results/Tutorial-${TUT}.json -size 2048 2048 --benchmark 1 7 --benchmark_repetitions 7 --benchmark_name ${TUT}"
  time  ./build/${TUT}_sycl --benchmark_out=benchmark_results/Tutorial-${TUT}.json -size 2048 2048 --benchmark 1 7 --benchmark_repetitions 7 --benchmark_name ${TUT}
}

CONFIG=${1}

################################################
# run benchmarks and store result in json file #
################################################

run_benchmark "crown"                "${HOME}/embree-models/crown/crown_closeup.ecs"              "-size 2048 2048 --benchmark 1 7 --benchmark_repetitions 7 --benchmark_name ${CONFIG}"
run_benchmark "powerplant"           "${HOME}/embree-models/powerplant/powerplant.ecs"            "-size 2048 2048 --benchmark 1 7 --benchmark_repetitions 7 --benchmark_name ${CONFIG}"
run_benchmark "barbarian_mblur"      "${HOME}/embree-models/barbarian/barbarian_mblur.ecs"        "-size 2048 2048 --benchmark 1 7 --benchmark_repetitions 7 --benchmark_name ${CONFIG}"
run_benchmark "landscape"            "${HOME}/embree-models/xfrog/xfrog_landscape.ecs"            "-size 2048 2048 --benchmark 1 7 --benchmark_repetitions 7 --benchmark_name ${CONFIG}"

if [[ "$CONFIG" == *"curve"* || "$CONFIG" == *"all"* ]]; then
  run_benchmark "curly_hair"         "${HOME}/embree-models/hair/curly.ecs"                       "-size 2048 2048 --benchmark 1 7 --benchmark_repetitions 7 --benchmark_name ${CONFIG} --vp -414.96 43.40 21.78 --vi -70.81 8.0 -0.33"
fi

if [[ "$CONFIG" == *"all"* ]]; then
  run_tutorial_benchmark "user_geometry"
  run_tutorial_benchmark "intersection_filter"
fi
