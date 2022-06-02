#!/bin/bash

## Copyright 2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

CONFIG=${1}
BENCHMARK="--benchmark 10 0 --benchmark_repetitions 1 --benchmark_name ${CONFIG}"

run_benchmark() {

  SCENE_NAME=${1}
  SCENE_FILE=${2}
  echo "running ./build/viewer_sycl --benchmark_out=benchmark_results/Embree-Viewer-GPU-${SCENE_NAME}-${CONFIG}.json ${BENCHMARK} -c ${SCENE_FILE}"
  ./build/viewer_sycl --benchmark_out=benchmark_results/Embree-Viewer-GPU-${SCENE_NAME}-${CONFIG}.json ${BENCHMARK} -c ${SCENE_FILE}
  echo "running ./build/pathtracer_sycl --benchmark_out=benchmark_results/Embree-Pathtracer-GPU-${SCENE_NAME}-${CONFIG}.json ${BENCHMARK} -c ${SCENE_FILE}"
  ./build/pathtracer_sycl --benchmark_out=benchmark_results/Embree-Pathtracer-GPU-${SCENE_NAME}-${CONFIG}.json ${BENCHMARK} -c ${SCENE_FILE}
}

################################################
# run benchmarks and store result in json file #
################################################

run_benchmark "crown"                "${HOME}/embree-models/crown/crown_closeup.ecs"
run_benchmark "sponza"               "${HOME}/embree-models/sponza/sponza.ecs"
run_benchmark "barbarian_instancing" "${HOME}/embree-models/barbarian/barbarian_instancing.ecs"
run_benchmark "barbarian_mblur"      "${HOME}/embree-models/barbarian/barbarian_mblur.ecs"
#run_benchmark "landscape"            "${HOME}/embree-models/xfrog/xfrog_landscape.ecs"