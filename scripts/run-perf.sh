#!/bin/bash -xe

PROJECT_NAME=Embree
SUITE_NAME=perfCI

echo perfdb token = ${BENNY_SYSTEM_TOKEN}

benny insert code_context "${PROJECT_NAME}" ${GITHUB_WORKSPACE} --save-json code_context.json
benny insert run_context ${BENNY_SYSTEM_TOKEN} ./code_context.json --save-json run_context.json
benny insert suite ${PROJECT_NAME} ${SUITE_NAME}


models_dir=./embree-models/
git lfs install
git clone https://${RENDERKIT_GITHUB_TOKEN}@github.com/intel-sandbox/embree-models.git $models_dir

ecs_files=($(find $models_dir -name "*.ecs"))

# config
resolution="2048 2048"
cpubench="1 8"
gpubench="5 20"

if [ "$1" != "gpu" ]; then
  echo "VIEWER CPU"
  SUBSUITE_NAME=embree_viewer
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_viewer -c $ecs_file --size $resolution --benchmark $cpubench --benchmark_out=result.json
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} result.json
  done

  SUBSUITE_NAME=embree_pathtracer
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  echo "PATHTRACER CPU"
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_pathtracer -c $ecs_file --size $resolution --benchmark $cpubench --benchmark_out=result.json
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} result.json
  done
fi

if [ "$1" != "gpu" ]; then
  echo "VIEWER CPU-ispc"
  SUBSUITE_NAME=embree_viewer_ispc
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_viewer_ispc -c $ecs_file --size $resolution --benchmark $cpubench --benchmark_out=result.json
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} result.json
  done

  echo "PATHTRACER CPU-ispc"
  SUBSUITE_NAME=embree_pathtracer_ispc
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_pathtracer_ispc -c $ecs_file --size $resolution --benchmark $cpubench --benchmark_out=result.json
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} result.json
  done
fi

if [ "$1" != "cpu" ]; then
  echo "VIEWER GPU"
  SUBSUITE_NAME=embree_viewer_sycl
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_viewer_sycl -c $ecs_file --size $resolution --benchmark $gpubench --benchmark_out=result.json
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} result.json
  done

  echo "PATHTRACER GPU"
  SUBSUITE_NAME=embree_pathtracer_sycl
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  for ecs_file in "${ecs_files[@]}"
  do
    ./embree_pathtracer_sycl -c $ecs_file --size $resolution --benchmark $gpubench --benchmark_out=result.json
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} result.json
  done
fi


