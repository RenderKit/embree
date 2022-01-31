#!/bin/bash

## Copyright 2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

git log -1

# environment for benchmark client
source ~/benchmark_client.git/env.sh
source ~/system_token.sh

# benchmark configuration
SOURCE_ROOT=`pwd`
PROJECT_NAME="Embree"

NUMACTL="numactl --physcpubind=+0-27 --"
BENCHMARK="--benchmark 10 0 --benchmark_repetitions 11"
THREADS="--set_affinity 1"

MODEL_DIR="${HOME}/embree-models"

export LD_LIBRARY_PATH=`pwd`/build:${LD_LIBRARY_PATH}

cd build
rm -f *.json

BUILD_SCENES_FILE="../tutorials/models/build.bench"
TRACE_SCENES_FILE="../tutorials/models/trace.bench"

RUN_BUILD_BENCHMARKS=true
RUN_TRACE_BENCHMARKS=true
RUN_TUT_BENCHMARKS=true

################################# PLEASE READ ##################################
#
# Note that suites and subsuites must exist in the database _before_ attempting
# insertion of results. This is intentional! You should think carefully about
# your [suite -> subsuite -> benchmark] hierarchy and definitions. These should
# be stable over time (especially for suites and subsuites) to facilitate
# long-term comparisons.
#
# These can be inserted using the benchmark client, through the "insert suite"
# and "insert subsuite" commands. Ask for help if you have questions.
#
################################# PLEASE READ ###################################

initContext() {
  if [ -z "$HAVE_CONTEXT" ]; then
    HAVE_CONTEXT=1
    benny insert code_context "${PROJECT_NAME}" ${SOURCE_ROOT} --save-json code_context.json
    benny insert run_context ${TOKEN} ./code_context.json --save-json run_context.json
  fi
}

####################
# build benchmarks #
####################

if ${RUN_BUILD_BENCHMARKS}; then
  SUITE_NAME="Build"
  benny insert suite ${PROJECT_NAME} ${SUITE_NAME}

  ## subsuites of build benchmarks
  subsuites="update_dynamic_deformable \
             create_dynamic_dynamic \
             create_static_static \
             create_high_quality_static_static"

  while IFS= read -r line
  do
    IFS=' ' read -r -a array <<< "$line"
    NAME="${array[0]}"
    if [[ $NAME == '#'* ]]; then
      continue
    fi
    SCENE=${MODEL_DIR}/"${array[1]}"

    unset "array[0]"
    unset "array[1]"

    for i in ${subsuites}
    do
      SUBSUITE_NAME=$i
      benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

      initContext

      echo "${NUMACTL} ./buildbench --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} --benchmark_type ${SUBSUITE_NAME} -i ${SCENE} ${array[@]} ${THREADS}"
      ${NUMACTL} ./buildbench --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} --benchmark_type ${SUBSUITE_NAME} -i ${SCENE} ${array[@]} ${THREADS}
      benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json
    done
  done < "${BUILD_SCENES_FILE}"
fi

####################
# trace benchmarks #
####################

if ${RUN_TRACE_BENCHMARKS}; then
  SUITE_NAME="Trace"
  benny insert suite ${PROJECT_NAME} ${SUITE_NAME}

  while IFS= read -r line
  do
    IFS=' ' read -r -a array <<< "$line"
    NAME="${array[0]}"
    if [[ $NAME == '#'* ]]; then
      continue
    fi
    SCENE=${MODEL_DIR}/"${array[1]}"

    unset "array[0]"
    unset "array[1]"

    SUBSUITE_NAME="Viewer"
    benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
    initContext

    echo "${NUMACTL} ./viewer --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} -c ${SCENE} ${array[@]} ${THREADS} --size 256 256"
    ${NUMACTL} ./viewer --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} -c ${SCENE} ${array[@]} ${THREADS}
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json

    SUBSUITE_NAME="Pathtracer"
    benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
    initContext

    echo "${NUMACTL} ./pathtracer --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} -c ${SCENE} ${array[@]} ${THREADS} --size 256 256 --spp 32"
    ${NUMACTL} ./pathtracer --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} -c ${SCENE} ${array[@]} ${THREADS}
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json

  done < "${TRACE_SCENES_FILE}"

  if ${RUN_TUT_BENCHMARKS}; then
    SUBSUITE_NAME="Tutorials"
    benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
    initContext

    tutorials="triangle_geometry \
               grid_geometry \
               curve_geometry \
               displacement_geometry \
               hair_geometry \
               instanced_geometry \
               intersection_filter \
               point_geometry \
               subdivision_geometry \
               user_geometry"

    for i in ${tutorials}
    do
      ${NUMACTL} ./${i} --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} ${THREADS}
      benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json
    done
  fi
fi
