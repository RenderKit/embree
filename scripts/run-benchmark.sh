#!/bin/bash -xe

## Copyright 2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

git log -1

# environment for benchmark client
source ~/benchmark_client.git/env.sh
source ~/system_token.sh

# benchmark configuration
SOURCE_ROOT=`pwd`
PROJECT_NAME="TestProject"

NUMACTL="numactl --physcpubind=+0-28 --"
BENCHMARK="--benchmark 10 5"
THREADS="--set_affinity 1"

export LD_LIBRARY_PATH=`pwd`/build:${LD_LIBRARY_PATH}

cd build
rm -f *.json

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

benny insert code_context "${PROJECT_NAME}" ${SOURCE_ROOT} --save-json code_context.json
benny insert run_context ${TOKEN} ./code_context.json --save-json run_context.json


####################
# build benchmarks #
####################

SUITE_NAME="TestSuiteEmbreeBuild"

## subsuites of build benchmarks
subsuites="update_dynamic_deformable \
           update_dynamic_dynamic \
           update_dynamic_static \
           create_dynamic_deformable \
           create_dynamic_dynamic \
           create_dynamic_static \
           create_static_static \
           create_high_quality_static_static"

for i in ${subsuites}
do
  SUBSUITE_NAME=$i
  benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

  ${NUMACTL} ./buildbench --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} --benchmark_mask ${SUBSUITE_NAME} -i ../tutorials/models/build.bench ${THREADS}
  benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json
done

# user thread benchmark
SUBSUITE_NAME="create_user_threads_static_static"
benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

${NUMACTL} ./buildbench --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} --benchmark_mask ${SUBSUITE_NAME} -i ../tutorials/models/build.bench --user_threads 1 ${THREADS}
benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json


####################
# trace benchmarks #
####################

SUITE_NAME="TestSuiteEmbreeTrace"

SUBSUITE_NAME="viewer"
benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

${NUMACTL} ./viewer --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} -i ../tutorials/models/trace.bench ${THREADS}
benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json


SUBSUITE_NAME="pathtracer"
benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

${NUMACTL} ./pathtracer --benchmark_out=results-${SUITE_NAME}-${SUBSUITE_NAME}.json ${BENCHMARK} -i ../tutorials/models/trace.bench ${THREADS}
benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ./results-${SUITE_NAME}-${SUBSUITE_NAME}.json


SUBSUITE_NAME="tutorials"
benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

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