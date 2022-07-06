#!/bin/bash -xe

## Copyright 2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

git config --global --add safe.directory /builds/renderkit/embree
git log -1

# environment for benchmark client
git clone "http://oauth2:${GITLAB_API_TOKEN}@vis-gitlab.an.intel.com/renderkit/benchmark_client.git" benchmark_client
pushd `pwd`
cd benchmark_client
./setup_venv.sh
popd
source ./benchmark_client/env.sh

TOKEN="d_p9I_dcykHloQFpP-sVrQ"
SOURCE_ROOT=`pwd`
PROJECT_NAME="TestProject"

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

record_result() {
  ##############################################################################################
  # merge json result files from benchmarks, merge them, and record them in benchmark database #
  ##############################################################################################
  SUBSUITE_NAME=${1}

  SUITE_NAME="Embree-Viewer-GPU"

  benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

  scripts/merge_json.py benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json
  benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json

  SUITE_NAME="Embree-Pathtracer-GPU"

  benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}

  scripts/merge_json.py benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json
  benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json
}

record_tutorial_result() {
  SUITE_NAME="Tutorial"
  SUBSUITE_NAME=${1}

  FILE="benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json"
  echo "check if ${FILE} exists"

  if [ -f "${FILE}" ]; then
    benny insert suite ${PROJECT_NAME} ${SUITE_NAME}
    benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
    benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} ${FILE}
  else
    echo "file ${FILE} does not exist"
  fi
}

record_result "crown"
record_result "powerplant"
record_result "barbarian_mblur"
record_result "landscape"
record_result "curly_hair"

record_tutorial_result "user_geometry"
record_tutorial_result "intersection_filter"