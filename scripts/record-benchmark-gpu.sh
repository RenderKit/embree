#!/bin/bash

## Copyright 2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# environment for benchmark client
source ~/benchmark_client.git/env.sh
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

initContext() {
  if [ -z "$HAVE_CONTEXT" ]; then
    HAVE_CONTEXT=1
    benny insert code_context "${PROJECT_NAME}" ${SOURCE_ROOT} --save-json code_context.json
    benny insert run_context ${TOKEN} ./code_context.json --save-json run_context.json
  fi
}

record_result() {
  ##############################################################################################
  # merge json result files from benchmarks, merge them, and record them in benchmark database #
  ##############################################################################################
  SUBSUITE_NAME=${1}

  SUITE_NAME="Embree-Viewer-GPU"
  benny insert suite ${PROJECT_NAME} ${SUITE_NAME}

  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  initContext

  scripts/merge_json.py benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}-${CONFIG_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json
  benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json

  SUITE_NAME="Embree-Pathtracer-GPU"
  benny insert suite ${PROJECT_NAME} ${SUITE_NAME}

  benny insert subsuite ${PROJECT_NAME} ${SUITE_NAME} ${SUBSUITE_NAME}
  initContext

  scripts/merge_json.py benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}-${CONFIG_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json
  benny insert googlebenchmark ./run_context.json ${SUITE_NAME} ${SUBSUITE_NAME} benchmark_results/${SUITE_NAME}-${SUBSUITE_NAME}.json
}

record_result "crown"
record_result "sponza"
record_result "barbarian_instancing"
record_result "barbarian_mblur"
#record_result "landscape"
