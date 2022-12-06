#!/bin/bash

## Copyright 2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0
GPU_BENCHMARK_COMMON="platform:x64 compiler:dpcpp build:Release isa:SSE2 EMBREE_SYCL_SUPPORT:ON sycl:dg2 tasking:TBB2021.5.0 intensity:0 EMBREE_USE_GOOGLE_BENCHMARK:ON"

rm -rf benchmark_results && mkdir benchmark_results

rm -rf build
scripts/test.py configure ${GPU_BENCHMARK_COMMON} QUAD:OFF GRID:OFF SUBDIV:OFF POINT:OFF FILTER_FUNCTION:OFF USERGEOM:OFF CURVE:OFF INSTANCE:ON TRI:ON LARGEGRF:OFF
scripts/test.py build
scripts/test.py 'run-benchmark-gpu-config.sh 0-tri+inst'

rm -rf build
scripts/test.py configure ${GPU_BENCHMARK_COMMON} QUAD:OFF GRID:OFF SUBDIV:OFF POINT:OFF FILTER_FUNCTION:OFF USERGEOM:OFF CURVE:OFF INSTANCE:ON TRI:ON LARGEGRF:OFF
scripts/test.py build
scripts/test.py 'run-benchmark-gpu-config.sh 1-tri+inst+mb'

rm -rf build
scripts/test.py configure ${GPU_BENCHMARK_COMMON} QUAD:OFF GRID:OFF SUBDIV:OFF POINT:ON FILTER_FUNCTION:OFF USERGEOM:OFF CURVE:OFF INSTANCE:ON TRI:ON LARGEGRF:OFF
scripts/test.py build
scripts/test.py 'run-benchmark-gpu-config.sh 2-tri+inst+mb+point'

rm -rf build
scripts/test.py configure ${GPU_BENCHMARK_COMMON} QUAD:OFF GRID:OFF SUBDIV:OFF POINT:OFF FILTER_FUNCTION:OFF USERGEOM:OFF CURVE:ON INSTANCE:ON TRI:ON LARGEGRF:ON
scripts/test.py build
scripts/test.py 'run-benchmark-gpu-config.sh 3-tri+inst+mb+curve'

rm -rf build
scripts/test.py configure ${GPU_BENCHMARK_COMMON} QUAD:OFF GRID:OFF SUBDIV:OFF POINT:OFF FILTER_FUNCTION:ON USERGEOM:ON CURVE:ON INSTANCE:ON TRI:ON LARGEGRF:ON
scripts/test.py build
scripts/test.py 'run-benchmark-gpu-config.sh 4-tri+inst+mb+curve+icf'

rm -rf build
scripts/test.py configure ${GPU_BENCHMARK_COMMON} QUAD:ON GRID:ON SUBDIV:ON POINT:ON FILTER_FUNCTION:ON USERGEOM:ON CURVE:ON INSTANCE:ON TRI:ON LARGEGRF:ON
scripts/test.py build
scripts/test.py 'run-benchmark-gpu-config.sh 5-all'
