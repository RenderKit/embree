## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(algorithms OBJECT
 parallel_for.cpp
 parallel_reduce.cpp
 parallel_prefix_sum.cpp
 parallel_for_for.cpp
 parallel_for_for_prefix_sum.cpp
 parallel_partition.cpp
 parallel_sort.cpp
 parallel_set.cpp
 parallel_map.cpp
 parallel_filter.cpp
)

SET_PROPERTY(TARGET algorithms PROPERTY FOLDER common)
SET_PROPERTY(TARGET algorithms APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
