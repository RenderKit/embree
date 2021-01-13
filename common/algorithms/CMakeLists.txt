## Copyright 2009-2021 Intel Corporation
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

# ideally we would use target_link_libraries to provide the algorithms target
# with the TBB include directory information. Howeve, this is only possible in
# very recent versions of TBB and therefore we use the target property instead
GET_TARGET_PROPERTY(tasking_include_dirs tasking INCLUDE_DIRECTORIES)
if (tasking_include_dirs)
  TARGET_INCLUDE_DIRECTORIES(algorithms PUBLIC "${tasking_include_dirs}")
  GET_TARGET_PROPERTY(algorithms_include_dirs algorithms INCLUDE_DIRECTORIES)
endif()
