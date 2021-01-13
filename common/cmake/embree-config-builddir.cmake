## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# use default install config
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/embree-config-install.cmake)

# and override path variables to match for build directory
SET(EMBREE_INCLUDE_DIRS @PROJECT_SOURCE_DIR@/include)
SET(EMBREE_LIBRARY @PROJECT_BINARY_DIR@/@EMBREE_LIBRARY_FULLNAME@)
SET(EMBREE_LIBRARIES ${EMBREE_LIBRARY})
