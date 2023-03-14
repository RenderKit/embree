## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
#ADD_TUTORIAL(nanite_geometry)
find_package(TBB 4.1 REQUIRED ${EMBREE_TBB_COMPONENT})
FIND_PACKAGE(OpenImageDenoise REQUIRED)

SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} OpenImageDenoise)
ADD_TUTORIAL_SYCL(nanite_geometry simplifier.cpp cluster_processing.cpp pathtracer.cpp stripifier.cpp vcacheoptimizer.cpp)
INCLUDE_DIRECTORIES(${OpenImageDenoise_DIR} ${OpenImageDenoise_DIR}/../../../include)

