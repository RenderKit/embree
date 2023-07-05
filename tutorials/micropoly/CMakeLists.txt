## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
## find_package(TBB 4.1 REQUIRED ${EMBREE_TBB_COMPONENT})
FIND_PACKAGE(OpenImageDenoise)

IF (OpenImageDenoise_FOUND)
    ADD_DEFINITIONS(-DENABLE_OIDN)
    SET(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} OpenImageDenoise)
    INCLUDE_DIRECTORIES(${OpenImageDenoise_DIR} ${OpenImageDenoise_DIR}/../../../include)
ENDIF()

ADD_TUTORIAL_SYCL(micropoly simplifier.cpp cluster_processing.cpp pathtracer.cpp stripifier.cpp vcacheoptimizer.cpp lod_selection.cpp)

