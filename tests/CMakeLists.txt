## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

FILE(GLOB PRIMITIVE_TESTS "${PROJECT_SOURCE_DIR}/tests/primitives/*.xml")
SET(SHADERS eyelight;uv;Ng;primID;occlusion)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
    FOREACH(F ${PRIMITIVE_TESTS})
        FOREACH(S ${SHADERS})
            IF (${EMBREE_SYCL_SUPPORT})
                GET_FILENAME_COMPONENT(FN ${F} NAME_WE)

                IF (${FN} MATCHES ".*mblur.*" OR ${FN} MATCHES ".*motion_blur.*")  # skip mblur models if motion blur is not enabled
                  IF (NOT EMBREE_SYCL_MBLUR)
                    CONTINUE()
                  ENDIF()
                ENDIF()
            ENDIF()

            ADD_EMBREE_PRIMITIVE_TEST(${F} ${S})
        ENDFOREACH()
    ENDFOREACH()
ENDIF()
