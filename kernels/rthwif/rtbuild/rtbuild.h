// Copyright 2009-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ze_api.h"

#if !defined(ZE_RTAS_BUILDER_EXP_NAME)
#include "../../level_zero/ze_rtas.h"
#endif

#if defined(__cplusplus)
#  define RTHWIF_API_EXTERN_C extern "C"
#else
#  define RTHWIF_API_EXTERN_C
#endif

#if defined(_WIN32)
#if defined(EMBREE_RTHWIF_STATIC_LIB)
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C
#else
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C __declspec(dllimport)
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C __declspec(dllexport)
#endif
#else
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C __attribute__ ((visibility ("default")))
#endif

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASBuilderCreateExpImpl(ze_driver_handle_t hDriver, const ze_rtas_builder_exp_desc_t *pDescriptor, ze_rtas_builder_exp_handle_t *phBuilder);

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASBuilderDestroyExpImpl(ze_rtas_builder_exp_handle_t hBuilder);

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeDeviceGetRTASPropertiesExpImpl( const ze_device_handle_t hDevice, ze_rtas_device_exp_properties_t* pRtasProp );

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASBuilderFormatCompatibilityCheckExpImpl( ze_rtas_builder_exp_handle_t hBuilder,
                                                                                       const ze_rtas_format_exp_t accelFormat,
                                                                                       const ze_rtas_format_exp_t otherAccelFormat);
RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASBuilderGetBuildPropertiesExpImpl(ze_rtas_builder_exp_handle_t hBuilder,
                                                                                const ze_rtas_builder_build_op_exp_desc_t* args,
                                                                                ze_rtas_builder_exp_properties_t* pProp);
  
RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASBuilderBuildExpImpl(ze_rtas_builder_exp_handle_t hBuilder,
                                                                   const ze_rtas_builder_build_op_exp_desc_t* args,
                                                                   void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                                                   void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                                                   ze_rtas_parallel_operation_exp_handle_t hParallelOperation,
                                                                   void *pBuildUserPtr, ze_rtas_aabb_exp_t *pBounds, size_t *pRtasBufferSizeBytes);

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASParallelOperationCreateExpImpl(ze_driver_handle_t hDriver, ze_rtas_parallel_operation_exp_handle_t* phParallelOperation);

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASParallelOperationDestroyExpImpl( ze_rtas_parallel_operation_exp_handle_t hParallelOperation );

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASParallelOperationGetPropertiesExpImpl( ze_rtas_parallel_operation_exp_handle_t hParallelOperation, ze_rtas_parallel_operation_exp_properties_t* pProperties );

RTHWIF_API_EXPORT ze_result_t ZE_APICALL zeRTASParallelOperationJoinExpImpl( ze_rtas_parallel_operation_exp_handle_t hParallelOperation);

