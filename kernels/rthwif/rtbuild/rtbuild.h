// Copyright 2009-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stddef.h>
#include <stdint.h>

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

#if defined(RTHWIF_EXPORT_API)
#  define RTHWIF_API RTHWIF_API_EXPORT
#else
#  define RTHWIF_API RTHWIF_API_IMPORT
#endif

#define ZE_APICALL_

#include <level_zero/ze_wrapper.h>

#if !defined(ZE_RTAS_BUILDER_EXP_NAME)
#undef ZE_APIEXPORT
#define ZE_APIEXPORT RTHWIF_API_EXPORT
#include "ze_rtas.h"
#endif

RTHWIF_API ze_result_t zeRTASBuilderCreateExpImpl(ze_driver_handle_t hDriver, const ze_rtas_builder_exp_desc_t *pDescriptor, ze_rtas_builder_exp_handle_t *phBuilder);

inline ze_result_t zeRTASBuilderCreateExp(ze_driver_handle_t hDriver, const ze_rtas_builder_exp_desc_t *pDescriptor, ze_rtas_builder_exp_handle_t *phBuilder) {
  return zeRTASBuilderCreateExpImpl(hDriver,pDescriptor,phBuilder);
}

RTHWIF_API ze_result_t zeRTASBuilderDestroyExpImpl(ze_rtas_builder_exp_handle_t hBuilder);

inline ze_result_t zeRTASBuilderDestroyExp(ze_rtas_builder_exp_handle_t hBuilder) {
  return zeRTASBuilderDestroyExpImpl(hBuilder);
}


RTHWIF_API ze_result_t zeDeviceGetRTASPropertiesExpImpl( const ze_device_handle_t hDevice, ze_rtas_device_exp_properties_t* pRtasProp );
  
inline ze_result_t zeDeviceGetRTASPropertiesExp( const ze_device_handle_t hDevice, ze_rtas_device_exp_properties_t* pRtasProp ) {
  return zeDeviceGetRTASPropertiesExpImpl( hDevice, pRtasProp);
}

RTHWIF_API ze_result_t zeRTASBuilderDeviceFormatCompatibilityCheckExpImpl( ze_rtas_builder_exp_handle_t hBuilder,
                                                                       const ze_rtas_format_exp_t accelFormat,
                                                                       const ze_rtas_format_exp_t otherAccelFormat);

inline ze_result_t zeRTASBuilderDeviceFormatCompatibilityCheckExp( ze_rtas_builder_exp_handle_t hBuilder,
                                                                       const ze_rtas_format_exp_t accelFormat,
                                                                       const ze_rtas_format_exp_t otherAccelFormat)
{
  return zeRTASBuilderDeviceFormatCompatibilityCheckExpImpl( hBuilder, accelFormat, otherAccelFormat);
}

RTHWIF_API ze_result_t zeRTASBuilderGetBuildPropertiesExpImpl(ze_rtas_builder_exp_handle_t hBuilder,
                                                              const ze_rtas_builder_build_op_exp_desc_t* args,
                                                              ze_rtas_builder_exp_properties_t* pProp);
  
inline ze_result_t zeRTASBuilderGetBuildPropertiesExp(ze_rtas_builder_exp_handle_t hBuilder,
                                                          const ze_rtas_builder_build_op_exp_desc_t* args,
                                                          ze_rtas_builder_exp_properties_t* pProp)
{
  return zeRTASBuilderGetBuildPropertiesExpImpl(hBuilder, args, pProp);
}
  
RTHWIF_API ze_result_t zeRTASBuilderBuildExpImpl(ze_rtas_builder_exp_handle_t hBuilder,
                                                 const ze_rtas_builder_build_op_exp_desc_t* args,
                                                 void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                                 void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                                 ze_rtas_parallel_operation_exp_handle_t hParallelOperation,
                                                 void *pBuildUserPtr, ze_rtas_aabb_exp_t *pBounds, size_t *pRtasBufferSizeBytes);

inline ze_result_t zeRTASBuilderBuildExp(ze_rtas_builder_exp_handle_t hBuilder,
                                         const ze_rtas_builder_build_op_exp_desc_t* args,
                                         void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                         void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                         ze_rtas_parallel_operation_exp_handle_t hParallelOperation,
                                         void *pBuildUserPtr, ze_rtas_aabb_exp_t *pBounds, size_t *pRtasBufferSizeBytes)
{
  return zeRTASBuilderBuildExpImpl(hBuilder, args, pScratchBuffer, scratchBufferSizeBytes, pRtasBuffer, rtasBufferSizeBytes,
                                   hParallelOperation, pBuildUserPtr, pBounds, pRtasBufferSizeBytes);
}

RTHWIF_API ze_result_t zeRTASParallelOperationCreateExpImpl(ze_driver_handle_t hDriver, ze_rtas_parallel_operation_exp_handle_t* phParallelOperation);

inline ze_result_t zeRTASParallelOperationCreateExp(ze_driver_handle_t hDriver, ze_rtas_parallel_operation_exp_handle_t* phParallelOperation)
{
  return zeRTASParallelOperationCreateExpImpl(hDriver, phParallelOperation);

}

RTHWIF_API ze_result_t zeRTASParallelOperationDestroyExpImpl( ze_rtas_parallel_operation_exp_handle_t hParallelOperation );

inline ze_result_t zeRTASParallelOperationDestroyExp( ze_rtas_parallel_operation_exp_handle_t hParallelOperation ) {
  return zeRTASParallelOperationDestroyExpImpl( hParallelOperation );
};

RTHWIF_API ze_result_t zeRTASParallelOperationGetPropertiesExpImpl( ze_rtas_parallel_operation_exp_handle_t hParallelOperation, ze_rtas_parallel_operation_exp_properties_t* pProperties );

inline ze_result_t zeRTASParallelOperationGetPropertiesExp( ze_rtas_parallel_operation_exp_handle_t hParallelOperation, ze_rtas_parallel_operation_exp_properties_t* pProperties )
{
  return zeRTASParallelOperationGetPropertiesExpImpl( hParallelOperation, pProperties );
}
 
RTHWIF_API ze_result_t zeRTASParallelOperationJoinExpImpl( ze_rtas_parallel_operation_exp_handle_t hParallelOperation);

inline ze_result_t zeRTASParallelOperationJoinExp( ze_rtas_parallel_operation_exp_handle_t hParallelOperation)
{
  return zeRTASParallelOperationJoinExpImpl(hParallelOperation);
}

