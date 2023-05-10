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

#if defined(EMBREE_LEVEL_ZERO)
#include <level_zero/ze_api.h>
#else
typedef struct _ze_driver_handle_t *ze_driver_handle_t;
typedef struct _ze_device_handle_t *ze_device_handle_t;
typedef enum _ze_result_t
{
    ZE_RESULT_SUCCESS = 0,                          ///< [Core] success
    ZE_RESULT_ERROR_OUT_OF_HOST_MEMORY = 0x70000002,///< [Core] insufficient host memory to satisfy call
    ZE_RESULT_EXP_ERROR_OPERANDS_INCOMPATIBLE = 0x7ff00004, ///< [Core, Experimental] operands of comparison are not compatible
    ZE_RESULT_ERROR_INVALID_ARGUMENT = 0x78000004,  ///< [Validation] generic error code for invalid arguments
    ZE_RESULT_ERROR_INVALID_NULL_HANDLE = 0x78000005,   ///< [Validation] handle argument is not valid
    ZE_RESULT_ERROR_HANDLE_OBJECT_IN_USE = 0x78000006,  ///< [Validation] object pointed to by handle still in-use by device
    ZE_RESULT_ERROR_INVALID_NULL_POINTER = 0x78000007,  ///< [Validation] pointer argument may not be nullptr
    ZE_RESULT_ERROR_INVALID_ENUMERATION = 0x7800000c,   ///< [Validation] enumerator argument is not valid
    ZE_RESULT_ERROR_UNKNOWN = 0x7ffffffe,           ///< [Core] unknown or internal error
    ZE_RESULT_FORCE_UINT32 = 0x7fffffff
} ze_result_t;
typedef enum _ze_structure_type_t {} ze_structure_type_t;
#ifndef ZE_MAKE_VERSION
#  define ZE_MAKE_VERSION( _major, _minor )  (( _major << 16 )|( _minor & 0x0000ffff))
#endif // ZE_MAKE_VERSION
#ifndef ZE_BIT
#  define ZE_BIT( _i )  ( 1 << _i )
#endif // ZE_BIT
#define ZE_APICALL
#endif

#if !defined(ZE_RTAS_BUILDER_EXP_NAME)
#undef ZE_APIEXPORT
#define ZE_APIEXPORT RTHWIF_API_EXPORT
#include "ze_rtas.h"
#endif

RTHWIF_API ze_result_t ZE_APICALL_ zeDeviceGetRTASPropertiesExp( const ze_device_handle_t hDevice, ze_rtas_device_exp_properties_t* pRtasProp );

