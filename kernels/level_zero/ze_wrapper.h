// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ze_api.h"

#if !defined(ZE_RTAS_BUILDER_EXP_NAME)
#include "ze_rtas.h"
#endif

struct ZeWrapper
{
  ~ZeWrapper();

  static ze_result_t init();
  static ze_result_t zeMemFree(ze_context_handle_t, void*);
  static ze_result_t zeMemAllocShared(ze_context_handle_t, const ze_device_mem_alloc_desc_t*, const ze_host_mem_alloc_desc_t*, size_t, size_t, ze_device_handle_t, void**);
  static ze_result_t zeDriverGetExtensionProperties(ze_driver_handle_t, uint32_t*, ze_driver_extension_properties_t*);
  static ze_result_t zeDeviceGetProperties(ze_device_handle_t, ze_device_properties_t*);
  static ze_result_t zeDeviceGetModuleProperties(ze_device_handle_t, ze_device_module_properties_t*);
};
