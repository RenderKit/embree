// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

/* detect Linux platform */
#if defined(linux) || defined(__linux__) || defined(__LINUX__)
#  if !defined(__LINUX__)
#     define __LINUX__
#  endif
#endif

#if defined(__LINUX__)
#include <dlfcn.h>
#else
#include <windows.h>
#endif

#include "ze_wrapper.h"

#include "../rthwif/rtbuild/rtbuild.h"

#include <iostream>
#include <cstdio>
#include <cassert>
#include <mutex>
#include <string.h>

namespace {

ZeWrapper zeWrapper;
std::mutex zeWrapperMutex;
void* handle = nullptr;

decltype(zeMemFree)* zeMemFreeInternal = nullptr;
decltype(zeMemAllocShared)* zeMemAllocSharedInternal = nullptr;
decltype(zeDriverGetExtensionProperties)* zeDriverGetExtensionPropertiesInternal = nullptr;
decltype(zeDeviceGetProperties)* zeDeviceGetPropertiesInternal = nullptr;
decltype(zeDeviceGetModuleProperties)* zeDeviceGetModulePropertiesInternal = nullptr;

decltype(zeRTASBuilderCreateExp)* zeRTASBuilderCreateExpInternal = nullptr;
decltype(zeRTASBuilderDestroyExp)* zeRTASBuilderDestroyExpInternal = nullptr;
decltype(zeDriverRTASFormatCompatibilityCheckExp)* zeDriverRTASFormatCompatibilityCheckExpInternal = nullptr;
decltype(zeRTASBuilderGetBuildPropertiesExp)* zeRTASBuilderGetBuildPropertiesExpInternal = nullptr;
decltype(zeRTASBuilderBuildExp)* zeRTASBuilderBuildExpInternal = nullptr;
  
decltype(zeRTASParallelOperationCreateExp)* zeRTASParallelOperationCreateExpInternal = nullptr;
decltype(zeRTASParallelOperationDestroyExp)* zeRTASParallelOperationDestroyExpInternal = nullptr; 
decltype(zeRTASParallelOperationGetPropertiesExp)* zeRTASParallelOperationGetPropertiesExpInternal = nullptr;
decltype(zeRTASParallelOperationJoinExp)* zeRTASParallelOperationJoinExpInternal = nullptr;
}

template<typename T>
T find_symbol(void* handle, std::string const& symbol) {
#if defined(__LINUX__)
  T result = (T) dlsym(handle, symbol.c_str());
#else
  T result = (T) GetProcAddress((HMODULE)handle, symbol.c_str());
#endif
  if (!result) {
    throw std::runtime_error("level_zero wrapper: symbol " + symbol + " not found");
  }
  return result;
}

void* load_module() {
#if defined(__LINUX__)
  void* handle = dlopen(ZE_LOADER_NAME_LINUX,RTLD_LAZY);
  if (!handle) {
    throw std::runtime_error("module " ZE_LOADER_NAME_LINUX " not found");
  }
#else
  void* handle = LoadLibraryExA(ZE_LOADER_NAME_WINDOWS,NULL,LOAD_LIBRARY_SEARCH_SYSTEM32);
  if (!handle) {
    throw std::runtime_error("module " ZE_LOADER_NAME_WINDOWS " not found");
  }
#endif
  return handle;
}

void unload_module(void* handle) {
  if (handle) {
#if defined(__LINUX__)
    dlclose(handle);
#else
    FreeLibrary((HMODULE)handle);
#endif
  }
}

ZeWrapper::~ZeWrapper() {
  unload_module(handle);
}

ze_result_t ZeWrapper::init()
{
  std::lock_guard<std::mutex> lock(zeWrapperMutex);
  if (handle) {
    // already initialized
    return ZE_RESULT_SUCCESS;
  }

  try {
    handle = load_module();
    
    zeMemFreeInternal = find_symbol<decltype(zeMemFree)*>(handle, "zeMemFree");
    zeMemAllocSharedInternal = find_symbol<decltype(zeMemAllocShared)*>(handle, "zeMemAllocShared");
    zeDriverGetExtensionPropertiesInternal = find_symbol<decltype(zeDriverGetExtensionProperties)*>(handle, "zeDriverGetExtensionProperties");
    zeDeviceGetPropertiesInternal = find_symbol<decltype(zeDeviceGetProperties)*>(handle, "zeDeviceGetProperties");
    zeDeviceGetModulePropertiesInternal = find_symbol<decltype(zeDeviceGetModuleProperties)*>(handle, "zeDeviceGetModuleProperties");

    try {
      zeRTASBuilderCreateExpInternal = find_symbol<decltype(zeRTASBuilderCreateExp)*>(handle,"zeRTASBuilderCreateExp");
      zeRTASBuilderDestroyExpInternal = find_symbol<decltype(zeRTASBuilderDestroyExp)*>(handle,"zeRTASBuilderDestroyExp");
      zeDriverRTASFormatCompatibilityCheckExpInternal = find_symbol<decltype(zeDriverRTASFormatCompatibilityCheckExp)*>(handle,"zeDriverRTASFormatCompatibilityCheckExp");
      zeRTASBuilderGetBuildPropertiesExpInternal = find_symbol<decltype(zeRTASBuilderGetBuildPropertiesExp)*>(handle,"zeRTASBuilderGetBuildPropertiesExp");
      zeRTASBuilderBuildExpInternal = find_symbol<decltype(zeRTASBuilderBuildExp)*>(handle,"zeRTASBuilderBuildExp");
      
      zeRTASParallelOperationCreateExpInternal = find_symbol<decltype(zeRTASParallelOperationCreateExp)*>(handle,"zeRTASParallelOperationCreateExp");
      zeRTASParallelOperationDestroyExpInternal = find_symbol<decltype(zeRTASParallelOperationDestroyExp)*>(handle,"zeRTASParallelOperationDestroyExp");
      zeRTASParallelOperationGetPropertiesExpInternal = find_symbol<decltype(zeRTASParallelOperationGetPropertiesExp)*>(handle,"zeRTASParallelOperationGetPropertiesExp");
      zeRTASParallelOperationJoinExpInternal = find_symbol<decltype(zeRTASParallelOperationJoinExp)*>(handle,"zeRTASParallelOperationJoinExp");
      
    } catch (std::exception& e) {

      zeRTASBuilderCreateExpInternal = &zeRTASBuilderCreateExpImpl;
      zeRTASBuilderDestroyExpInternal = &zeRTASBuilderDestroyExpImpl;
      zeDriverRTASFormatCompatibilityCheckExpInternal = &zeDriverRTASFormatCompatibilityCheckExpImpl;
      zeRTASBuilderGetBuildPropertiesExpInternal = &zeRTASBuilderGetBuildPropertiesExpImpl;
      zeRTASBuilderBuildExpInternal = &zeRTASBuilderBuildExpImpl;
      
      zeRTASParallelOperationCreateExpInternal = &zeRTASParallelOperationCreateExpImpl;
      zeRTASParallelOperationDestroyExpInternal = &zeRTASParallelOperationDestroyExpImpl;
      zeRTASParallelOperationGetPropertiesExpInternal = &zeRTASParallelOperationGetPropertiesExpImpl;
      zeRTASParallelOperationJoinExpInternal = &zeRTASParallelOperationJoinExpImpl;
    }
    
  } catch (std::exception& e) {
    std::cerr << "Error: Initializing ZeWrapper failed: " << e.what() << std::endl;
    return ZE_RESULT_ERROR_UNKNOWN;
  }
  return ZE_RESULT_SUCCESS;
}

ze_result_t ZeWrapper::zeMemFree(ze_context_handle_t context, void* ptr)
{
  if (!handle || !zeMemFreeInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeMemFreeInternal(context, ptr);
}

ze_result_t ZeWrapper::zeMemAllocShared(ze_context_handle_t context, const ze_device_mem_alloc_desc_t* descd, const ze_host_mem_alloc_desc_t* desch, size_t s0, size_t s1, ze_device_handle_t ze_handle, void** ptr)
{
  if (!handle || !zeMemAllocSharedInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeMemAllocSharedInternal(context, descd, desch, s0, s1, ze_handle, ptr);
}

ze_result_t ZeWrapper::zeDriverGetExtensionProperties(ze_driver_handle_t ze_handle, uint32_t* ptr, ze_driver_extension_properties_t* props)
{
  if (!handle || !zeDriverGetExtensionPropertiesInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeDriverGetExtensionPropertiesInternal(ze_handle, ptr, props);
}

ze_result_t ZeWrapper::zeDeviceGetProperties(ze_device_handle_t ze_handle, ze_device_properties_t* props)
{
  if (!handle || !zeDeviceGetPropertiesInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeDeviceGetPropertiesInternal(ze_handle, props);
}

ze_result_t ZeWrapper::zeDeviceGetModuleProperties(ze_device_handle_t ze_handle, ze_device_module_properties_t* props)
{
  if (!handle || !zeDeviceGetModulePropertiesInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeDeviceGetModulePropertiesInternal(ze_handle, props);
}

#define VALIDATE(arg) \
  {\
  ze_result_t result = validate(arg);\
  if (result != ZE_RESULT_SUCCESS) return result; \
  }

ze_result_t validate(ze_device_handle_t hDevice)
  {
    if (hDevice == nullptr)
      return ZE_RESULT_ERROR_INVALID_NULL_HANDLE;
    
    return ZE_RESULT_SUCCESS;
  }

ze_result_t validate(ze_rtas_device_exp_properties_t* pProperties)
{ 
  if (pProperties == nullptr)
    return ZE_RESULT_ERROR_INVALID_NULL_POINTER;
  
  if (pProperties->stype != ZE_STRUCTURE_TYPE_RTAS_DEVICE_EXP_PROPERTIES)
    return ZE_RESULT_ERROR_INVALID_ENUMERATION;
  
  //if (!checkDescChain((zet_base_desc_t_*)pProperties))
  //return ZE_RESULT_ERROR_INVALID_ENUMERATION;
  
  return ZE_RESULT_SUCCESS;
}

typedef enum _ze_raytracing_accel_format_internal_t {
  ZE_RTAS_DEVICE_FORMAT_EXP_INVALID = 0,      // invalid acceleration structure format
  ZE_RTAS_DEVICE_FORMAT_EXP_VERSION_1 = 1, // acceleration structure format version 1
  ZE_RTAS_DEVICE_FORMAT_EXP_VERSION_2 = 2, // acceleration structure format version 2
  ZE_RTAS_DEVICE_FORMAT_EXP_VERSION_MAX = 2
} ze_raytracing_accel_format_internal_t;

ze_result_t ZeWrapper::zeDeviceGetRTASPropertiesExp( const ze_device_handle_t hDevice, ze_rtas_device_exp_properties_t* pProperties )
{
  /* input validation */
  VALIDATE(hDevice);
  VALIDATE(pProperties);
  
  /* fill properties */
  pProperties->flags = 0;
  pProperties->rtasFormat = (ze_rtas_format_exp_t) ZE_RTAS_DEVICE_FORMAT_EXP_INVALID;
  pProperties->rtasBufferAlignment = 128;
  
  /* check for supported device ID */
  ze_device_properties_t device_props{ ZE_STRUCTURE_TYPE_DEVICE_PROPERTIES };
  ze_result_t status = ZeWrapper::zeDeviceGetProperties(hDevice, &device_props);
  if (status != ZE_RESULT_SUCCESS)
    return status;
  
  /* check for Intel vendor */
  const uint32_t vendor_id = device_props.vendorId;
  const uint32_t device_id = device_props.deviceId;
  if (vendor_id != 0x8086) return ZE_RESULT_ERROR_UNKNOWN;
  
  /* disabling of device check through env variable */
  const char* disable_device_check = getenv("EMBREE_DISABLE_DEVICEID_CHECK");
  if (disable_device_check && strcmp(disable_device_check,"1") == 0) {
    pProperties->rtasFormat = (ze_rtas_format_exp_t) ZE_RTAS_DEVICE_FORMAT_EXP_VERSION_1;
    return ZE_RESULT_SUCCESS;
  }
  
  /* DG2 */
  const bool dg2 =
    (0x4F80 <= device_id && device_id <= 0x4F88) ||
    (0x5690 <= device_id && device_id <= 0x5698) ||
    (0x56A0 <= device_id && device_id <= 0x56A6) ||
    (0x56B0 <= device_id && device_id <= 0x56B3) ||
    (0x56C0 <= device_id && device_id <= 0x56C1);
  
  if (dg2) {
    pProperties->rtasFormat = (ze_rtas_format_exp_t) ZE_RTAS_DEVICE_FORMAT_EXP_VERSION_1;
    return ZE_RESULT_SUCCESS;
  }
  
  /* PVC */
  const bool pvc =
    (0x0BD0 <= device_id && device_id <= 0x0BDB) ||
    (device_id == 0x0BE5                       );
  
  if (pvc) {
    pProperties->rtasFormat = (ze_rtas_format_exp_t) ZE_RTAS_DEVICE_FORMAT_EXP_VERSION_1;
    return ZE_RESULT_SUCCESS;
  }
  
  /* MTL */
  const bool mtl =
    (device_id == 0x7D40) ||
    (device_id == 0x7D55) ||
    (device_id == 0x7DD5) ||
    (device_id == 0x7D45) ||
    (device_id == 0x7D60);
  
  if (mtl) {
    pProperties->rtasFormat = (ze_rtas_format_exp_t) ZE_RTAS_DEVICE_FORMAT_EXP_VERSION_1;
    return ZE_RESULT_SUCCESS;
  }
  
  return ZE_RESULT_ERROR_UNKNOWN;
}

ze_result_t ZeWrapper::zeRTASBuilderCreateExp(ze_driver_handle_t hDriver, const ze_rtas_builder_exp_desc_t *pDescriptor, ze_rtas_builder_exp_handle_t *phBuilder)
{
  if (!handle || !zeRTASBuilderCreateExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASBuilderCreateExpInternal(hDriver,pDescriptor,phBuilder);
}

ze_result_t ZeWrapper::zeRTASBuilderDestroyExp(ze_rtas_builder_exp_handle_t hBuilder)
{
  if (!handle || !zeRTASBuilderDestroyExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
    
  return zeRTASBuilderDestroyExpInternal(hBuilder);
}

ze_result_t ZeWrapper::zeDriverRTASFormatCompatibilityCheckExp( ze_driver_handle_t hDriver,
                                                                 const ze_rtas_format_exp_t accelFormat,
                                                                 const ze_rtas_format_exp_t otherAccelFormat)
{
  if (!handle || !zeDriverRTASFormatCompatibilityCheckExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeDriverRTASFormatCompatibilityCheckExpInternal( hDriver, accelFormat, otherAccelFormat);
}

ze_result_t ZeWrapper::zeRTASBuilderGetBuildPropertiesExp(ze_rtas_builder_exp_handle_t hBuilder,
                                                          const ze_rtas_builder_build_op_exp_desc_t* args,
                                                          ze_rtas_builder_exp_properties_t* pProp)
{
  if (!handle || !zeRTASBuilderGetBuildPropertiesExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
    
  return zeRTASBuilderGetBuildPropertiesExpInternal(hBuilder, args, pProp);
}
  
ze_result_t ZeWrapper::zeRTASBuilderBuildExp(ze_rtas_builder_exp_handle_t hBuilder,
                                             const ze_rtas_builder_build_op_exp_desc_t* args,
                                             void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                             void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                             ze_rtas_parallel_operation_exp_handle_t hParallelOperation,
                                             void *pBuildUserPtr, ze_rtas_aabb_exp_t *pBounds, size_t *pRtasBufferSizeBytes)
{
  if (!handle || !zeRTASBuilderBuildExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASBuilderBuildExpInternal(hBuilder, args, pScratchBuffer, scratchBufferSizeBytes, pRtasBuffer, rtasBufferSizeBytes,
                                       hParallelOperation, pBuildUserPtr, pBounds, pRtasBufferSizeBytes);
}

ze_result_t ZeWrapper::zeRTASParallelOperationCreateExp(ze_driver_handle_t hDriver, ze_rtas_parallel_operation_exp_handle_t* phParallelOperation)
{
  if (!handle || !zeRTASParallelOperationCreateExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASParallelOperationCreateExpInternal(hDriver, phParallelOperation);
}

ze_result_t ZeWrapper::zeRTASParallelOperationDestroyExp( ze_rtas_parallel_operation_exp_handle_t hParallelOperation )
{
  if (!handle || !zeRTASParallelOperationDestroyExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASParallelOperationDestroyExpInternal( hParallelOperation );
};

ze_result_t ZeWrapper::zeRTASParallelOperationGetPropertiesExp( ze_rtas_parallel_operation_exp_handle_t hParallelOperation, ze_rtas_parallel_operation_exp_properties_t* pProperties )
{
  if (!handle || !zeRTASParallelOperationGetPropertiesExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASParallelOperationGetPropertiesExpInternal( hParallelOperation, pProperties );
}
 
ze_result_t ZeWrapper::zeRTASParallelOperationJoinExp( ze_rtas_parallel_operation_exp_handle_t hParallelOperation)
{
  if (!handle || !zeRTASParallelOperationJoinExpInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASParallelOperationJoinExpInternal(hParallelOperation);
}
