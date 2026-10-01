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

#include <iostream>
#include <cstdio>
#include <cassert>
#include <mutex>
#include <string.h>

bool ZeWrapper::rtas_builder_selected = false;

static std::mutex zeWrapperMutex;
static void* handle = nullptr;

static decltype(zeMemFree)* zeMemFreeInternal = nullptr;
static decltype(zeMemAllocHost)* zeMemAllocHostInternal = nullptr;
static decltype(zeMemAllocDevice)* zeMemAllocDeviceInternal = nullptr;
static decltype(zeMemAllocShared)* zeMemAllocSharedInternal = nullptr;
static decltype(zeDriverGetExtensionProperties)* zeDriverGetExtensionPropertiesInternal = nullptr;
static decltype(zeDeviceGetProperties)* zeDeviceGetPropertiesInternal = nullptr;
static decltype(zeDeviceGetModuleProperties)* zeDeviceGetModulePropertiesInternal = nullptr;

static decltype(zeRTASBuilderCreateExt)* zeRTASBuilderCreateExtInternal = nullptr;
static decltype(zeRTASBuilderDestroyExt)* zeRTASBuilderDestroyExtInternal = nullptr;
static decltype(zeDriverRTASFormatCompatibilityCheckExt)* zeDriverRTASFormatCompatibilityCheckExtInternal = nullptr;
static decltype(zeRTASBuilderGetBuildPropertiesExt)* zeRTASBuilderGetBuildPropertiesExtInternal = nullptr;
static decltype(zeRTASBuilderBuildExt)* zeRTASBuilderBuildExtInternal = nullptr;
  
static decltype(zeRTASParallelOperationCreateExt)* zeRTASParallelOperationCreateExtInternal = nullptr;
static decltype(zeRTASParallelOperationDestroyExt)* zeRTASParallelOperationDestroyExtInternal = nullptr;
static decltype(zeRTASParallelOperationGetPropertiesExt)* zeRTASParallelOperationGetPropertiesExtInternal = nullptr;
static decltype(zeRTASParallelOperationJoinExt)* zeRTASParallelOperationJoinExtInternal = nullptr;

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

ze_result_t selectLevelZeroRTASBuilder(ze_driver_handle_t hDriver)
{
  /* only select rtas builder once! */
  if (ZeWrapper::rtas_builder_selected)
    return ZE_RESULT_SUCCESS;
  
  auto zeRTASBuilderCreateExtTemp = find_symbol<decltype(zeRTASBuilderCreateExt)*>(handle,"zeRTASBuilderCreateExt");
  auto zeRTASBuilderDestroyExtTemp = find_symbol<decltype(zeRTASBuilderDestroyExt)*>(handle,"zeRTASBuilderDestroyExt");
  
  ze_rtas_builder_ext_desc_t builderDesc = { ZE_STRUCTURE_TYPE_RTAS_BUILDER_EXT_DESC, nullptr, ZE_RTAS_BUILDER_EXT_VERSION_CURRENT };
  ze_rtas_builder_ext_handle_t hBuilder = nullptr;
  ze_result_t err = zeRTASBuilderCreateExtTemp(hDriver, &builderDesc, &hBuilder);

  /* when ZE_RESULT_ERROR_DEPENDENCY_UNAVAILABLE is reported extension cannot get loaded */
  if (err == ZE_RESULT_ERROR_DEPENDENCY_UNAVAILABLE)
    return err;

  if (err == ZE_RESULT_SUCCESS)
    zeRTASBuilderDestroyExtTemp(hBuilder);

  zeRTASBuilderCreateExtInternal = zeRTASBuilderCreateExtTemp;
  zeRTASBuilderDestroyExtInternal = zeRTASBuilderDestroyExtTemp;
  
  zeDriverRTASFormatCompatibilityCheckExtInternal = find_symbol<decltype(zeDriverRTASFormatCompatibilityCheckExt)*>(handle,"zeDriverRTASFormatCompatibilityCheckExt");
  zeRTASBuilderGetBuildPropertiesExtInternal = find_symbol<decltype(zeRTASBuilderGetBuildPropertiesExt)*>(handle,"zeRTASBuilderGetBuildPropertiesExt");
  zeRTASBuilderBuildExtInternal = find_symbol<decltype(zeRTASBuilderBuildExt)*>(handle,"zeRTASBuilderBuildExt");
  
  zeRTASParallelOperationCreateExtInternal = find_symbol<decltype(zeRTASParallelOperationCreateExt)*>(handle,"zeRTASParallelOperationCreateExt");
  zeRTASParallelOperationDestroyExtInternal = find_symbol<decltype(zeRTASParallelOperationDestroyExt)*>(handle,"zeRTASParallelOperationDestroyExt");
  zeRTASParallelOperationGetPropertiesExtInternal = find_symbol<decltype(zeRTASParallelOperationGetPropertiesExt)*>(handle,"zeRTASParallelOperationGetPropertiesExt");
  zeRTASParallelOperationJoinExtInternal = find_symbol<decltype(zeRTASParallelOperationJoinExt)*>(handle,"zeRTASParallelOperationJoinExt");
  
  ZeWrapper::rtas_builder_selected = true;
  return ZE_RESULT_SUCCESS;
}

ze_result_t ZeWrapper::init()
{
  std::lock_guard<std::mutex> lock(zeWrapperMutex);
  if (handle)
    return ZE_RESULT_SUCCESS;

  try {
    handle = load_module();
    
    zeMemFreeInternal = find_symbol<decltype(zeMemFree)*>(handle, "zeMemFree");
    zeMemAllocHostInternal = find_symbol<decltype(zeMemAllocHost)*>(handle, "zeMemAllocHost");
    zeMemAllocDeviceInternal = find_symbol<decltype(zeMemAllocDevice)*>(handle, "zeMemAllocDevice");
    zeMemAllocSharedInternal = find_symbol<decltype(zeMemAllocShared)*>(handle, "zeMemAllocShared");
    zeDriverGetExtensionPropertiesInternal = find_symbol<decltype(zeDriverGetExtensionProperties)*>(handle, "zeDriverGetExtensionProperties");
    zeDeviceGetPropertiesInternal = find_symbol<decltype(zeDeviceGetProperties)*>(handle, "zeDeviceGetProperties");
    zeDeviceGetModulePropertiesInternal = find_symbol<decltype(zeDeviceGetModuleProperties)*>(handle, "zeDeviceGetModuleProperties");
  }
  catch (std::exception& e) {
    return ZE_RESULT_ERROR_UNKNOWN;
  }
  return ZE_RESULT_SUCCESS;
}

ze_result_t ZeWrapper::initRTASBuilder(ze_driver_handle_t hDriver)
{
  std::lock_guard<std::mutex> lock(zeWrapperMutex);

  try {
    return selectLevelZeroRTASBuilder(hDriver);
  }
  catch (std::exception& e) {
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

ze_result_t ZeWrapper::zeMemAllocHost(ze_context_handle_t context, const ze_host_mem_alloc_desc_t* desch, size_t s0, size_t s1, void** ptr)
{
  if (!handle || !zeMemAllocHostInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeMemAllocHostInternal(context, desch, s0, s1, ptr);
}

ze_result_t ZeWrapper::zeMemAllocDevice(ze_context_handle_t context, const ze_device_mem_alloc_desc_t* descd, size_t s0, size_t s1, ze_device_handle_t ze_handle, void** ptr)
{
  if (!handle || !zeMemAllocDeviceInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeMemAllocDeviceInternal(context, descd, s0, s1, ze_handle, ptr);
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

ze_result_t validate(ze_rtas_device_ext_properties_t* pProperties)
{
  if (pProperties == nullptr)
    return ZE_RESULT_ERROR_INVALID_NULL_POINTER;
  
  if (pProperties->stype != ZE_STRUCTURE_TYPE_RTAS_DEVICE_EXT_PROPERTIES)
    return ZE_RESULT_ERROR_INVALID_ENUMERATION;
  
  //if (!checkDescChain((zet_base_desc_t_*)pProperties))
  //return ZE_RESULT_ERROR_INVALID_ENUMERATION;
  
  return ZE_RESULT_SUCCESS;
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

ze_result_t ZeWrapper::zeRTASBuilderCreateExt(ze_driver_handle_t hDriver, const ze_rtas_builder_ext_desc_t *pDescriptor, ze_rtas_builder_ext_handle_t *phBuilder)
{
  if (!handle || !zeRTASBuilderCreateExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASBuilderCreateExtInternal(hDriver,pDescriptor,phBuilder);
}

ze_result_t ZeWrapper::zeRTASBuilderDestroyExt(ze_rtas_builder_ext_handle_t hBuilder)
{
  if (!handle || !zeRTASBuilderDestroyExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
    
  return zeRTASBuilderDestroyExtInternal(hBuilder);
}

ze_result_t ZeWrapper::zeDriverRTASFormatCompatibilityCheckExt(ze_driver_handle_t hDriver,
                                                               const ze_rtas_format_ext_t accelFormat,
                                                               const ze_rtas_format_ext_t otherAccelFormat)
{
  if (!handle || !zeDriverRTASFormatCompatibilityCheckExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeDriverRTASFormatCompatibilityCheckExtInternal(hDriver, accelFormat, otherAccelFormat);
}

ze_result_t ZeWrapper::zeRTASBuilderGetBuildPropertiesExt(ze_rtas_builder_ext_handle_t hBuilder,
                                                          const ze_rtas_builder_build_op_ext_desc_t* args,
                                                          ze_rtas_builder_ext_properties_t* pProp)
{
  if (!handle || !zeRTASBuilderGetBuildPropertiesExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
    
  return zeRTASBuilderGetBuildPropertiesExtInternal(hBuilder, args, pProp);
}
  
ze_result_t ZeWrapper::zeRTASBuilderBuildExt(ze_rtas_builder_ext_handle_t hBuilder,
                                             const ze_rtas_builder_build_op_ext_desc_t* args,
                                             void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                             void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                             ze_rtas_parallel_operation_ext_handle_t hParallelOperation,
                                             void *pBuildUserPtr, ze_rtas_aabb_ext_t *pBounds, size_t *pRtasBufferSizeBytes)
{
  if (!handle || !zeRTASBuilderBuildExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASBuilderBuildExtInternal(hBuilder, args, pScratchBuffer, scratchBufferSizeBytes, pRtasBuffer, rtasBufferSizeBytes,
                                       hParallelOperation, pBuildUserPtr, pBounds, pRtasBufferSizeBytes);
}

ze_result_t ZeWrapper::zeRTASParallelOperationCreateExt(ze_driver_handle_t hDriver, ze_rtas_parallel_operation_ext_handle_t* phParallelOperation)
{
  if (!handle || !zeRTASParallelOperationCreateExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");

  return zeRTASParallelOperationCreateExtInternal(hDriver, phParallelOperation);
}

ze_result_t ZeWrapper::zeRTASParallelOperationDestroyExt(ze_rtas_parallel_operation_ext_handle_t hParallelOperation)
{
  if (!handle || !zeRTASParallelOperationDestroyExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASParallelOperationDestroyExtInternal(hParallelOperation);
};

ze_result_t ZeWrapper::zeRTASParallelOperationGetPropertiesExt(ze_rtas_parallel_operation_ext_handle_t hParallelOperation, ze_rtas_parallel_operation_ext_properties_t* pProperties)
{
  if (!handle || !zeRTASParallelOperationGetPropertiesExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASParallelOperationGetPropertiesExtInternal(hParallelOperation, pProperties);
}
 
ze_result_t ZeWrapper::zeRTASParallelOperationJoinExt(ze_rtas_parallel_operation_ext_handle_t hParallelOperation)
{
  if (!handle || !zeRTASParallelOperationJoinExtInternal)
    throw std::runtime_error("ZeWrapper not initialized, call ZeWrapper::init() first.");
  
  return zeRTASParallelOperationJoinExtInternal(hParallelOperation);
}
