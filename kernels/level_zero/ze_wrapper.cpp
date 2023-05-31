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

typedef decltype(zeMemFree)* TyZeMemFree;
typedef decltype(zeMemAllocShared)* TyZeMemAllocShared;
typedef decltype(zeDriverGetExtensionProperties)* TyZeDriverGetExtensionProperties;
typedef decltype(zeDeviceGetProperties)* TyZeDeviceGetProperties;
typedef decltype(zeDeviceGetModuleProperties)* TyZeDeviceGetModuleProperties;

namespace {

ZeWrapper zeWrapper;
std::mutex zeWrapperMutex;
void* handle = nullptr;
TyZeMemFree zeMemFreeInternal = nullptr;
TyZeMemAllocShared zeMemAllocSharedInternal = nullptr;
TyZeDriverGetExtensionProperties zeDriverGetExtensionPropertiesInternal = nullptr;
TyZeDeviceGetProperties zeDeviceGetPropertiesInternal = nullptr;
TyZeDeviceGetModuleProperties zeDeviceGetModulePropertiesInternal = nullptr;

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
    zeMemFreeInternal = find_symbol<TyZeMemFree>(handle, "zeMemFree");
    zeMemAllocSharedInternal = find_symbol<TyZeMemAllocShared>(handle, "zeMemAllocShared");
    zeDriverGetExtensionPropertiesInternal = find_symbol<TyZeDriverGetExtensionProperties>(handle, "zeDriverGetExtensionProperties");
    zeDeviceGetPropertiesInternal = find_symbol<TyZeDeviceGetProperties>(handle, "zeDeviceGetProperties");
    zeDeviceGetModulePropertiesInternal = find_symbol<TyZeDeviceGetModuleProperties>(handle, "zeDeviceGetModuleProperties");
  } catch(std::exception& e) {
    std::cerr << "Error: Initializing ZeWrapper failed: " << e.what() << std::endl;
    return ZE_RESULT_ERROR_UNKNOWN;
  }
  return ZE_RESULT_SUCCESS;
}

ze_result_t ZeWrapper::zeMemFree(ze_context_handle_t context, void* ptr)
{
  assert(handle && zeMemFreeInternal && "ZeWrapper not initialized, call ZeWrapper::init() first.");
  return zeMemFreeInternal(context, ptr);
}

ze_result_t ZeWrapper::zeMemAllocShared(ze_context_handle_t context, const ze_device_mem_alloc_desc_t* descd, const ze_host_mem_alloc_desc_t* desch, size_t s0, size_t s1, ze_device_handle_t ze_handle, void** ptr)
{
  assert(handle && zeMemAllocSharedInternal && "ZeWrapper not initialized, call ZeWrapper::init() first.");
  return zeMemAllocSharedInternal(context, descd, desch, s0, s1, ze_handle, ptr);
}

ze_result_t ZeWrapper::zeDriverGetExtensionProperties(ze_driver_handle_t ze_handle, uint32_t* ptr, ze_driver_extension_properties_t* props)
{
  assert(handle && zeDriverGetExtensionPropertiesInternal && "ZeWrapper not initialized, call ZeWrapper::init() first.");
  return zeDriverGetExtensionPropertiesInternal(ze_handle, ptr, props);
}

ze_result_t ZeWrapper::zeDeviceGetProperties(ze_device_handle_t ze_handle, ze_device_properties_t* props)
{
  assert(handle && zeDeviceGetPropertiesInternal && "ZeWrapper not initialized, call ZeWrapper::init() first.");
  return zeDeviceGetPropertiesInternal(ze_handle, props);
}

ze_result_t ZeWrapper::zeDeviceGetModuleProperties(ze_device_handle_t ze_handle, ze_device_module_properties_t* props)
{
  assert(handle && zeDeviceGetModulePropertiesInternal && "ZeWrapper not initialized, call ZeWrapper::init() first.");
  return zeDeviceGetModulePropertiesInternal(ze_handle, props);
}
