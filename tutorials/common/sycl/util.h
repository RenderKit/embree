// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../kernels/level_zero/ze_wrapper.h"

namespace embree {
  
  inline void check_raytracing_support(sycl::platform& platform, bool& has_raytracing, bool& has_accel_builder)
  {
    ze_driver_handle_t hDriver;
    try {
      hDriver = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(platform);
    } catch (...) { return; }
    
    uint32_t count = 0;
    std::vector<ze_driver_extension_properties_t> extensions;
    ze_result_t result = ZeWrapper::zeDriverGetExtensionProperties(hDriver,&count,extensions.data());
    
    if (result == ZE_RESULT_SUCCESS)
    {
      extensions.resize(count);
      result = ZeWrapper::zeDriverGetExtensionProperties(hDriver,&count,extensions.data());
      if (result == ZE_RESULT_SUCCESS)
      {
        for (uint32_t i=0; i<extensions.size(); i++)
        {
          //std::cout << extensions[i].name << " version " << extensions[i].version << std::endl;

          if (strncmp("ZE_extension_raytracing",extensions[i].name,sizeof(extensions[i].name)) == 0)
            has_raytracing = true;

          if (has_raytracing && strncmp("ZE_experimental_rtas_builder",extensions[i].name,sizeof(extensions[i].name)) == 0) {
            ze_result_t result_rtas_builder = ZeWrapper::initRTASBuilder(hDriver);
            if (result_rtas_builder == ZE_RESULT_ERROR_DEPENDENCY_UNAVAILABLE) {
              return;
            }
            if (result_rtas_builder != ZE_RESULT_SUCCESS) {
              return;
            }
            has_accel_builder = true;
          }
        }
      }
    }
  }
  
  inline void check_raytracing_support()
  {
    bool has_level_zero_support = false;
    bool has_raytracing_support = false;
    bool has_accel_builder_support = false;

    for (auto platform : sycl::platform::get_platforms())
    {
      try {
        sycl::get_native<sycl::backend::ext_oneapi_level_zero>(platform);
        has_level_zero_support = true;
      } catch (...) { continue; }

      if (ZeWrapper::init() != ZE_RESULT_SUCCESS) {
        throw std::runtime_error("ZeWrapper not successfully initialized. Please make sure that level zero loader (libze_loader.so on Linux, ze_loader.dll on Windows) can be found in your environment.");
      }

      check_raytracing_support(platform, has_raytracing_support, has_accel_builder_support);
    }

    if (!has_level_zero_support) {
      throw std::runtime_error("No level zero capable SYCL platform found. Please install a recent driver. On Linux, make sure that the intel-level-zero-gpu package is installed.");
    }

    if (!has_raytracing_support) {
      throw std::runtime_error("No raytracing capable SYCL platform found. Please install a recent driver. On Linux, make sure that the intel-level-zero-gpu package is installed.");
    }

    if (!has_accel_builder_support) {
      throw std::runtime_error("No driver support for acceleration structure building found. Please install a recent driver. On Linux, make sure that the package intel-level-zero-gpu-raytracing is installed.");
    }
  }

  inline void printDeviceInfo(sycl::device const& device)
  {
    std::cout << " SYCL Device:\n";
    std::cout << "  Name: " << device.get_info<sycl::info::device::name>() << "\n";
    std::cout << "  Platform: " << device.get_platform().get_info<sycl::info::platform::name>() << "\n";
    if (device.is_gpu()) { 
      std::cout << "  Type: GPU\n";
      std::cout << "  Max Work Group Size : " << device.get_info<sycl::info::device::max_work_group_size>() << "\n";
      std::cout << "  Max Compute Units   : " << device.get_info<sycl::info::device::max_compute_units>() << std::endl;
    } else {
      std::cout << "  Type: CPU" << std::endl;
    }

    /* list extensions */
    bool ze_extension_ray_tracing = false;
    bool ze_rtas_builder = false;

    sycl::platform platform = device.get_platform();
    check_raytracing_support(platform, ze_extension_ray_tracing, ze_rtas_builder);

    std::cout << "  raytracing = " << (ze_extension_ray_tracing ? "YES" : "NO") << std::endl;
    std::cout << "  rtas_builder = " << (ze_rtas_builder ? "YES" : "NO") << std::endl;

    std::cout << std::endl;
  }

  inline void printAllSYCLDevices()
  {
    ZeWrapper::init();
    std::vector<sycl::device> compatible_devices;
    std::vector<sycl::device> incompatible_devices;
    std::vector<sycl::platform> platforms = sycl::platform::get_platforms();

    for (auto &platform : platforms) {
      std::vector<sycl::device> devices = platform.get_devices();
      for (auto &device : devices) {
        if (rtcIsSYCLDeviceSupported(device))
          compatible_devices.push_back(device);
        else
          incompatible_devices.push_back(device);
      }
    }
  
    if (compatible_devices.empty() && incompatible_devices.empty()) {
      std::cout << "No SYCL device found." << std::endl;
      std::cout << std::endl;
      return;
    }

    if (compatible_devices.empty()) {
      std::cout << "No Embree compatible SYCL GPU device found." << std::endl;
      std::cout << std::endl;
    } else {
      std::cout << "Embree compatible SYCL " << (compatible_devices.size() > 1 ? "devices:" : "device") << std::endl;
      for (auto & device : compatible_devices)
        printDeviceInfo(device);
      if (compatible_devices.empty())
        std::cout << std::endl;
    }
    if (!incompatible_devices.empty()  ) {
      std::cout << "Embree incompatible SYCL " << (incompatible_devices.size() > 1 ? "devices:" : "device") << std::endl;
      for (auto & device : incompatible_devices)
        printDeviceInfo(device);
      std::cout << std::endl;
    }
  }
}
