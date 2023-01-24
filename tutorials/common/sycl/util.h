// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

namespace embree {

  void printDeviceInfo(sycl::device const& device) {
    std::cout << " SYCL Device:\n";
    std::cout << "  Name: " << device.get_info<sycl::info::device::name>() << "\n";
    std::cout << "  Platform: " << device.get_platform().get_info<sycl::info::platform::name>() << "\n";
    if (device.is_gpu()) { 
      std::cout << "  Type: GPU\n";
      std::cout << "  Max Work Group Size : " << device.get_info<sycl::info::device::max_work_group_size>() << "\n";
      std::cout << "  Max Compute Units   : " << device.get_info<sycl::info::device::max_compute_units>();
    } else {
      std::cout << "  Type: CPU";
    }
    std::cout << std::endl;
  }

  void printAllSYCLDevices()
  {
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