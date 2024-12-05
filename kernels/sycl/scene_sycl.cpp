
// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#define RTC_EXPORT_API

#include "../common/default.h"
#include "../common/device.h"
#include "../common/scene.h"
#include "../common/context.h"
#include "../geometry/filter.h"
#include "rthwif_embree.h"

#if defined(EMBREE_SYCL_SUPPORT)
#  include "../sycl/rthwif_embree_builder.h"
#endif

using namespace embree;

#define DBG(x)

RTC_NAMESPACE_BEGIN;

#if defined(EMBREE_SYCL_SUPPORT)

// This function fetches USM shared allocated data of the scene (e.g, all geometries, BVH memory buffer)
// in a GPU kernel to trigger migration of the memory from CPU to GPU.
RTC_API_EXTERN_C bool prefetchUSMSharedOnGPU(RTCScene hscene)
{
  Scene* scene = (Scene*) hscene;
  //RTC_CATCH_BEGIN;
  //RTC_TRACE(prefetchUSMSharedOnGPU);
  //RTC_VERIFY_HANDLE(hscene);
  //RTC_ENTER_DEVICE(hscene);

  if (!(rtcGetSceneFlags(hscene) & RTC_SCENE_FLAG_PREFETCH_USM_SHARED_ON_GPU)) {
    return false;
  }

  if (!rtcGetDeviceProperty((RTCDevice)scene->device, RTC_DEVICE_PROPERTY_SYCL_DEVICE)) {
    return false;
  }

  auto deviceGPU = reinterpret_cast<DeviceGPU*>(scene->device);
  auto deviceSYCL = deviceGPU->getGPUDevice();

  sycl::queue queue(deviceSYCL);

  const unsigned num_workers = 1024;

  // we accumulate some nonsene data to prevent compiler
  // optimizing away the memory fetches in the GPU kernel
  size_t* result = sycl::malloc_shared<size_t>(num_workers, queue);
  
  size_t accelSize = scene->accelBuffer.getHWAccelSize();
  char* accelPtr = scene->accelBuffer.getHWAccel(0);

  // Use num_workers GPU work items to iterate over all USM shared
  // allocations to trigger USM migration from CPU to GPU
  queue.parallel_for(num_workers, [=](sycl::id<1> idx)
  {
    result[idx] = 0;
    {
      const size_t num_iterations = (scene->geometries.size() + num_workers - 1) / num_workers;
      for (size_t j = 0; j < num_iterations; ++j) {
        const size_t offset = idx * num_iterations + j;
        if (offset >= scene->geometries.size())
          return;

        const Geometry* geom = scene->geometries[offset].ptr;
        if (geom == nullptr) {
          continue;
        }

        result[idx] += (size_t)geom->size();

        // for instances and instance arrays, fetch the
        // transformation data to triggger USM migration
        if (geom->getType() == AccelSet::GTY_INSTANCE_CHEAP ||
            geom->getType() == AccelSet::GTY_INSTANCE_EXPENSIVE)
        {
          Instance* inst = (Instance*)geom;
          AffineSpace3ff t = inst->local2world[0];
          result[idx] += (size_t)(cast_f2i(t.l.vx.x) % 32);
        }
        else if (geom->getType() == AccelSet::GTY_INSTANCE_ARRAY)
        {
          InstanceArray* inst = (InstanceArray*)geom;
          for (int i = 0; i < inst->size(); ++i) {
            AffineSpace3fa t = inst->getLocal2World(i);
            result[idx] += (size_t)(cast_f2i(t.l.vx.x) % 32);
          }
        }
      }
    }

    {
      // iterate over BVH memory buffer in steps of 4KB
      // (page size on Intel Data Center Max GPUs)
      const size_t accel_size = accelSize / (1 << 12);
      const size_t num_iterations = (accel_size + num_workers - 1) / num_workers;
      for (size_t j = 0; j < num_iterations; ++j) {
        const size_t offset = (idx * num_iterations + j) * (1 << 12);
        if (offset >= accel_size)
          continue;
        result[idx] += ((size_t)accelPtr[offset]) % 32;
      }
    }
  });
  queue.wait_and_throw();

  size_t sum = 0;
  for (size_t i = 0; i < num_workers; ++i)
    sum += result[i];

  sycl::free(result, queue);

  // not really usefull result, this is just to prevent compiler
  // optimizing away the memory reads in the GPU kernel
  return sum > 0;

  //RTC_CATCH_END2(scene);
}

void Scene::syncWithDevice()
{
  DeviceGPU* gpu_device = dynamic_cast<DeviceGPU*>(device);
  if(!gpu_device) {
    return;
  }

  sycl::queue queue = sycl::queue(gpu_device->getGPUDevice());
  syncWithDevice(queue, nullptr);
  queue.wait_and_throw();
}

void Scene::syncWithDevice(sycl::queue queue, sycl::event* event)
{
  auto startSyncWithDevice = std::chrono::high_resolution_clock::now();
  if(!device->is_gpu()) {
    return;
  }

  // TODO: why is this compiled for __SYCL_DEVICE_ONLY__ ???
#if !defined(__SYCL_DEVICE_ONLY__)
  auto startAccelBufferCommit = std::chrono::high_resolution_clock::now();
  accelBuffer.commit(queue);
  auto durationAccelBufferCommit = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startAccelBufferCommit);
  std::cout << "accelBufferCommit took " << durationAccelBufferCommit.count() << " milliseconds to execute." << std::endl;
#endif

  bool num_geometries_changed = num_geometries_device != geometries.size();
  num_geometries_device = geometries.size();

  if (num_geometries_changed) {
    if (geometries_device) {
      device->free(geometries_device);
    }
    if (geometries_host) {
      device->free(geometries_host);
    }
    if (offsets) {
      device->free(offsets);
    }
    offsets = (size_t*)device->malloc(geometries.size() * sizeof(size_t), 16, EmbreeMemoryType::UNKNOWN);
    geometries_device = (Geometry**)device->malloc(sizeof(Geometry*) * geometries.size(), 16, EmbreeMemoryType::DEVICE);
    geometries_host = (Geometry**)device->malloc(sizeof(Geometry*)*geometries.size(), 16, EmbreeMemoryType::UNKNOWN);
  }

  size_t geometry_data_byte_size_ = 0;
  for (size_t i = 0; i < geometries.size(); ++i) {
    Geometry* geom = geometries[i].ptr;
    const size_t byte_size = geom->getGeometryDataDeviceByteSize();
    offsets[i] = geometry_data_byte_size_;
    geometry_data_byte_size_ += byte_size;
  }

  bool geometry_data_device_byte_size_changed = geometry_data_device_byte_size != geometry_data_byte_size_;
  geometry_data_device_byte_size = geometry_data_byte_size_;

  if (geometry_data_device_byte_size_changed) {
    if (geometries_data_device) {
      device->free(geometries_data_device);
    }
    if (geometries_data_host) {
      device->free(geometries_data_host);
    }
    geometries_data_device = (char*)device->malloc(geometry_data_device_byte_size, 16, EmbreeMemoryType::DEVICE);
    geometries_data_host = (char*)device->malloc(geometry_data_device_byte_size, 16, EmbreeMemoryType::UNKNOWN);
  }

  //TODO:
  for(size_t i = 0; i < geometries.size(); ++i) {
  //parallel_for(geometries.size(), [&] ( const size_t i ) {
    if (geometries[i] && geometries[i]->isEnabled()) {
      geometries[i]->convertToDeviceRepresentation(offsets[i], geometries_data_host, geometries_data_device);
      geometries_host[i] = (Geometry*)(geometries_data_device + offsets[i]);
    }
  }//);

  queue.memcpy(geometries_data_device, geometries_data_host, geometry_data_device_byte_size);
  queue.memcpy(geometries_device, geometries_host, sizeof(Geometry*) * geometries.size());

  if (!scene_device) {
    scene_device = (Scene*) device->malloc(sizeof(Scene), 16, EmbreeMemoryType::DEVICE);
  }
  sycl::event last_event = queue.memcpy(scene_device, (void*)this, sizeof(Scene));
  auto endSyncWithDevice = std::chrono::high_resolution_clock::now();
  auto durationSyncWithDevice = std::chrono::duration_cast<std::chrono::milliseconds>(endSyncWithDevice - startSyncWithDevice);

  std::cout << "Scene sync with device took " << durationSyncWithDevice.count() << " milliseconds to execute." << std::endl;

  if (event)
    *event = last_event;
}

#endif

RTC_NAMESPACE_END;