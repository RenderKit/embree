// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "default.h"
#include "state.h"
#include "accel.h"

#if defined(EMBREE_DPCPP_SUPPORT)
#include "../gpu/common.h"
#endif

namespace embree
{
  class BVH4Factory;
  class BVH8Factory;
  class BVHGPUFactory;

  class Device : public State, public MemoryMonitorInterface
  {
    ALIGNED_CLASS_(16);

  public:

    /*! Device construction */
    Device (const char* cfg);

    /*! Device destruction */
    virtual ~Device ();

    /*! prints info about the device */
    void print();

    /*! sets the error code */
    void setDeviceErrorCode(RTCError error);

    /*! returns and clears the error code */
    RTCError getDeviceErrorCode();

    /*! sets the error code */
    static void setThreadErrorCode(RTCError error);

    /*! returns and clears the error code */
    static RTCError getThreadErrorCode();

    /*! processes error codes, do not call directly */
    static void process_error(Device* device, RTCError error, const char* str);

    /*! invokes the memory monitor callback */
    void memoryMonitor(ssize_t bytes, bool post);

    /*! sets the size of the software cache. */
    void setCacheSize(size_t bytes);

    /*! sets a property */
    void setProperty(const RTCDeviceProperty prop, ssize_t val);

    /*! gets a property */
    ssize_t getProperty(const RTCDeviceProperty prop);

  private:

    /*! initializes the tasking system */
    void initTaskingSystem(size_t numThreads);

    /*! shuts down the tasking system */
    void exitTaskingSystem();

    /*! some variables that can be set via rtcSetParameter1i for debugging purposes */
  public:
    static ssize_t debug_int0;
    static ssize_t debug_int1;
    static ssize_t debug_int2;
    static ssize_t debug_int3;

  public:
    std::unique_ptr<BVH4Factory> bvh4_factory;
#if defined(EMBREE_TARGET_SIMD8)
    std::unique_ptr<BVH8Factory> bvh8_factory;
#endif
    std::unique_ptr<BVHGPUFactory> bvh_gpu_factory;
    
#if USE_TASK_ARENA
    std::unique_ptr<tbb::task_arena> arena;
#endif
    
    /* ray streams filter */
    RayStreamFilterFuncs rayStreamFilters;
  };

#if defined(EMBREE_DPCPP_SUPPORT)
     
  class DeviceGPU : public Device
  {
  public:

    DeviceGPU(const char* cfg, void *device, void *queue);
    ~DeviceGPU();

  private:
    cl::sycl::queue   *gpu_queue;
    cl::sycl::device  *gpu_device;
    cl::sycl::context gpu_context;
    
    unsigned int maxWorkGroupSize;
    unsigned int maxComputeUnits;
    
  public:
    inline cl::sycl::queue   &getQueue()   { return *gpu_queue; }
    inline cl::sycl::device  &getDevice()  { return *gpu_device; }        
    inline cl::sycl::context &getContext() { return gpu_context; }    

    inline unsigned int getMaxWorkGroupSize() { return maxWorkGroupSize; }
  };

#endif

}
