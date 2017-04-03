// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

namespace embree
{
  struct SceneInterface;
  struct DeviceInterface
  {
    virtual ~DeviceInterface() {};
    virtual void setParameter1i(const RTCParameter parm, ssize_t val) = 0;
    virtual ssize_t getParameter1i(const RTCParameter parm) = 0;
    virtual RTCError getDeviceErrorCode() = 0;
    virtual void setErrorFunction(RTCErrorFunc fptr) = 0;
    virtual void setErrorFunction(RTCErrorFunc2 fptr, void* uptr) = 0;
    virtual void setMemoryMonitorFunction(RTCMemoryMonitorFunc fptr) = 0;
    virtual void setMemoryMonitorFunction(RTCMemoryMonitorFunc2 fptr, void* uptr) = 0;
    virtual void processError(RTCError error, const char* str) = 0;
    virtual SceneInterface* newScene (RTCSceneFlags flags, RTCAlgorithmFlags aflags) = 0;

    static size_t getMaxNumThreads();
    static size_t getMaxCacheSize();
    static void setCacheSize(DeviceInterface* device, size_t bytes);
    static void setNumThreads(DeviceInterface* device, size_t numThreads);
    static bool unsetNumThreads(DeviceInterface* device);

    /*! some variables that can be set via rtcSetParameter1i for debugging purposes */
  public:
    static ssize_t debug_int0;
    static ssize_t debug_int1;
    static ssize_t debug_int2;
    static ssize_t debug_int3;
  };

namespace isa
{
  class BVH4Factory;
  class BVH8Factory;
  class InstanceFactory;

  class Device : public DeviceInterface, public State, public MemoryMonitorInterface
  {
    ALIGNED_CLASS;

  public:

    /*! Device construction */
    Device (const State& state);

    /*! Device construction */
    Device (const char* cfg, bool singledevice);

    /*! Device destruction */
    virtual ~Device ();

    void init();

    /*! prints info about the device */
    void print();

    virtual SceneInterface* newScene (RTCSceneFlags flags, RTCAlgorithmFlags aflags);

    /*! sets the error code */
    void setDeviceErrorCode(RTCError error);

    /*! returns and clears the error code */
    RTCError getDeviceErrorCode();

    /*! processes error codes, do not call directly */
    virtual void processError(RTCError error, const char* str);

    /*! invokes the memory monitor callback */
    void memoryMonitor(ssize_t bytes, bool post);

    /*! sets the size of the software cache. */
    void setCacheSize(size_t bytes);

    /*! configures some parameter */
    void setParameter1i(const RTCParameter parm, ssize_t val);

    /*! returns some configuration */
    ssize_t getParameter1i(const RTCParameter parm);

  private:

    /*! initializes the tasking system */
    void initTaskingSystem(size_t numThreads);

    /*! shuts down the tasking system */
    void exitTaskingSystem();

  public:
    ErrorHandler errorHandler;

    std::unique_ptr<InstanceFactory> instance_factory;
    std::unique_ptr<BVH4Factory> bvh4_factory;
#if defined(__AVX__)
    std::unique_ptr<BVH8Factory> bvh8_factory;
#endif
    
#if USE_TASK_ARENA
    std::unique_ptr<tbb::task_arena> arena;
#endif
    
    /* ray streams filter */
    RayStreamFilterFuncs rayStreamFilters;

 public:
    void setErrorFunction(RTCErrorFunc fptr) 
    {
      error_function = fptr;
      error_function2 = nullptr;
      error_function_userptr = nullptr;
    }
    
    void setErrorFunction(RTCErrorFunc2 fptr, void* uptr) 
    {
      error_function = nullptr;
      error_function2 = fptr;
      error_function_userptr = uptr;
    }

    RTCErrorFunc error_function;
    RTCErrorFunc2 error_function2;
    void* error_function_userptr;

  public:
    void setMemoryMonitorFunction(RTCMemoryMonitorFunc fptr) 
    {
      memory_monitor_function = fptr;
      memory_monitor_function2 = nullptr;
      memory_monitor_userptr = nullptr;
    }
    
    void setMemoryMonitorFunction(RTCMemoryMonitorFunc2 fptr, void* uptr) 
    {
      memory_monitor_function = nullptr;
      memory_monitor_function2 = fptr;
      memory_monitor_userptr = uptr;
    }
      
    RTCMemoryMonitorFunc memory_monitor_function;
    RTCMemoryMonitorFunc2 memory_monitor_function2;
    void* memory_monitor_userptr;
  };
}
}
