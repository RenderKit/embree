// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/scene_device.h"

#include "meshoptimizer.h"
#include <OpenImageDenoise/oidn.h>

namespace embree {

extern "C" int g_spp;
extern "C" int g_max_path_length;
extern "C" ISPCScene* g_ispc_scene;

  enum RenderMode {
    RENDER_PRIMARY            = 0,
    RENDER_DEBUG_GRIDS        = 1,
    RENDER_DEBUG_SUBGRIDS     = 2,    
    RENDER_DEBUG_QUADS        = 3,
    RENDER_DEBUG_LOD          = 4,
    RENDER_DEBUG_CRACK_FIXING = 5,
    RENDER_DEBUG_CLOD         = 6,
    RENDER_DEBUG_TEXTURE      = 7,        
    RENDER_DEBUG_CLUSTER_ID   = 8,
    RENDER_DEBUG_LOD_LEVEL    = 9,                
    RENDER_PATH_TRACER        = 10,            
    RENDER_PATH_TRACER_DENOISE = 11,            
    
  };

struct Denoiser
{
  OIDNDevice device;
  OIDNFilter filter;
  Vec3f *colorBuffer;
  Vec3f *normalBuffer;  
  Vec3f *outputBuffer;
  uint width, height;

  void checkError()
  {
    const char *errorMessage;    
    if (oidnGetDeviceError(device, &errorMessage) != OIDN_ERROR_NONE) {
      std::cerr << "OIDN error: " << errorMessage << std::endl;
      FATAL("OIDN ERROR");
    }
   
  }
  
  Denoiser(const uint width, const uint height) : width(width), height(height)
  {    
    std::cout << "Init Denoiser...";
    device = oidnNewDevice(OIDN_DEVICE_TYPE_SYCL);
    checkError();
    oidnCommitDevice(device);
    checkError();    
    filter = oidnNewFilter(device, "RT"); // generic ray tracing filter      
    checkError();

#if 0
    colorBuffer  = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,EmbreeUSMMode::EMBREE_USM_SHARED); //DEVICE_READ_WRITE);
    normalBuffer = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,EmbreeUSMMode::EMBREE_USM_SHARED); //DEVICE_READ_WRITE);
    outputBuffer = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,EmbreeUSMMode::EMBREE_USM_SHARED); //DEVICE_READ_WRITE);
#else
    colorBuffer  = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE);
    normalBuffer = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE);
    outputBuffer = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE);    
#endif    
    oidnSetSharedFilterImage(filter, "color",  colorBuffer,
                             OIDN_FORMAT_FLOAT3, width, height, 0, 0, 0); // beauty
    checkError();

    // oidnSetSharedFilterImage(filter, "normal",  normalBuffer,
    //                          OIDN_FORMAT_FLOAT3, width, height, 0, 0, 0); // normal

    checkError();
    
    oidnSetSharedFilterImage(filter, "output",  outputBuffer,
                       OIDN_FORMAT_FLOAT3, width, height, 0, 0, 0); // denoised beauty
    checkError();    
    oidnSetFilter1b(filter, "ldr", true); // beauty image is HDR
    checkError();

    oidnCommitFilter(filter);
    checkError();
    
    std::cout << "done" << std::endl;
  }

  __forceinline void execute()
  {
    oidnExecuteFilter(filter);
    checkError();    
  }

  ~Denoiser()
  {
    oidnReleaseFilter(filter);    
    oidnReleaseDevice(device);    
  }
};
  
struct TutorialData
{
  /* scene data */
  RTCScene g_scene;
  int spp;
  int max_path_length;
  int accu_count;
  ISPCScene* ispc_scene;
};

#if __SYCL_COMPILER_VERSION >= 20210801
}
namespace sycl {
  template<> struct is_device_copyable<embree::TutorialData> : std::true_type {};
  template<> struct is_device_copyable<const embree::TutorialData> : std::true_type {};
}
namespace embree {
#endif

inline void TutorialData_Constructor(TutorialData* This)
{
  This->ispc_scene = g_ispc_scene;  
  This->g_scene  = nullptr;
  This->spp = g_spp;
  This->max_path_length = g_max_path_length;
  This->accu_count = 0;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
}

} // namespace embree
