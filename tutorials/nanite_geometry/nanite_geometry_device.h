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

extern "C" uint user_spp;
extern "C" uint g_max_path_length;
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


#define ENABLE_FP16_GBUFFER 0

#if ENABLE_FP16_GBUFFER == 1
  typedef sycl::vec<cl::sycl::cl_half, 3>  Vec3fp16;

  __forceinline Vec3f    fp_convert(const Vec3fp16 &v) { return Vec3f((float)v.x(),(float)v.y(),(float)v.z()); }
  __forceinline Vec3fp16 fp_convert(const Vec3f    &v) { return Vec3fp16(v.x,v.y,v.z);  }

#else
  __forceinline Vec3f    fp_convert(const Vec3f    &v) { return v; }  
#endif


struct GBuffer
{
#if ENABLE_FP16_GBUFFER == 1
  Vec3fp16 color,normal,albedo;
#else
  Vec3f color;
  Vec3f normal;
  Vec3f albedo;
#endif  
};

#if ENABLE_FP16_GBUFFER == 1
  typedef Vec3fp16 GBufferOutput;
#else
  typedef Vec3f GBufferOutput;
#endif
  
struct Denoiser
{
  OIDNDevice device;
  OIDNFilter filter;
  GBuffer *gBuffer;
#if ENABLE_FP16_GBUFFER == 1
  Vec3fp16   *outputBuffer;  
#else  
  Vec3f   *outputBuffer;
#endif  
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

#if ENABLE_FP16_GBUFFER == 0
    OIDNFormat format = OIDN_FORMAT_FLOAT3;
#else
    OIDNFormat format = OIDN_FORMAT_HALF3;
#endif  
    
    gBuffer  = (GBuffer*)alignedUSMMalloc(sizeof(GBuffer)*width*height,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE);
    outputBuffer  = (GBufferOutput*)alignedUSMMalloc(sizeof(GBufferOutput)*width*height,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE);

    oidnSetSharedFilterImage(filter, "color",  gBuffer,
                             format, width, height, offsetof(GBuffer, color), sizeof(GBuffer), sizeof(GBuffer) * width); // beauty
    checkError();

    oidnSetSharedFilterImage(filter, "normal",  gBuffer,
                             format, width, height, offsetof(GBuffer, normal), sizeof(GBuffer), sizeof(GBuffer) * width); // normal

    checkError();

    oidnSetSharedFilterImage(filter, "albedo",  gBuffer,
                             format, width, height, offsetof(GBuffer, albedo), sizeof(GBuffer), sizeof(GBuffer) * width); // normal

    checkError();    
    
    oidnSetSharedFilterImage(filter, "output",  outputBuffer,
                             format, width, height, 0, sizeof(GBufferOutput), sizeof(GBufferOutput)*width); // denoised beauty
    checkError();    
    oidnSetFilter1b(filter, "ldr", true); // beauty image is HDR
    //oidnSetFilter1b(filter, "hdr", true);
    //oidnSetFilter1f(filter, "inputScale", 1.0f);  
    
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
  This->spp = user_spp;
  This->max_path_length = g_max_path_length;
  This->accu_count = 0;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
}

} // namespace embree
