// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/scene_device.h"

#include "meshoptimizer.h"
#if defined(ENABLE_OIDN)
#include <OpenImageDenoise/oidn.h>
#endif

#include "../../kernels/rthwif/rtbuild/gpu/lcgbp.h"
#include "../../kernels/rthwif/rtbuild/gpu/morton.h"


#define ENABLE_DAG 1
#define ALLOC_DEVICE_MEMORY 1
#define RELATIVE_MIN_LOD_DISTANCE_FACTOR 256
//#define RELATIVE_MIN_LOD_DISTANCE_FACTOR 28.0f
//#define RELATIVE_MIN_LOD_DISTANCE_FACTOR 11.0f

namespace embree {

extern "C" uint32_t user_spp;
extern "C" uint32_t g_max_path_length;
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

#if defined(EMBREE_SYCL_SUPPORT)
#define ENABLE_FP16_GBUFFER 1
#else
#define ENABLE_FP16_GBUFFER 0  
#endif  

#if ENABLE_FP16_GBUFFER == 1
  //typedef sycl::vec<sycl::opencl::cl_half, 3>  Vec3fp16;
  //typedef sycl::vec<sycl::opencl::cl_half, 2>  Vec2fp16;
  typedef sycl::vec<sycl::half, 3>  Vec3fp16;
  typedef sycl::vec<sycl::half, 2>  Vec2fp16;

  __forceinline Vec3f    fp_convert(const Vec3fp16 &v) { return Vec3f((float)v.x(),(float)v.y(),(float)v.z()); }
  __forceinline Vec3fp16 fp_convert(const Vec3f    &v) { return Vec3fp16(v.x,v.y,v.z);  }

  __forceinline Vec2f    fp_convert(const Vec2fp16 &v) { return Vec2f((float)v.x(),(float)v.y()); }
  __forceinline Vec2fp16 fp_convert(const Vec2f    &v) { return Vec2fp16(v.x,v.y);  }

#else
  __forceinline Vec3f    fp_convert(const Vec3f    &v) { return v; }
  __forceinline Vec2f    fp_convert(const Vec2f    &v) { return v; }  
  
#endif


struct  __attribute__ ((packed,aligned(4))) GBuffer
{  
#if ENABLE_FP16_GBUFFER == 1
  Vec3fp16 color,normal,albedo;
#else
  Vec3f color;
  Vec3f normal;
  Vec3f albedo;
#endif
  Vec3f position;
  float t;
  int primID;
  //int N;
  
  inline GBuffer() {}

  inline GBuffer(const GBuffer& gb)
    : color(gb.color), albedo(gb.albedo), normal(gb.normal), position(gb.position), t(gb.t), primID(gb.primID) {} // 
    
  inline void clear() {
#if ENABLE_FP16_GBUFFER == 0
    color = Vec3f(0.0f); 
    albedo = Vec3f(1.0f);
    normal = Vec3f(1.0f);
#else
    color = Vec3fp16(0.0f); 
    albedo = Vec3fp16(1.0);
    normal = Vec3fp16(1.0f);
#endif
    position = Vec3f(FLT_MAX);
    t = pos_inf; // inf == no hit
    primID = -1;
    //N = 1;
  }

  inline Vec3f get_normal() const {
    return fp_convert(normal);
  }
  
  
  friend GBuffer operator+ (const GBuffer& a, const GBuffer& b) {
    GBuffer res;
    res.color  = a.color  + b.color;
    res.normal = a.normal + b.normal;    
    res.albedo = a.albedo + b.albedo;
    return res;
  }

  friend GBuffer operator* (const GBuffer& a, const float b) {
    GBuffer res;
    res.color  = a.color   * b;
    res.normal = a.normal  * b;    
    res.albedo = a.albedo  * b;
    return res;      
  }

};


  

#if ENABLE_FP16_GBUFFER == 1
  typedef Vec3fp16 GBufferOutput;
#else
  typedef Vec3f GBufferOutput;
#endif

  struct PosRadius
  {
    inline PosRadius() {}

    inline PosRadius(Vec3f pos, float radius)
      : pos(pos), radius(radius) {}

    Vec3f pos = Vec3f(0);
    float radius = -float(inf);
  };


  __forceinline PosRadius getPosRadius(const float fx,
                                       const float fy,
                                       const float t,
                                       const Vec3f normal,
                                       const AffineSpace3f& xfm)
  {
    /* re-generate ray */
    Ray ray(Vec3f(xfm.p),
            Vec3f(normalize(fx*xfm.l.vx + fy*xfm.l.vy + xfm.l.vz)),
            0.0f,
            t,
            0.0f);
    
    const Vec3f wo = neg(ray.dir);
    const Vec3f dg_P  = (Vec3f)(ray.org+ray.tfar*ray.dir);
    const Vec3f dg_Ng = normal;
    const float nx = fx + 1.0f;
    const float ny = fy;
    const Vec3f neighP = ray.org + ray.tfar * normalize(nx*xfm.l.vx + ny*xfm.l.vy + xfm.l.vz);

    
    PosRadius posr;

    if (ray.tfar != (float)inf)
    {
      posr.pos = dg_P;
      posr.radius = distance(dg_P, neighP) / dot(dg_Ng, wo);
    }
    else
    {
      // nohit -> default posr !!!      
      posr = PosRadius();
    }
    return posr;
  }

inline float luminance(const Vec3f &color) {
  const Vec3f T(.2126f, .7152f, .0722f);
  return dot(color, T);
}

  
struct Denoiser
{

  static constexpr float PI = 3.1415926535897932384626422832795028841971f;
  static constexpr float TWO_PI = 6.2831853071795864769252867665590057683943f;
  static constexpr float SQRT_OF_ONE_THIRD = 0.5773502691896257645091487805019574556476f;
  static constexpr float EPSILON = 0.00001f;
  static constexpr float ECON = 2.71828f;
  
#if defined(ENABLE_OIDN)
  OIDNDevice device;
  OIDNFilter filter;
#endif  
  GBuffer *gBuffer[2];
  GBufferOutput *colorBuffer[2];
  Vec3f *momentsBuffer[2];
  float *varianceBuffer[3];
  
#define FILTER_SIZE 3
  Vec2i *hst_offset;
  float *hst_filter;
  

  
#if ENABLE_FP16_GBUFFER == 1
  Vec3fp16   *outputBuffer;  
#else  
  Vec3f   *outputBuffer;
#endif  
  uint32_t width, height;


  void filterInit(int filterSize) {
    // compute offsets and kernel values
    int o_range = filterSize / 2;
    int index = 0;
    for (int i = -o_range; i <= o_range; ++i) {
        for (int j = -o_range; j <= o_range; ++j) {
            hst_offset[index] = Vec2i(j, i);
            hst_filter[index] = ((1.f / TWO_PI) * powf(ECON, -(j * j + i * i) / 2.f));
            index++;
        }
    }
    if (index != FILTER_SIZE*FILTER_SIZE) FATAL("FILTER_SIZE");
}

  void checkError()
  {
#if defined(ENABLE_OIDN)
    const char *errorMessage;    
    if (oidnGetDeviceError(device, &errorMessage) != OIDN_ERROR_NONE) {
      std::cerr << "OIDN error: " << errorMessage << std::endl;
      FATAL("OIDN ERROR");
    }
#endif   
  }
  
  Denoiser(const uint32_t width, const uint32_t height) : width(width), height(height)
  {    
    
    // FIXME

    //auto mode = EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE;
    auto mode = EmbreeUSMMode::EMBREE_USM_SHARED;
    
    gBuffer[0]  = (GBuffer*)alignedUSMMalloc(sizeof(GBuffer)*width*height,64,mode);
    gBuffer[1]  = (GBuffer*)alignedUSMMalloc(sizeof(GBuffer)*width*height,64,mode);
    colorBuffer[0] = (GBufferOutput*)alignedUSMMalloc(sizeof(GBufferOutput)*width*height,64,mode);
    colorBuffer[1] = (GBufferOutput*)alignedUSMMalloc(sizeof(GBufferOutput)*width*height,64,mode);
    hst_offset = (Vec2i*)alignedUSMMalloc(sizeof(Vec2i)*FILTER_SIZE*FILTER_SIZE,64,mode);
    hst_filter = (float*)alignedUSMMalloc(sizeof(float)*FILTER_SIZE*FILTER_SIZE,64,mode);

    momentsBuffer[0] = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,mode);
    momentsBuffer[1] = (Vec3f*)alignedUSMMalloc(sizeof(Vec3f)*width*height,64,mode);

    varianceBuffer[0] = (float*)alignedUSMMalloc(sizeof(float)*width*height,64,mode);
    varianceBuffer[1] = (float*)alignedUSMMalloc(sizeof(float)*width*height,64,mode);
    varianceBuffer[2] = (float*)alignedUSMMalloc(sizeof(float)*width*height,64,mode);

    for (uint32_t i=0;i<width*height;i++)
    {
      gBuffer[0][i].clear();
      gBuffer[1][i].clear();
    }

    for (uint32_t i=0;i<width*height;i++) colorBuffer[0][i] = GBufferOutput(0.0f);
    for (uint32_t i=0;i<width*height;i++) colorBuffer[1][i] = GBufferOutput(0.0f);      
    
    
    //outputBuffer  = (GBufferOutput*)alignedUSMMalloc(sizeof(GBufferOutput)*width*height,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE);
    outputBuffer  = (GBufferOutput*)alignedUSMMalloc(sizeof(GBufferOutput)*width*height,64,mode);

#if defined(ENABLE_OIDN)
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
    
    filterInit(FILTER_SIZE);


    oidnSetSharedFilterImage(filter, "color",  gBuffer[0],
                             format, width, height, offsetof(GBuffer, color), sizeof(GBuffer), sizeof(GBuffer) * width); // beauty
    checkError();

    oidnSetSharedFilterImage(filter, "normal",  gBuffer[0],
                             format, width, height, offsetof(GBuffer, normal), sizeof(GBuffer), sizeof(GBuffer) * width); // normal

    checkError();

    oidnSetSharedFilterImage(filter, "albedo",  gBuffer[0],
                             format, width, height, offsetof(GBuffer, albedo), sizeof(GBuffer), sizeof(GBuffer) * width); // normal

    checkError();    
    
    oidnSetSharedFilterImage(filter, "output",  outputBuffer,
                             format, width, height, 0, sizeof(GBufferOutput), sizeof(GBufferOutput)*width); // denoised beauty
    checkError();    
    oidnSetFilterBool(filter, "ldr", true); // beauty image is HDR
    //oidnSetFilter1b(filter, "hdr", true);
    //oidnSetFilter1f(filter, "inputScale", 1.0f);  
    
    checkError();

    oidnCommitFilter(filter);
    checkError();
    
    std::cout << "done" << std::endl;
#endif    
  }

  __forceinline void execute()
  {
#if defined(ENABLE_OIDN)
    oidnExecuteFilter(filter);
    checkError();    
#endif    
  }

  ~Denoiser()
  {
#if defined(ENABLE_OIDN)
    oidnReleaseFilter(filter);    
    oidnReleaseDevice(device);    
#endif    
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

  struct Patch {
    static const unsigned int MAX_PATCH_EDGE_TESS = 10; 
    static const size_t MAX_PATCH_VERTICES = MAX_PATCH_EDGE_TESS*MAX_PATCH_EDGE_TESS;
    static const size_t MAX_PATCH_QUADS = (MAX_PATCH_EDGE_TESS-1)*(MAX_PATCH_EDGE_TESS-1);

    static const unsigned int BEZIER = 0;
      
    uint32_t type,geomID,primID,flags;
    Vec3f v[4][4];

    __forceinline Patch() {}
    __forceinline BBox3f bounds()
    {
      BBox3f patch_bounds(empty);
      if (type == 0)
      {
        for (int y=0;y<4;y++)
          for (int x=0;x<4;x++)
            patch_bounds.extend(v[y][x]);
      }
      else
      {
        for (int y=1;y<=2;y++)
          for (int x=1;x<=2;x++)
            patch_bounds.extend(v[y][x]);        
      }
      return patch_bounds;
    }
    
  };

  struct __aligned(64) LCG_Scene {
    static const unsigned int LOD_LEVELS = 3;

    /* --- general data --- */
    BBox3f bounds;
    
    /* --- lossy compressed bilinear patches --- */
    unsigned int numAllocatedLCGBP;
    unsigned int numAllocatedLCGBPStates;    
    unsigned int numLCGBP;
    unsigned int numCurrentLCGBPStates;        
    LCGBP *lcgbp;
    LCGBP_State *lcgbp_state;    
    unsigned int numCrackFixQuadNodes;

    /* --- lossy compressed meshes --- */
    unsigned int numLCQuadsTotal;
    unsigned int numLCBlocksTotal;    
    unsigned int numLCMeshClustersMaxRes;    
    unsigned int numLCMeshClusters;
    unsigned int numLCMeshClusterRoots;
    unsigned int numLCMeshClusterRootsPerFrame;
    unsigned int numLCMeshClusterQuadsPerFrame;
    unsigned int numLCMeshClusterBlocksPerFrame;
    
    //LossyCompressedMesh *lcm;
    LossyCompressedMeshCluster  *lcm_cluster;
    unsigned int *lcm_cluster_roots_IDs;
    unsigned int *lcm_cluster_roots_IDs_per_frame;
    unsigned char *lcm_cluster_active_state_per_frame;

    /* --- subdiv patches --- */
    LossyCompressedMesh *patch_mesh;
    unsigned int numSubdivPatches;
    Patch *patches;
    
    /* --- embree geometry --- */
    RTCGeometry geometry;
    unsigned int geomID;

    /* --- texture handle --- */
    Texture* map_Kd;

    /* --- LOD settings --- */
    float minLODDistance;

    /* --- pick --- */
    uint32_t pick_geomID;
    uint32_t pick_primID;
    Vec3f pick_pos;
    
    LCG_Scene(const unsigned int maxNumLCGBP);

    void addGrid(const unsigned int gridResX, const unsigned int gridResY, const Vec3fa *const vtx);
  };

  void select_clusters_lod_grid_tree(LCG_Scene *local_lcgbp_scene,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const ISPCCamera* const camera);

  void select_clusters_lod_mesh_dag(LCG_Scene *local_lcgbp_scene,
                                    const unsigned int width,
                                    const unsigned int height,
                                     const ISPCCamera* const camera);

  void select_clusters_lod_mesh_tree(LCG_Scene *local_lcgbp_scene,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const ISPCCamera* const camera);

  void select_clusters_lod_patches(LCG_Scene *local_lcgbp_scene,
                                   const unsigned int width,
                                   const unsigned int height,
                                   const ISPCCamera* const camera);


  __forceinline size_t alignTo(const unsigned int size, const unsigned int alignment)
  {
    return ((size+alignment-1)/alignment)*alignment;
  }

  __forceinline void waitOnQueueAndCatchException(sycl::queue &gpu_queue)
  {
    try {
      gpu_queue.wait_and_throw();
    } catch (sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
                << e.what() << std::endl;
      FATAL("SYCL Exception");     
    }      
  }

  __forceinline void waitOnEventAndCatchException(sycl::event &event)
  {
    try {
      event.wait_and_throw();
    } catch (sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
                << e.what() << std::endl;
      FATAL("SYCL Exception");     
    }      
  }

  __forceinline float getDeviceExecutionTiming(sycl::event &queue_event)
  {
    const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
    const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
    return (float)((t1-t0)*1E-6);      
  }

#define FEATURE_MASK RTC_FEATURE_FLAG_QUAD 

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
