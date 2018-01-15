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

#include "../../kernels/common/default.h"
#include "../../include/embree2/rtcore.h"
#include "../common/math/random_sampler.h"

namespace embree
{
  typedef decltype(nullptr) nullptr_t;

  inline std::string string_of(RTCError code)
  {
    switch (code) {
    case RTC_UNKNOWN_ERROR    : return "RTC_UNKNOWN_ERROR";
    case RTC_INVALID_ARGUMENT : return "RTC_INVALID_ARGUMENT";
    case RTC_INVALID_OPERATION: return "RTC_INVALID_OPERATION";
    case RTC_OUT_OF_MEMORY    : return "RTC_OUT_OF_MEMORY";
    case RTC_UNSUPPORTED_CPU  : return "RTC_UNSUPPORTED_CPU";
    case RTC_CANCELLED        : return "RTC_CANCELLED";
    default                   : return "invalid error code";
    }
  }

  struct RTCDeviceRef
  {
  public:
    mutable RTCDevice device;
    
    RTCDeviceRef () 
      : device(nullptr) {}

    RTCDeviceRef (nullptr_t) 
      : device(nullptr) {}
    
    RTCDeviceRef (RTCDevice device) 
    : device(device) {}

    RTCDeviceRef (const RTCDeviceRef& in) 
    {
      device = in.device;
      in.device = nullptr;
    }
    
    ~RTCDeviceRef ()
    {
      if (device == nullptr) return;
      rtcDeleteDevice(device);
    }
    
    operator RTCDevice () const { return device; }
    
    RTCDeviceRef& operator= (RTCDevice in) 
    {
      device = in;
      return *this;
    }

    RTCDeviceRef& operator= (const RTCDeviceRef& in) 
    {
      if (in.device != device && device) 
        rtcDeleteDevice(device);
      device = in.device;
      in.device = nullptr;
      return *this;
    }
      
    RTCDeviceRef& operator= (nullptr_t) 
    {
      if (device) rtcDeleteDevice(device);
      device = nullptr;
      return *this;
    }
  };

  struct RTCSceneRef
  {
  public:
    mutable RTCScene scene;
    
    RTCSceneRef (nullptr_t) 
      : scene(nullptr) {}
    
    RTCSceneRef (RTCScene scene) 
    : scene(scene) {}
    
    ~RTCSceneRef () { 
      rtcDeleteScene(scene); 
    }
    
    __forceinline operator RTCScene () const { return scene; }
    
    __forceinline RTCSceneRef& operator= (const RTCSceneRef& in) 
    {
      RTCScene tmp = in.scene;
      in.scene = nullptr;
      if (scene) rtcDeleteScene(scene);
      scene = tmp;
      return *this;
    }
      
    __forceinline RTCSceneRef& operator= (RTCScene in) 
    {
      if (scene) rtcDeleteScene(scene);
      scene = in;
      return *this;
    }
        
    __forceinline RTCSceneRef& operator= (nullptr_t) 
    {
      if (scene) rtcDeleteScene(scene);
      scene = nullptr;
      return *this;
    }
  };

  __forceinline void clearRay(RTCRay& ray)
  {
    ray.org[0] = zero;
    ray.org[1] = zero;
    ray.org[2] = zero;
    ray.align0 = 0;
    ray.dir[0] = zero;
    ray.dir[1] = zero;
    ray.dir[2] = zero;
    ray.align1 = 0;
    ray.tnear = pos_inf;
    ray.tfar = neg_inf;
    ray.time = 0;
    ray.mask = -1;
    ray.Ng[0] = 0.0f;
    ray.Ng[1] = 0.0f;
    ray.Ng[2] = 0.0f;
    ray.align2 = 0;
    ray.u = 0.0f;
    ray.v = 0.0f;
    ray.geomID = -1;
    ray.primID = -1;
    ray.instID = -1;
  }

  __forceinline RTCRay makeRay(const Vec3fa& org, const Vec3fa& dir) 
  {
    RTCRay ray; clearRay(ray);
    ray.org[0] = org.x; ray.org[1] = org.y; ray.org[2] = org.z;
    ray.dir[0] = dir.x; ray.dir[1] = dir.y; ray.dir[2] = dir.z;
    ray.tnear = 0.0f; ray.tfar = inf;
    ray.time = 0; ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
  }

  __forceinline RTCRay makeRay(const Vec3fa& org, const Vec3fa& dir, float tnear, float tfar) 
  {
    RTCRay ray; clearRay(ray);
    ray.org[0] = org.x; ray.org[1] = org.y; ray.org[2] = org.z;
    ray.dir[0] = dir.x; ray.dir[1] = dir.y; ray.dir[2] = dir.z;
    ray.tnear = tnear; ray.tfar = tfar;
    ray.time = 0; ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
  }

  __forceinline RTCRay fastMakeRay(const Vec3fa &org, const Vec3fa &dir) 
  {
    RTCRay ray;
    *(Vec3fa*)ray.org = org;
    *(Vec3fa*)ray.dir = dir;
    ray.tnear = 0.0f; 
    ray.tfar = inf;
    ray.time = 0; 
    ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
  }

  __forceinline RTCRay fastMakeRay(const Vec3fa &org, RandomSampler &sampler)
  {
    return fastMakeRay(org, 2.0f*RandomSampler_get3D(sampler)-Vec3fa(1.0f));
  }

  __forceinline void fastMakeRay(RTCRay &ray, const Vec3fa &org, const Vec3fa &dir) 
  {
    *(Vec3fa*)ray.org = org;
    *(Vec3fa*)ray.dir = dir;
    ray.tnear = 0.0f; 
    ray.tfar = inf;
    ray.time = 0; 
    ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
  }

  __forceinline void fastMakeRay(RTCRay &ray, const Vec3fa &org, RandomSampler &sampler)
  {
    fastMakeRay(ray, org, 2.0f*RandomSampler_get3D(sampler)-Vec3fa(1.0f));
  }

  __forceinline RTCRay fastMakeRay(Vec3f org, Vec3f dir, float tnear, float tfar) 
  {
    RTCRay ray;
    ray.org[0] = org.x; ray.org[1] = org.y; ray.org[2] = org.z; // FIXME: optimize
    ray.dir[0] = dir.x; ray.dir[1] = dir.y; ray.dir[2] = dir.z;
    ray.tnear = tnear; ray.tfar = tfar;
    ray.time = 0; ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
  }

  __forceinline bool neq_ray_special (const RTCRay& ray0, const RTCRay& ray1)
  {
    if (*(int*)&ray0.org[0] != *(int*)&ray1.org[0]) return true;
    if (*(int*)&ray0.org[1] != *(int*)&ray1.org[1]) return true;
    if (*(int*)&ray0.org[2] != *(int*)&ray1.org[2]) return true;
    if (*(int*)&ray0.dir[0] != *(int*)&ray1.dir[0]) return true;
    if (*(int*)&ray0.dir[1] != *(int*)&ray1.dir[1]) return true;
    if (*(int*)&ray0.dir[2] != *(int*)&ray1.dir[2]) return true;
    if (*(int*)&ray0.tnear  != *(int*)&ray1.tnear ) return true;
    if (*(int*)&ray0.tfar   != *(int*)&ray1.tfar  ) return true;
    if (*(int*)&ray0.time   != *(int*)&ray1.time  ) return true;
    if (*(int*)&ray0.mask   != *(int*)&ray1.mask  ) return true;
    if (*(int*)&ray0.u      != *(int*)&ray1.u     ) return true;
    if (*(int*)&ray0.v      != *(int*)&ray1.v     ) return true;
    if (*(int*)&ray0.instID != *(int*)&ray1.instID) return true;
    if (*(int*)&ray0.geomID != *(int*)&ray1.geomID) return true;
    if (*(int*)&ray0.primID != *(int*)&ray1.primID) return true;
    if (*(int*)&ray0.Ng[0]  != *(int*)&ray1.Ng[0] ) return true;
    if (*(int*)&ray0.Ng[1]  != *(int*)&ray1.Ng[1] ) return true;
    if (*(int*)&ray0.Ng[2]  != *(int*)&ray1.Ng[2] ) return true;
    return false;
  }

  /* Outputs ray to stream */
  __forceinline std::ostream& operator<<(std::ostream& cout, const RTCRay& ray)
  {
    return cout << "Ray { " << std::endl
                << "  org = " << ray.org[0] << " " << ray.org[1] << " " << ray.org[2] << std::endl
                << "  dir = " << ray.dir[0] << " " << ray.dir[1] << " " << ray.dir[2] << std::endl
                << "  near = " << ray.tnear << std::endl
                << "  far = " << ray.tfar << std::endl
                << "  time = " << ray.time << std::endl
                << "  mask = " << ray.mask << std::endl
                << "  instID = " << ray.instID << std::endl
                << "  geomID = " << ray.geomID << std::endl
                << "  primID = " << ray.primID <<  std::endl
                << "  u = " << ray.u <<  std::endl
                << "  v = " << ray.v << std::endl
                << "  Ng = " << ray.Ng[0] << " " << ray.Ng[1] << " " << ray.Ng[2] << std::endl
                << "}";
  }

  __forceinline void setRay(RTCRay4& ray_o, size_t i, const RTCRay& ray_i)
  {
    ray_o.orgx[i] = ray_i.org[0];
    ray_o.orgy[i] = ray_i.org[1];
    ray_o.orgz[i] = ray_i.org[2];
    ray_o.dirx[i] = ray_i.dir[0];
    ray_o.diry[i] = ray_i.dir[1];
    ray_o.dirz[i] = ray_i.dir[2];
    ray_o.tnear[i] = ray_i.tnear;
    ray_o.tfar[i] = ray_i.tfar;
    ray_o.time[i] = ray_i.time;
    ray_o.mask[i] = ray_i.mask;
    ray_o.instID[i] = ray_i.instID;
    ray_o.geomID[i] = ray_i.geomID;
    ray_o.primID[i] = ray_i.primID;
    ray_o.u[i] = ray_i.u;
    ray_o.v[i] = ray_i.v;
    ray_o.Ngx[i] = ray_i.Ng[0];
    ray_o.Ngy[i] = ray_i.Ng[1];
    ray_o.Ngz[i] = ray_i.Ng[2];
  }

  __forceinline void setRay(RTCRay8& ray_o, size_t i, const RTCRay& ray_i)
  {
    ray_o.orgx[i] = ray_i.org[0];
    ray_o.orgy[i] = ray_i.org[1];
    ray_o.orgz[i] = ray_i.org[2];
    ray_o.dirx[i] = ray_i.dir[0];
    ray_o.diry[i] = ray_i.dir[1];
    ray_o.dirz[i] = ray_i.dir[2];
    ray_o.tnear[i] = ray_i.tnear;
    ray_o.tfar[i] = ray_i.tfar;
    ray_o.time[i] = ray_i.time;
    ray_o.mask[i] = ray_i.mask;
    ray_o.instID[i] = ray_i.instID;
    ray_o.geomID[i] = ray_i.geomID;
    ray_o.primID[i] = ray_i.primID;
    ray_o.u[i] = ray_i.u;
    ray_o.v[i] = ray_i.v;
    ray_o.Ngx[i] = ray_i.Ng[0];
    ray_o.Ngy[i] = ray_i.Ng[1];
    ray_o.Ngz[i] = ray_i.Ng[2];
  }

  __forceinline void setRay(RTCRay16& ray_o, size_t i, const RTCRay& ray_i)
  {
    ray_o.orgx[i] = ray_i.org[0];
    ray_o.orgy[i] = ray_i.org[1];
    ray_o.orgz[i] = ray_i.org[2];
    ray_o.dirx[i] = ray_i.dir[0];
    ray_o.diry[i] = ray_i.dir[1];
    ray_o.dirz[i] = ray_i.dir[2];
    ray_o.tnear[i] = ray_i.tnear;
    ray_o.tfar[i] = ray_i.tfar;
    ray_o.time[i] = ray_i.time;
    ray_o.mask[i] = ray_i.mask;
    ray_o.instID[i] = ray_i.instID;
    ray_o.geomID[i] = ray_i.geomID;
    ray_o.primID[i] = ray_i.primID;
    ray_o.u[i] = ray_i.u;
    ray_o.v[i] = ray_i.v;
    ray_o.Ngx[i] = ray_i.Ng[0];
    ray_o.Ngy[i] = ray_i.Ng[1];
    ray_o.Ngz[i] = ray_i.Ng[2];
  }

  __forceinline void setRay(RTCRayN* ray_o, size_t N, size_t i, const RTCRay& ray_i)
  {
    RTCRayN_org_x(ray_o,N,i) = ray_i.org[0];
    RTCRayN_org_y(ray_o,N,i) = ray_i.org[1];
    RTCRayN_org_z(ray_o,N,i) = ray_i.org[2];
    RTCRayN_dir_x(ray_o,N,i) = ray_i.dir[0];
    RTCRayN_dir_y(ray_o,N,i) = ray_i.dir[1];
    RTCRayN_dir_z(ray_o,N,i) = ray_i.dir[2];
    RTCRayN_tnear(ray_o,N,i) = ray_i.tnear;
    RTCRayN_tfar(ray_o,N,i) = ray_i.tfar;
    RTCRayN_time(ray_o,N,i) = ray_i.time;
    RTCRayN_mask(ray_o,N,i) = ray_i.mask;
    RTCRayN_instID(ray_o,N,i) = ray_i.instID;
    RTCRayN_geomID(ray_o,N,i) = ray_i.geomID;
    RTCRayN_primID(ray_o,N,i) = ray_i.primID;
    RTCRayN_u(ray_o,N,i) = ray_i.u;
    RTCRayN_v(ray_o,N,i) = ray_i.v;
    RTCRayN_Ng_x(ray_o,N,i) = ray_i.Ng[0];
    RTCRayN_Ng_y(ray_o,N,i) = ray_i.Ng[1];
    RTCRayN_Ng_z(ray_o,N,i) = ray_i.Ng[2];
  }

  __forceinline RTCRay getRay(RTCRay4& ray_i, size_t i)
  {
    RTCRay ray_o;
    ray_o.org[0] = ray_i.orgx[i];
    ray_o.org[1] = ray_i.orgy[i];
    ray_o.org[2] = ray_i.orgz[i];
    ray_o.dir[0] = ray_i.dirx[i];
    ray_o.dir[1] = ray_i.diry[i];
    ray_o.dir[2] = ray_i.dirz[i];
    ray_o.tnear = ray_i.tnear[i];
    ray_o.tfar = ray_i.tfar[i];
    ray_o.time = ray_i.time[i];
    ray_o.mask = ray_i.mask[i];
    ray_o.instID = ray_i.instID[i];
    ray_o.geomID = ray_i.geomID[i];
    ray_o.primID = ray_i.primID[i];
    ray_o.u = ray_i.u[i];
    ray_o.v = ray_i.v[i];
    ray_o.Ng[0] = ray_i.Ngx[i];
    ray_o.Ng[1] = ray_i.Ngy[i];
    ray_o.Ng[2] = ray_i.Ngz[i];
    return ray_o;
  }

  __forceinline RTCRay getRay(RTCRay8& ray_i, size_t i)
  {
    RTCRay ray_o;
    ray_o.org[0] = ray_i.orgx[i];
    ray_o.org[1] = ray_i.orgy[i];
    ray_o.org[2] = ray_i.orgz[i];
    ray_o.dir[0] = ray_i.dirx[i];
    ray_o.dir[1] = ray_i.diry[i];
    ray_o.dir[2] = ray_i.dirz[i];
    ray_o.tnear = ray_i.tnear[i];
    ray_o.tfar = ray_i.tfar[i];
    ray_o.time = ray_i.time[i];
    ray_o.mask = ray_i.mask[i];
    ray_o.instID = ray_i.instID[i];
    ray_o.geomID = ray_i.geomID[i];
    ray_o.primID = ray_i.primID[i];
    ray_o.u = ray_i.u[i];
    ray_o.v = ray_i.v[i];
    ray_o.Ng[0] = ray_i.Ngx[i];
    ray_o.Ng[1] = ray_i.Ngy[i];
    ray_o.Ng[2] = ray_i.Ngz[i];
    return ray_o;
  }

  __forceinline RTCRay getRay(RTCRay16& ray_i, size_t i)
  {
    RTCRay ray_o;
    ray_o.org[0] = ray_i.orgx[i];
    ray_o.org[1] = ray_i.orgy[i];
    ray_o.org[2] = ray_i.orgz[i];
    ray_o.dir[0] = ray_i.dirx[i];
    ray_o.dir[1] = ray_i.diry[i];
    ray_o.dir[2] = ray_i.dirz[i];
    ray_o.tnear = ray_i.tnear[i];
    ray_o.tfar = ray_i.tfar[i];
    ray_o.time = ray_i.time[i];
    ray_o.mask = ray_i.mask[i];
    ray_o.instID = ray_i.instID[i];
    ray_o.geomID = ray_i.geomID[i];
    ray_o.primID = ray_i.primID[i];
    ray_o.u = ray_i.u[i];
    ray_o.v = ray_i.v[i];
    ray_o.Ng[0] = ray_i.Ngx[i];
    ray_o.Ng[1] = ray_i.Ngy[i];
    ray_o.Ng[2] = ray_i.Ngz[i];
    return ray_o;
  }

  __forceinline RTCRay getRay(RTCRayN* ray_i, size_t N, size_t i)
  {
    RTCRay ray_o;
    ray_o.org[0] = RTCRayN_org_x(ray_i,N,i);
    ray_o.org[1] = RTCRayN_org_y(ray_i,N,i);
    ray_o.org[2] = RTCRayN_org_z(ray_i,N,i);
    ray_o.dir[0] = RTCRayN_dir_x(ray_i,N,i);
    ray_o.dir[1] = RTCRayN_dir_y(ray_i,N,i);
    ray_o.dir[2] = RTCRayN_dir_z(ray_i,N,i);
    ray_o.tnear = RTCRayN_tnear(ray_i,N,i);
    ray_o.tfar  = RTCRayN_tfar(ray_i,N,i);
    ray_o.time = RTCRayN_time(ray_i,N,i);
    ray_o.mask = RTCRayN_mask(ray_i,N,i);
    ray_o.instID = RTCRayN_instID(ray_i,N,i);
    ray_o.geomID = RTCRayN_geomID(ray_i,N,i);
    ray_o.primID = RTCRayN_primID(ray_i,N,i);
    ray_o.u = RTCRayN_u(ray_i,N,i);
    ray_o.v = RTCRayN_v(ray_i,N,i);
    ray_o.Ng[0] = RTCRayN_Ng_x(ray_i,N,i);
    ray_o.Ng[1] = RTCRayN_Ng_y(ray_i,N,i);
    ray_o.Ng[2] = RTCRayN_Ng_z(ray_i,N,i);
    return ray_o;
  }

  enum IntersectMode 
  {
    MODE_INTERSECT_NONE,
    MODE_INTERSECT1,
    MODE_INTERSECT4,
    MODE_INTERSECT8,
    MODE_INTERSECT16,
    MODE_INTERSECT1M,
    MODE_INTERSECT1Mp,
    MODE_INTERSECTNM1,
    MODE_INTERSECTNM3,
    MODE_INTERSECTNM4,
    MODE_INTERSECTNM8,
    MODE_INTERSECTNM16,
    MODE_INTERSECTNp
  };

  inline std::string to_string(IntersectMode imode)
  {
    switch (imode) {
    case MODE_INTERSECT_NONE: return "None";
    case MODE_INTERSECT1: return "1";
    case MODE_INTERSECT4: return "4";
    case MODE_INTERSECT8: return "8";
    case MODE_INTERSECT16: return "16";
    case MODE_INTERSECT1M: return "1M";
    case MODE_INTERSECT1Mp: return "1Mp";
    case MODE_INTERSECTNM1: return "NM1";
    case MODE_INTERSECTNM3: return "NM3";
    case MODE_INTERSECTNM4: return "NM4";
    case MODE_INTERSECTNM8: return "NM8";
    case MODE_INTERSECTNM16: return "NM16";
    case MODE_INTERSECTNp: return "Np";
    default                : return "U";
    }
  }

  inline size_t alignment_of(IntersectMode imode)
  {
    switch (imode) {
    case MODE_INTERSECT_NONE: return 0;
    case MODE_INTERSECT1: return 16;
    case MODE_INTERSECT4: return 16;
    case MODE_INTERSECT8: return 32;
    case MODE_INTERSECT16: return 64;
    case MODE_INTERSECT1M: return 16;
    case MODE_INTERSECT1Mp: return 16;
    case MODE_INTERSECTNM1: return 16;
    case MODE_INTERSECTNM3: return 16;
    case MODE_INTERSECTNM4: return 16;
    case MODE_INTERSECTNM8: return 16;
    case MODE_INTERSECTNM16: return 16;
    case MODE_INTERSECTNp: return 16;
    default              : return 0;
    }
  }
  
  enum IntersectVariant
  {
    VARIANT_INTERSECT = 1,
    VARIANT_OCCLUDED = 2,
    VARIANT_COHERENT = 0,
    VARIANT_INCOHERENT = 4,
    VARIANT_INTERSECT_OCCLUDED_MASK = 3,
    VARIANT_COHERENT_INCOHERENT_MASK = 4,
    
    VARIANT_INTERSECT_COHERENT = 1,
    VARIANT_OCCLUDED_COHERENT = 2,
    VARIANT_INTERSECT_INCOHERENT = 5,
    VARIANT_OCCLUDED_INCOHERENT = 6,
    VARIANT_INTERSECT_OCCLUDED = 3,
    VARIANT_INTERSECT_OCCLUDED_COHERENT = 3,
    VARIANT_INTERSECT_OCCLUDED_INCOHERENT = 7,
  };

  inline std::string to_string(IntersectVariant ivariant)
  {
    switch (ivariant) {
    case VARIANT_INTERSECT_COHERENT: return "IntersectCoherent";
    case VARIANT_OCCLUDED_COHERENT : return "OccludedCoherent";
    case VARIANT_INTERSECT_INCOHERENT: return "IntersectIncoherent";
    case VARIANT_OCCLUDED_INCOHERENT : return "OccludedIncoherent";
    case VARIANT_INTERSECT_OCCLUDED_COHERENT: return "IntersectOccludedCoherent";
    case VARIANT_INTERSECT_OCCLUDED_INCOHERENT : return "IntersectOccludedIncoherent";
    default: assert(false);
    }
    return "";
  }

  inline bool has_variant(IntersectMode imode, IntersectVariant ivariant)
  {
    switch (imode) {
    case MODE_INTERSECT1:
    case MODE_INTERSECT4:
    case MODE_INTERSECT8:
    case MODE_INTERSECT16:
      switch (ivariant) {
      case VARIANT_INTERSECT: return true;
      case VARIANT_OCCLUDED : return true;
      case VARIANT_INTERSECT_OCCLUDED : return true;
      default: return false;
      }
    default:
      return true;
    }
  }

  inline std::string to_string(IntersectMode imode, IntersectVariant ivariant)
  {
    switch (imode) {
    case MODE_INTERSECT1:
    case MODE_INTERSECT4:
    case MODE_INTERSECT8:
    case MODE_INTERSECT16:
      switch (ivariant) {
      case VARIANT_INTERSECT: return "Intersect" + to_string(imode);
      case VARIANT_OCCLUDED : return "Occluded" + to_string(imode);
      case VARIANT_INTERSECT_OCCLUDED : return "IntersectOccluded" + to_string(imode);
      default: assert(false);
      }
    default:
      return to_string(ivariant) + to_string(imode);
    }
  }

  inline RTCAlgorithmFlags to_aflags(IntersectMode imode)
  {
    switch (imode) {
    case MODE_INTERSECT1: return RTC_INTERSECT1;
    case MODE_INTERSECT4: return RTC_INTERSECT4;
    case MODE_INTERSECT8: return RTC_INTERSECT8;
    case MODE_INTERSECT16: return RTC_INTERSECT16;
    case MODE_INTERSECT1M: return RTC_INTERSECT_STREAM;
    case MODE_INTERSECT1Mp: return RTC_INTERSECT_STREAM;
    case MODE_INTERSECTNM1: return RTC_INTERSECT_STREAM;
    case MODE_INTERSECTNM3: return RTC_INTERSECT_STREAM;
    case MODE_INTERSECTNM4: return RTC_INTERSECT_STREAM;
    case MODE_INTERSECTNM8: return RTC_INTERSECT_STREAM;
    case MODE_INTERSECTNM16: return RTC_INTERSECT_STREAM;
    case MODE_INTERSECTNp: return RTC_INTERSECT_STREAM;
    default                : return RTC_INTERSECT1;
    }
  }

  inline std::string to_string(RTCSceneFlags sflags)
  {
    std::string str = "";
    if (sflags & RTC_SCENE_DYNAMIC) str += "Dynamic"; else str += "Static";
    if (sflags & RTC_SCENE_COMPACT) str += "Compact";
    if (sflags & RTC_SCENE_ROBUST ) str += "Robust";
    if (sflags & RTC_SCENE_HIGH_QUALITY) str += "HighQuality";
    return str;
  }

  inline std::string to_string(RTCGeometryFlags gflags)
  {
    switch (gflags) {
    case RTC_GEOMETRY_STATIC: return "Static";
    case RTC_GEOMETRY_DEFORMABLE: return "Deformable";
    case RTC_GEOMETRY_DYNAMIC: return "Dynamic";
    default: return "UnknownGeometryFlags";
    }
  }

  inline std::string to_string(RTCSceneFlags sflags, RTCGeometryFlags gflags) {
    return to_string(sflags)+to_string(gflags);
  }

  static const size_t numSceneFlags = 64;

  RTCSceneFlags getSceneFlag(size_t i) 
  {
    int flag = 0;                               
    if (i & 1) flag |= RTC_SCENE_DYNAMIC;
    if (i & 2) flag |= RTC_SCENE_COMPACT;
    if (i & 4) flag |= RTC_SCENE_COHERENT;
    if (i & 8) flag |= RTC_SCENE_INCOHERENT;
    if (i & 16) flag |= RTC_SCENE_HIGH_QUALITY;
    if (i & 32) flag |= RTC_SCENE_ROBUST;
    return (RTCSceneFlags) flag;
  }

  static const size_t numSceneGeomFlags = 32;

  void getSceneGeomFlag(size_t i, RTCSceneFlags& sflags, RTCGeometryFlags& gflags) 
  {
    int sflag = 0, gflag = 0;
    if (i & 4) {
      sflag |= RTC_SCENE_DYNAMIC;
      gflag = min(int(i)&3,2);
    }
    if (i & 8) sflag |= RTC_SCENE_HIGH_QUALITY;
    if (i & 16) sflag |= RTC_SCENE_ROBUST;
    sflags = (RTCSceneFlags) sflag;
    gflags = (RTCGeometryFlags) gflag;
  }

  inline bool supportsIntersectMode(RTCDevice device, IntersectMode imode)
  { 
    switch (imode) {
    case MODE_INTERSECT_NONE: return true;
    case MODE_INTERSECT1:   return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT1);
    case MODE_INTERSECT4:   return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT4);
    case MODE_INTERSECT8:   return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT8);
    case MODE_INTERSECT16:  return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT16);
    case MODE_INTERSECT1M:  return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    case MODE_INTERSECT1Mp: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    case MODE_INTERSECTNM1: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    case MODE_INTERSECTNM3: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    case MODE_INTERSECTNM4: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    case MODE_INTERSECTNM8: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    case MODE_INTERSECTNM16:return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    case MODE_INTERSECTNp:  return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
    }
    assert(false);
    return false;
  }

  template<int N>
    __noinline void IntersectWithNMMode(IntersectVariant ivariant, RTCScene scene, RTCIntersectContext* context, RTCRay* rays, size_t Nrays)
  {
    assert(Nrays<1024);
    const size_t alignment = size_t(rays) % 64;
    __aligned(64) char data[1024*sizeof(RTCRay)+64];
    assert((size_t)data % 64 == 0);
    for (size_t i=0; i<Nrays; i+=N) 
    {
      size_t L = min(size_t(N),Nrays-i);
      RTCRayN* ray = (RTCRayN*) &data[alignment+i*sizeof(RTCRay)];
      for (size_t j=0; j<L; j++) setRay(ray,N,j,rays[i+j]);
      for (size_t j=L; j<N; j++) setRay(ray,N,j,makeRay(zero,zero,pos_inf,neg_inf));
    }
    
    size_t M = (Nrays+N-1)/N;
    assert(sizeof(RTCRayN) <= sizeof(RTCRay)*N);
    switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
    case VARIANT_INTERSECT: rtcIntersectNM(scene,context,(RTCRayN*)&data[alignment],N,M,N*sizeof(RTCRay)); break;
    case VARIANT_OCCLUDED : rtcOccludedNM(scene,context,(RTCRayN*)&data[alignment],N,M,N*sizeof(RTCRay)); break;
    default: assert(false);
    }
    
    for (size_t i=0; i<Nrays; i+=N) 
    {
      size_t L = min(size_t(N),Nrays-i);
      RTCRayN* ray = (RTCRayN*) &data[alignment+i*sizeof(RTCRay)];
      for (size_t j=0; j<L; j++) rays[i+j] = getRay(ray,N,j);
    }
  }
	
	__noinline void IntersectWithNpMode(IntersectVariant ivariant, RTCScene scene, RTCIntersectContext* context, RTCRay* rays, size_t N)
	{
		assert(N < 1024);
		const size_t alignment = size_t(rays) % 64;
		__aligned(64) char data[1024 * sizeof(RTCRay) + 64];
		RTCRayN* ray = (RTCRayN*)&data[alignment];
		for (size_t j = 0; j < N; j++) setRay(ray, N, j, rays[j]);

		RTCRayNp rayp;
		rayp.orgx = &RTCRayN_org_x(ray, N, 0);
		rayp.orgy = &RTCRayN_org_y(ray, N, 0);
		rayp.orgz = &RTCRayN_org_z(ray, N, 0);
		rayp.dirx = &RTCRayN_dir_x(ray, N, 0);
		rayp.diry = &RTCRayN_dir_y(ray, N, 0);
		rayp.dirz = &RTCRayN_dir_z(ray, N, 0);
		rayp.tnear = &RTCRayN_tnear(ray, N, 0);
		rayp.tfar = &RTCRayN_tfar(ray, N, 0);
		rayp.time = &RTCRayN_time(ray, N, 0);
		rayp.mask = &RTCRayN_mask(ray, N, 0);
		rayp.instID = &RTCRayN_instID(ray, N, 0);
		rayp.geomID = &RTCRayN_geomID(ray, N, 0);
		rayp.primID = &RTCRayN_primID(ray, N, 0);
		rayp.u = &RTCRayN_u(ray, N, 0);
		rayp.v = &RTCRayN_v(ray, N, 0);
		rayp.Ngx = &RTCRayN_Ng_x(ray, N, 0);
		rayp.Ngy = &RTCRayN_Ng_y(ray, N, 0);
		rayp.Ngz = &RTCRayN_Ng_z(ray, N, 0);

		switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
		case VARIANT_INTERSECT: rtcIntersectNp(scene, context, rayp, N); break;
		case VARIANT_OCCLUDED: rtcOccludedNp(scene, context, rayp, N); break;
		default: assert(false);
		}

		for (size_t j = 0; j < N; j++) rays[j] = getRay(ray, N, j);
	}
	
  __noinline void IntersectWithModeInternal(IntersectMode mode, IntersectVariant ivariant, RTCScene scene, RTCRay* rays, size_t N)
  {
    RTCIntersectContext context;
    context.flags = ((ivariant & VARIANT_COHERENT_INCOHERENT_MASK) == VARIANT_COHERENT) ? RTC_INTERSECT_COHERENT :  RTC_INTERSECT_INCOHERENT;
    context.userRayExt = nullptr;

    switch (mode) 
    {
    case MODE_INTERSECT_NONE: 
      break;
    case MODE_INTERSECT1:
    {
      switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
      case VARIANT_INTERSECT: for (size_t i=0; i<N; i++) rtcIntersect(scene,rays[i]); break;
      case VARIANT_OCCLUDED : for (size_t i=0; i<N; i++) rtcOccluded (scene,rays[i]); break;
      default: assert(false);
      }
      break;
    }

    case MODE_INTERSECT4: 
    {
      for (size_t i=0; i<N; i+=4) 
      {
        size_t M = min(size_t(4),N-i);
        __aligned(16) int valid[4];
        __aligned(16) RTCRay4 ray4;
        for (size_t j=0; j<4; j++) valid[j] = (j<M && rays[i+j].tnear <= rays[i+j].tfar) ? -1 : 0;
        for (size_t j=0; j<M; j++) setRay(ray4,j,rays[i+j]);
        for (size_t j=M; j<4; j++) setRay(ray4,j,makeRay(zero,zero,pos_inf,neg_inf));
        switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
        case VARIANT_INTERSECT: rtcIntersect4(valid,scene,ray4); break;
        case VARIANT_OCCLUDED : rtcOccluded4 (valid,scene,ray4); break;
        default: assert(false);
        }
        for (size_t j=0; j<M; j++) rays[i+j] = getRay(ray4,j);
      }
      break;
    }
    case MODE_INTERSECT8: 
    {
      for (size_t i=0; i<N; i+=8) 
      {
        size_t M = min(size_t(8),N-i);
        __aligned(32) int valid[8];
        __aligned(32) RTCRay8 ray8;
        for (size_t j=0; j<8; j++) valid[j] = (j<M && rays[i+j].tnear <= rays[i+j].tfar) ? -1 : 0;
        for (size_t j=0; j<M; j++) setRay(ray8,j,rays[i+j]);
        for (size_t j=M; j<8; j++) setRay(ray8,j,makeRay(zero,zero,pos_inf,neg_inf));
        switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
        case VARIANT_INTERSECT: rtcIntersect8(valid,scene,ray8); break;
        case VARIANT_OCCLUDED : rtcOccluded8 (valid,scene,ray8); break;
        default: assert(false);
        }
        for (size_t j=0; j<M; j++) rays[i+j] = getRay(ray8,j);
      }
      break;
    }
    case MODE_INTERSECT16: 
    {
      for (size_t i=0; i<N; i+=16) 
      {
        size_t M = min(size_t(16),N-i);
        __aligned(64) int valid[16];
        __aligned(64) RTCRay16 ray16;
        for (size_t j=0; j<16; j++) valid[j] = (j<M && rays[i+j].tnear <= rays[i+j].tfar) ? -1 : 0;
        for (size_t j=0; j<M ; j++) setRay(ray16,j,rays[i+j]);
        for (size_t j=M; j<16; j++) setRay(ray16,j,makeRay(zero,zero,pos_inf,neg_inf));
        switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
        case VARIANT_INTERSECT: rtcIntersect16(valid,scene,ray16); break;
        case VARIANT_OCCLUDED : rtcOccluded16 (valid,scene,ray16); break;
        default: assert(false);
        }
        for (size_t j=0; j<M; j++) rays[i+j] = getRay(ray16,j);
      }
      break;
    }
    case MODE_INTERSECT1M: 
    {
      switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
      case VARIANT_INTERSECT: rtcIntersect1M(scene,&context,rays,N,sizeof(RTCRay)); break;
      case VARIANT_OCCLUDED : rtcOccluded1M (scene,&context,rays,N,sizeof(RTCRay)); break;
      default: assert(false);
      }
      break;
    }
    case MODE_INTERSECT1Mp: 
    {
      assert(N<1024);
      RTCRay* rptrs[1024];
      for (size_t i=0; i<N; i++) rptrs[i] = &rays[i];
      switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
      case VARIANT_INTERSECT: rtcIntersect1Mp(scene,&context,rptrs,N); break;
      case VARIANT_OCCLUDED : rtcOccluded1Mp (scene,&context,rptrs,N); break;
      default: assert(false);
      }
      break;
    }
    case MODE_INTERSECTNM1: {
      IntersectWithNMMode<1>(ivariant,scene,&context,rays,N);
      break;
    }
    case MODE_INTERSECTNM3: {
      IntersectWithNMMode<3>(ivariant,scene,&context,rays,N);
      break;
    }
    case MODE_INTERSECTNM4: {
      IntersectWithNMMode<4>(ivariant,scene,&context,rays,N);
      break;
    }
    case MODE_INTERSECTNM8: {
      IntersectWithNMMode<8>(ivariant,scene,&context,rays,N);
      break;
    }
    case MODE_INTERSECTNM16: {
      IntersectWithNMMode<16>(ivariant,scene,&context,rays,N);
      break;
    }
    case MODE_INTERSECTNp: {
	  IntersectWithNpMode(ivariant, scene, &context, rays, N);
      break;
    }
    }
  }

  void IntersectWithMode(IntersectMode mode, IntersectVariant ivariant, RTCScene scene, RTCRay* rays, size_t N)
  {
    /* verify occluded result against intersect */
    if ((ivariant & VARIANT_INTERSECT_OCCLUDED) == VARIANT_INTERSECT_OCCLUDED)
    {
      vector_t<RTCRay,aligned_allocator<RTCRay,16>> rays2(N);
      for (size_t i=0; i<N; i++) rays2[i] = rays[i];
      IntersectWithModeInternal(mode,IntersectVariant(ivariant & ~VARIANT_OCCLUDED),scene,rays,N);
      IntersectWithModeInternal(mode,IntersectVariant(ivariant & ~VARIANT_INTERSECT),scene,rays2.data(),N);
      for (size_t i=0; i<N; i++)
        if ((rays[i].geomID == RTC_INVALID_GEOMETRY_ID) != (rays2[i].geomID == RTC_INVALID_GEOMETRY_ID))
          throw std::runtime_error("Intersect/Occluded mismatch");
    }
    else
      IntersectWithModeInternal(mode,ivariant,scene,rays,N);
  }

  enum GeometryType
  {
    TRIANGLE_MESH,
    TRIANGLE_MESH_MB,
    QUAD_MESH,
    QUAD_MESH_MB,
    SUBDIV_MESH,
    SUBDIV_MESH_MB,
    HAIR_GEOMETRY,
    HAIR_GEOMETRY_MB,
    CURVE_GEOMETRY,
    CURVE_GEOMETRY_MB,
    LINE_GEOMETRY,
    LINE_GEOMETRY_MB
  };

  inline std::string to_string(GeometryType gtype)
  {
    switch (gtype) {
    case TRIANGLE_MESH    : return "triangles";
    case TRIANGLE_MESH_MB : return "triangles_mb";
    case QUAD_MESH        : return "quads";
    case QUAD_MESH_MB     : return "quads_mb";
    case SUBDIV_MESH      : return "subdivs";
    case SUBDIV_MESH_MB   : return "subdivs_mb";
    case HAIR_GEOMETRY    : return "hair";
    case HAIR_GEOMETRY_MB : return "hair_mb";
    case CURVE_GEOMETRY   : return "curves";
    case CURVE_GEOMETRY_MB: return "curves_mb";
    case LINE_GEOMETRY   : return "lines";
    case LINE_GEOMETRY_MB: return "lines_mb";
    }
    return "";
  }

   inline std::string to_string(RTCSceneFlags sflags, IntersectMode imode) {
    return to_string(sflags) + "." + to_string(imode);
  }

  inline std::string to_string(RTCSceneFlags sflags, IntersectMode imode, IntersectVariant ivariant) {
    return to_string(sflags) + "." + to_string(imode,ivariant);
  }

  inline std::string to_string(GeometryType gtype, RTCSceneFlags sflags, IntersectMode imode, IntersectVariant ivariant) {
    return to_string(gtype) + "." + to_string(sflags) + "." + to_string(imode,ivariant);
  }

  /* error reporting function */
  void error_handler(void* userPtr, const RTCError code, const char* str = nullptr)
  {
    if (code == RTC_NO_ERROR) 
      return;
    
    std::string errorStr;
    errorStr += "Embree: ";
    errorStr += string_of(code);
    if (str) errorStr += " (" + std::string(str) + ")";
    throw std::runtime_error(errorStr);
  }
}

