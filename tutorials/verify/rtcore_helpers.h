// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../kernels/common/default.h"
#include "../../include/embree4/rtcore.h"
RTC_NAMESPACE_USE
#include "../common/math/random_sampler.h"

namespace embree
{
  typedef decltype(nullptr) nullptr_t;

  inline std::string string_of(RTCError code)
  {
    switch (code) {
    case RTC_ERROR_UNKNOWN          : return "RTC_ERROR_UNKNOWN";
    case RTC_ERROR_INVALID_ARGUMENT : return "RTC_ERROR_INVALID_ARGUMENT";
    case RTC_ERROR_INVALID_OPERATION: return "RTC_ERROR_INVALID_OPERATION";
    case RTC_ERROR_OUT_OF_MEMORY    : return "RTC_ERROR_OUT_OF_MEMORY";
    case RTC_ERROR_UNSUPPORTED_CPU  : return "RTC_ERROR_UNSUPPORTED_CPU";
    case RTC_ERROR_CANCELLED        : return "RTC_ERROR_CANCELLED";
    default                         : return "invalid error code";
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
      rtcReleaseDevice(device);
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
        rtcReleaseDevice(device);
      device = in.device;
      in.device = nullptr;
      return *this;
    }
      
    RTCDeviceRef& operator= (nullptr_t) 
    {
      if (device) rtcReleaseDevice(device);
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
      rtcReleaseScene(scene); 
    }
    
    __forceinline operator RTCScene () const { return scene; }
    
    __forceinline RTCSceneRef& operator= (const RTCSceneRef& in) 
    {
      RTCScene tmp = in.scene;
      in.scene = nullptr;
      if (scene) rtcReleaseScene(scene);
      scene = tmp;
      return *this;
    }
      
    __forceinline RTCSceneRef& operator= (RTCScene in) 
    {
      if (scene) rtcReleaseScene(scene);
      scene = in;
      return *this;
    }
        
    __forceinline RTCSceneRef& operator= (nullptr_t) 
    {
      if (scene) rtcReleaseScene(scene);
      scene = nullptr;
      return *this;
    }
  };

  __forceinline void clearRay(RTCRayHit& rh)
  {
    rh.ray.org_x = zero;
    rh.ray.org_y = zero;
    rh.ray.org_z = zero;
    rh.ray.dir_x = zero;
    rh.ray.dir_y = zero;
    rh.ray.dir_z = zero;
    rh.ray.tnear = pos_inf;
    rh.ray.tfar = neg_inf;
    rh.ray.time = 0;
    rh.ray.mask = -1;
    rh.ray.id = -1;
    rh.hit.Ng_x = 0.0f;
    rh.hit.Ng_y = 0.0f;
    rh.hit.Ng_z = 0.0f;
    rh.hit.u = 0.0f;
    rh.hit.v = 0.0f;
    rh.hit.geomID = -1;
    rh.hit.primID = -1;
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      rh.hit.instID[l] = RTC_INVALID_GEOMETRY_ID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      rh.hit.instPrimID[l] = RTC_INVALID_GEOMETRY_ID;
#endif
    }
  }

  __forceinline RTCRayHit makeRay(const Vec3fa& org, const Vec3fa& dir) 
  {
    RTCRayHit rh; clearRay(rh);
    rh.ray.org_x = org.x; rh.ray.org_y = org.y; rh.ray.org_z = org.z;
    rh.ray.dir_x = dir.x; rh.ray.dir_y = dir.y; rh.ray.dir_z = dir.z;
    rh.ray.tnear = 0.0f; rh.ray.tfar = inf;
    rh.ray.time = 0; rh.ray.mask = -1;
    rh.hit.geomID = rh.hit.primID = -1;
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      rh.hit.instID[l] = RTC_INVALID_GEOMETRY_ID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      rh.hit.instPrimID[l] = RTC_INVALID_GEOMETRY_ID;
#endif
    }
    return rh;
  }

  __forceinline RTCRayHit makeRay(const Vec3fa& org, const Vec3fa& dir, float tnear, float tfar) 
  {
    RTCRayHit rh; clearRay(rh);
    rh.ray.org_x = org.x; rh.ray.org_y = org.y; rh.ray.org_z = org.z;
    rh.ray.dir_x = dir.x; rh.ray.dir_y = dir.y; rh.ray.dir_z = dir.z;
    rh.ray.tnear = tnear; rh.ray.tfar = tfar;
    rh.ray.time = 0; rh.ray.mask = -1;
    rh.hit.geomID = rh.hit.primID = -1;
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      rh.hit.instID[l] = RTC_INVALID_GEOMETRY_ID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      rh.hit.instPrimID[l] = RTC_INVALID_GEOMETRY_ID;
#endif
    }
    return rh;
  }

  __forceinline RTCRayHit fastMakeRay(const Vec3fa& org, const Vec3fa& dir) 
  {
    RTCRayHit rh;
    *(Vec3fa*)&rh.ray.org_x = org;
    *(Vec3fa*)&rh.ray.dir_x = dir;
    rh.ray.tnear = 0.0f; 
    rh.ray.tfar = inf;
    rh.ray.time = 0; 
    rh.ray.mask = -1;
    rh.ray.id = -1;
    rh.hit.geomID = rh.hit.primID = -1;
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      rh.hit.instID[l] = RTC_INVALID_GEOMETRY_ID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      rh.hit.instPrimID[l] = RTC_INVALID_GEOMETRY_ID;
#endif
    }
    return rh;
  }

  __forceinline RTCRayHit fastMakeRay(const Vec3fa& org, RandomSampler& sampler)
  {
    return fastMakeRay(org, 2.0f*RandomSampler_get3D(sampler)-Vec3fa(1.0f));
  }

  __forceinline void fastMakeRay(RTCRayHit& rh, const Vec3fa& org, const Vec3fa& dir) 
  {
    *(Vec3fa*)&rh.ray.org_x = org;
    *(Vec3fa*)&rh.ray.dir_x = dir;
    rh.ray.tnear = 0.0f; 
    rh.ray.tfar = inf;
    rh.ray.time = 0; 
    rh.ray.mask = -1;
    rh.ray.id = -1;
    rh.hit.geomID = rh.hit.primID = rh.hit.instID[0] = -1;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    rh.hit.instPrimID[0] = RTC_INVALID_GEOMETRY_ID;
#endif
  }

  __forceinline void fastMakeRay(RTCRayHit& ray, const Vec3fa& org, RandomSampler& sampler)
  {
    fastMakeRay(ray, org, 2.0f*RandomSampler_get3D(sampler)-Vec3fa(1.0f));
  }

  __forceinline RTCRayHit fastMakeRay(Vec3f org, Vec3f dir, float tnear, float tfar) 
  {
    RTCRayHit rh;
    rh.ray.org_x = org.x; rh.ray.org_y = org.y; rh.ray.org_z = org.z; // FIXME: optimize
    rh.ray.dir_x = dir.x; rh.ray.dir_y = dir.y; rh.ray.dir_z = dir.z;
    rh.ray.tnear = tnear; rh.ray.tfar = tfar;
    rh.ray.time = 0; rh.ray.mask = -1; rh.ray.id = -1;
    rh.hit.geomID = rh.hit.primID = -1;
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      rh.hit.instID[l] = RTC_INVALID_GEOMETRY_ID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      rh.hit.instPrimID[l] = RTC_INVALID_GEOMETRY_ID;
#endif
    }
    return rh;
  }

  __forceinline bool neq_ray_special (const RTCRayHit& ray0, const RTCRayHit& ray1)
  {
    if (*(int*)&ray0.ray.org_x != *(int*)&ray1.ray.org_x) return true;
    if (*(int*)&ray0.ray.org_y != *(int*)&ray1.ray.org_y) return true;
    if (*(int*)&ray0.ray.org_z != *(int*)&ray1.ray.org_z) return true;
    if (*(int*)&ray0.ray.dir_x != *(int*)&ray1.ray.dir_x) return true;
    if (*(int*)&ray0.ray.dir_y != *(int*)&ray1.ray.dir_y) return true;
    if (*(int*)&ray0.ray.dir_z != *(int*)&ray1.ray.dir_z) return true;
    if (*(int*)&ray0.ray.tnear  != *(int*)&ray1.ray.tnear ) return true;
    if (*(int*)&ray0.ray.tfar   != *(int*)&ray1.ray.tfar  ) return true;
    if (*(int*)&ray0.ray.time   != *(int*)&ray1.ray.time  ) return true;
    if (*(int*)&ray0.ray.mask   != *(int*)&ray1.ray.mask  ) return true;
    if (*(int*)&ray0.ray.id     != *(int*)&ray1.ray.id    ) return true;
    if (*(int*)&ray0.hit.u      != *(int*)&ray1.hit.u     ) return true;
    if (*(int*)&ray0.hit.v      != *(int*)&ray1.hit.v     ) return true;
    if (*(int*)&ray0.hit.instID != *(int*)&ray1.hit.instID) return true;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    if (*(int*)&ray0.hit.instPrimID != *(int*)&ray1.hit.instPrimID) return true;
#endif
    if (*(int*)&ray0.hit.geomID != *(int*)&ray1.hit.geomID) return true;
    if (*(int*)&ray0.hit.primID != *(int*)&ray1.hit.primID) return true;
    if (*(int*)&ray0.hit.Ng_x  != *(int*)&ray1.hit.Ng_x ) return true;
    if (*(int*)&ray0.hit.Ng_y  != *(int*)&ray1.hit.Ng_y ) return true;
    if (*(int*)&ray0.hit.Ng_z  != *(int*)&ray1.hit.Ng_z ) return true;
    return false;
  }

  /* Outputs ray to stream */
  __forceinline embree_ostream operator<<(embree_ostream cout, const RTCRayHit& rh)
  {
    cout << "Ray { " << embree_endl
         << "  org = " << rh.ray.org_x << " " << rh.ray.org_y << " " << rh.ray.org_z << embree_endl
         << "  dir = " << rh.ray.dir_x << " " << rh.ray.dir_y << " " << rh.ray.dir_z << embree_endl
         << "  near = " << rh.ray.tnear << embree_endl
         << "  far = " << rh.ray.tfar << embree_endl
         << "  time = " << rh.ray.time << embree_endl
         << "  mask = " << rh.ray.mask << embree_endl
         << "  id = " << rh.ray.id << embree_endl
         << "  instID =";
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l)
    {
      cout << " " << rh.hit.instID[l];
      if (rh.hit.instID[l] == RTC_INVALID_GEOMETRY_ID)
        break;
    }
    cout << embree_endl;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    cout << "  instPrimID =";
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l)
    {
      cout << " " << rh.hit.instPrimID[l];
      if (rh.hit.instID[l] == RTC_INVALID_GEOMETRY_ID)
        break;
    }
#endif
    cout << embree_endl
         << "  geomID = " << rh.hit.geomID << embree_endl
         << "  primID = " << rh.hit.primID <<  embree_endl
         << "  u = " << rh.hit.u <<  embree_endl
         << "  v = " << rh.hit.v << embree_endl
         << "  Ng = " << rh.hit.Ng_x << " " << rh.hit.Ng_y << " " << rh.hit.Ng_z << embree_endl
         << "}";
    return cout;
  }

  __forceinline void setRay(RTCRayHit4& ray_o, size_t i, const RTCRayHit& ray_i)
  {
    ray_o.ray.org_x[i] = ray_i.ray.org_x;
    ray_o.ray.org_y[i] = ray_i.ray.org_y;
    ray_o.ray.org_z[i] = ray_i.ray.org_z;
    ray_o.ray.dir_x[i] = ray_i.ray.dir_x;
    ray_o.ray.dir_y[i] = ray_i.ray.dir_y;
    ray_o.ray.dir_z[i] = ray_i.ray.dir_z;
    ray_o.ray.tnear[i] = ray_i.ray.tnear;
    ray_o.ray.tfar[i] = ray_i.ray.tfar;
    ray_o.ray.time[i] = ray_i.ray.time;
    ray_o.ray.mask[i] = ray_i.ray.mask;
    ray_o.ray.id[i] = ray_i.ray.id;
    ray_o.hit.geomID[i] = ray_i.hit.geomID;
    ray_o.hit.primID[i] = ray_i.hit.primID;
    ray_o.hit.u[i] = ray_i.hit.u;
    ray_o.hit.v[i] = ray_i.hit.v;
    ray_o.hit.Ng_x[i] = ray_i.hit.Ng_x;
    ray_o.hit.Ng_y[i] = ray_i.hit.Ng_y;
    ray_o.hit.Ng_z[i] = ray_i.hit.Ng_z;

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray_o.hit.instID[l][i] = ray_i.hit.instID[l];
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      ray_o.hit.instPrimID[l][i] = ray_i.hit.instPrimID[l];
#endif
    }
  }

  __forceinline void setRay(RTCRayHit8& ray_o, size_t i, const RTCRayHit& ray_i)
  {
    ray_o.ray.org_x[i] = ray_i.ray.org_x;
    ray_o.ray.org_y[i] = ray_i.ray.org_y;
    ray_o.ray.org_z[i] = ray_i.ray.org_z;
    ray_o.ray.dir_x[i] = ray_i.ray.dir_x;
    ray_o.ray.dir_y[i] = ray_i.ray.dir_y;
    ray_o.ray.dir_z[i] = ray_i.ray.dir_z;
    ray_o.ray.tnear[i] = ray_i.ray.tnear;
    ray_o.ray.tfar[i] = ray_i.ray.tfar;
    ray_o.ray.time[i] = ray_i.ray.time;
    ray_o.ray.mask[i] = ray_i.ray.mask;
    ray_o.ray.id[i] = ray_i.ray.id;
    ray_o.hit.geomID[i] = ray_i.hit.geomID;
    ray_o.hit.primID[i] = ray_i.hit.primID;
    ray_o.hit.u[i] = ray_i.hit.u;
    ray_o.hit.v[i] = ray_i.hit.v;
    ray_o.hit.Ng_x[i] = ray_i.hit.Ng_x;
    ray_o.hit.Ng_y[i] = ray_i.hit.Ng_y;
    ray_o.hit.Ng_z[i] = ray_i.hit.Ng_z;

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray_o.hit.instID[l][i] = ray_i.hit.instID[l];
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      ray_o.hit.instPrimID[l][i] = ray_i.hit.instPrimID[l];
#endif
    }
  }

  __forceinline void setRay(RTCRayHit16& ray_o, size_t i, const RTCRayHit& ray_i)
  {
    ray_o.ray.org_x[i] = ray_i.ray.org_x;
    ray_o.ray.org_y[i] = ray_i.ray.org_y;
    ray_o.ray.org_z[i] = ray_i.ray.org_z;
    ray_o.ray.dir_x[i] = ray_i.ray.dir_x;
    ray_o.ray.dir_y[i] = ray_i.ray.dir_y;
    ray_o.ray.dir_z[i] = ray_i.ray.dir_z;
    ray_o.ray.tnear[i] = ray_i.ray.tnear;
    ray_o.ray.tfar[i] = ray_i.ray.tfar;
    ray_o.ray.time[i] = ray_i.ray.time;
    ray_o.ray.mask[i] = ray_i.ray.mask;
    ray_o.ray.id[i] = ray_i.ray.id;
    ray_o.hit.geomID[i] = ray_i.hit.geomID;
    ray_o.hit.primID[i] = ray_i.hit.primID;
    ray_o.hit.u[i] = ray_i.hit.u;
    ray_o.hit.v[i] = ray_i.hit.v;
    ray_o.hit.Ng_x[i] = ray_i.hit.Ng_x;
    ray_o.hit.Ng_y[i] = ray_i.hit.Ng_y;
    ray_o.hit.Ng_z[i] = ray_i.hit.Ng_z;

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray_o.hit.instID[l][i] = ray_i.hit.instID[l];
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      ray_o.hit.instPrimID[l][i] = ray_i.hit.instPrimID[l];
#endif
    }
  }

  __forceinline void setRay(RTCRayHitN* rayhit_o, unsigned int N, unsigned int i, const RTCRayHit& ray_i)
  {
    RTCRayN* ray_o = RTCRayHitN_RayN(rayhit_o,N);
    RTCRayN_org_x(ray_o,N,i) = ray_i.ray.org_x;
    RTCRayN_org_y(ray_o,N,i) = ray_i.ray.org_y;
    RTCRayN_org_z(ray_o,N,i) = ray_i.ray.org_z;
    RTCRayN_dir_x(ray_o,N,i) = ray_i.ray.dir_x;
    RTCRayN_dir_y(ray_o,N,i) = ray_i.ray.dir_y;
    RTCRayN_dir_z(ray_o,N,i) = ray_i.ray.dir_z;
    RTCRayN_tnear(ray_o,N,i) = ray_i.ray.tnear;
    RTCRayN_tfar(ray_o,N,i) = ray_i.ray.tfar;
    RTCRayN_time(ray_o,N,i) = ray_i.ray.time;
    RTCRayN_mask(ray_o,N,i) = ray_i.ray.mask;
    RTCRayN_id(ray_o,N,i) = ray_i.ray.id;
    RTCHitN* hit_o = RTCRayHitN_HitN(rayhit_o,N);
    RTCHitN_geomID(hit_o,N,i) = ray_i.hit.geomID;
    RTCHitN_primID(hit_o,N,i) = ray_i.hit.primID;
    RTCHitN_u(hit_o,N,i) = ray_i.hit.u;
    RTCHitN_v(hit_o,N,i) = ray_i.hit.v;
    RTCHitN_Ng_x(hit_o,N,i) = ray_i.hit.Ng_x;
    RTCHitN_Ng_y(hit_o,N,i) = ray_i.hit.Ng_y;
    RTCHitN_Ng_z(hit_o,N,i) = ray_i.hit.Ng_z;

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      RTCHitN_instID(hit_o,N,i,l) = ray_i.hit.instID[l];
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      RTCHitN_instPrimID(hit_o,N,i,l) = ray_i.hit.instPrimID[l];
#endif
    }
  }

  __forceinline RTCRayHit getRay(RTCRayHit4& ray_i, size_t i)
  {
    RTCRayHit ray_o;
    ray_o.ray.org_x = ray_i.ray.org_x[i];
    ray_o.ray.org_y = ray_i.ray.org_y[i];
    ray_o.ray.org_z = ray_i.ray.org_z[i];
    ray_o.ray.dir_x = ray_i.ray.dir_x[i];
    ray_o.ray.dir_y = ray_i.ray.dir_y[i];
    ray_o.ray.dir_z = ray_i.ray.dir_z[i];
    ray_o.ray.tnear = ray_i.ray.tnear[i];
    ray_o.ray.tfar = ray_i.ray.tfar[i];
    ray_o.ray.time = ray_i.ray.time[i];
    ray_o.ray.mask = ray_i.ray.mask[i];
    ray_o.ray.id = ray_i.ray.id[i];
    ray_o.hit.geomID = ray_i.hit.geomID[i];
    ray_o.hit.primID = ray_i.hit.primID[i];
    ray_o.hit.u = ray_i.hit.u[i];
    ray_o.hit.v = ray_i.hit.v[i];
    ray_o.hit.Ng_x = ray_i.hit.Ng_x[i];
    ray_o.hit.Ng_y = ray_i.hit.Ng_y[i];
    ray_o.hit.Ng_z = ray_i.hit.Ng_z[i];

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray_o.hit.instID[l] = ray_i.hit.instID[l][i];
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      ray_o.hit.instPrimID[l] = ray_i.hit.instPrimID[l][i];
#endif
    }

    return ray_o;
  }

  __forceinline RTCRayHit getRay(RTCRayHit8& ray_i, size_t i)
  {
    RTCRayHit ray_o;
    ray_o.ray.org_x = ray_i.ray.org_x[i];
    ray_o.ray.org_y = ray_i.ray.org_y[i];
    ray_o.ray.org_z = ray_i.ray.org_z[i];
    ray_o.ray.dir_x = ray_i.ray.dir_x[i];
    ray_o.ray.dir_y = ray_i.ray.dir_y[i];
    ray_o.ray.dir_z = ray_i.ray.dir_z[i];
    ray_o.ray.tnear = ray_i.ray.tnear[i];
    ray_o.ray.tfar = ray_i.ray.tfar[i];
    ray_o.ray.time = ray_i.ray.time[i];
    ray_o.ray.mask = ray_i.ray.mask[i];
    ray_o.ray.id = ray_i.ray.id[i];
    ray_o.hit.geomID = ray_i.hit.geomID[i];
    ray_o.hit.primID = ray_i.hit.primID[i];
    ray_o.hit.u = ray_i.hit.u[i];
    ray_o.hit.v = ray_i.hit.v[i];
    ray_o.hit.Ng_x = ray_i.hit.Ng_x[i];
    ray_o.hit.Ng_y = ray_i.hit.Ng_y[i];
    ray_o.hit.Ng_z = ray_i.hit.Ng_z[i];

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray_o.hit.instID[l] = ray_i.hit.instID[l][i];
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      ray_o.hit.instPrimID[l] = ray_i.hit.instPrimID[l][i];
#endif
    }

    return ray_o;
  }

  __forceinline RTCRayHit getRay(RTCRayHit16& ray_i, size_t i)
  {
    RTCRayHit ray_o;
    ray_o.ray.org_x = ray_i.ray.org_x[i];
    ray_o.ray.org_y = ray_i.ray.org_y[i];
    ray_o.ray.org_z = ray_i.ray.org_z[i];
    ray_o.ray.dir_x = ray_i.ray.dir_x[i];
    ray_o.ray.dir_y = ray_i.ray.dir_y[i];
    ray_o.ray.dir_z = ray_i.ray.dir_z[i];
    ray_o.ray.tnear = ray_i.ray.tnear[i];
    ray_o.ray.tfar = ray_i.ray.tfar[i];
    ray_o.ray.time = ray_i.ray.time[i];
    ray_o.ray.mask = ray_i.ray.mask[i];
    ray_o.ray.id = ray_i.ray.id[i];
    ray_o.hit.geomID = ray_i.hit.geomID[i];
    ray_o.hit.primID = ray_i.hit.primID[i];
    ray_o.hit.u = ray_i.hit.u[i];
    ray_o.hit.v = ray_i.hit.v[i];
    ray_o.hit.Ng_x = ray_i.hit.Ng_x[i];
    ray_o.hit.Ng_y = ray_i.hit.Ng_y[i];
    ray_o.hit.Ng_z = ray_i.hit.Ng_z[i];

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray_o.hit.instID[l] = ray_i.hit.instID[l][i];
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      ray_o.hit.instPrimID[l] = ray_i.hit.instPrimID[l][i];
#endif
    }
    return ray_o;
  }

  __forceinline RTCRayHit getRay(RTCRayHitN* rayhit_i, unsigned int N, unsigned int i)
  {
    RTCRayHit ray_o;
    RTCRayN* ray_i = RTCRayHitN_RayN(rayhit_i,N);
    RTCHitN* hit_i = RTCRayHitN_HitN(rayhit_i,N);
    ray_o.ray.org_x = RTCRayN_org_x(ray_i,N,i);
    ray_o.ray.org_y = RTCRayN_org_y(ray_i,N,i);
    ray_o.ray.org_z = RTCRayN_org_z(ray_i,N,i);
    ray_o.ray.dir_x = RTCRayN_dir_x(ray_i,N,i);
    ray_o.ray.dir_y = RTCRayN_dir_y(ray_i,N,i);
    ray_o.ray.dir_z = RTCRayN_dir_z(ray_i,N,i);
    ray_o.ray.tnear = RTCRayN_tnear(ray_i,N,i);
    ray_o.ray.tfar  = RTCRayN_tfar(ray_i,N,i);
    ray_o.ray.time = RTCRayN_time(ray_i,N,i);
    ray_o.ray.mask = RTCRayN_mask(ray_i,N,i);
    ray_o.ray.id = RTCRayN_id(ray_i,N,i);
    ray_o.hit.geomID = RTCHitN_geomID(hit_i,N,i);
    ray_o.hit.primID = RTCHitN_primID(hit_i,N,i);
    ray_o.hit.u = RTCHitN_u(hit_i,N,i);
    ray_o.hit.v = RTCHitN_v(hit_i,N,i);
    ray_o.hit.Ng_x = RTCHitN_Ng_x(hit_i,N,i);
    ray_o.hit.Ng_y = RTCHitN_Ng_y(hit_i,N,i);
    ray_o.hit.Ng_z = RTCHitN_Ng_z(hit_i,N,i);

    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray_o.hit.instID[l] = RTCHitN_instID(hit_i, N, i, l);
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      ray_o.hit.instPrimID[l] = RTCHitN_instPrimID(hit_i, N, i, l);
#endif
    }

    return ray_o;
  }

  enum IntersectMode 
  {
    MODE_INTERSECT_NONE,
    MODE_INTERSECT1,
    MODE_INTERSECT4,
    MODE_INTERSECT8,
    MODE_INTERSECT16
  };

  inline std::string to_string(IntersectMode imode)
  {
    switch (imode) {
    case MODE_INTERSECT_NONE: return "None";
    case MODE_INTERSECT1: return "1";
    case MODE_INTERSECT4: return "4";
    case MODE_INTERSECT8: return "8";
    case MODE_INTERSECT16: return "16";
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
    VARIANT_INTERSECT_OCCLUDED_COHERENT = 3, // intersect but verify if occluded also finds hit or not
    VARIANT_INTERSECT_OCCLUDED_INCOHERENT = 7, // intersect but verify if occluded also finds hit or not
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

  inline std::string to_string(RTCSceneFlags scene_flags)
  {
    std::string ret;
    if (scene_flags & RTC_SCENE_FLAG_DYNAMIC) ret += "Dynamic";
    else ret += "Static";
    if (scene_flags & RTC_SCENE_FLAG_COMPACT) ret += "Compact";
    if (scene_flags & RTC_SCENE_FLAG_ROBUST ) ret += "Robust";
    if (!(scene_flags & RTC_SCENE_FLAG_COMPACT) && !(scene_flags & RTC_SCENE_FLAG_ROBUST)) ret += "Fast"; 
    return ret;
  }
  
  inline std::string to_string(RTCBuildQuality quality_flags)
  {
    if      (quality_flags == RTC_BUILD_QUALITY_LOW   ) return "LowQuality";
    else if (quality_flags == RTC_BUILD_QUALITY_MEDIUM) return "MediumQuality";
    else if (quality_flags == RTC_BUILD_QUALITY_HIGH  ) return "HighQuality";
    else if (quality_flags == RTC_BUILD_QUALITY_REFIT ) return "RefitQuality";
    else { assert(false); return ""; }
  }

  struct SceneFlags
  {
    SceneFlags (RTCSceneFlags sflags, RTCBuildQuality qflags)
    : sflags(sflags), qflags(qflags) {}

    RTCSceneFlags sflags;
    RTCBuildQuality qflags;
  };
  
  inline std::string to_string(SceneFlags sflags) {
    return to_string(sflags.sflags) + "." + to_string(sflags.qflags);
  }

  inline std::string to_string(SceneFlags sflags, RTCBuildQuality quality) {
    return to_string(sflags)+to_string(quality);
  }

  static const size_t numSceneFlags = 8*3;

  SceneFlags getSceneFlags(size_t i) {
    i = i % numSceneFlags;
    return SceneFlags((RTCSceneFlags)(i&7),(RTCBuildQuality)(i>>3));
  }

  static const size_t numSceneGeomFlags = 32;

  __noinline void IntersectWithModeInternal(IntersectMode mode, IntersectVariant ivariant, RTCScene scene, RTCRayHit* rays, unsigned int N, RTCIntersectArguments* args)
  {
    RTCIntersectArguments _args;
    if (!args)
    {
      rtcInitIntersectArguments(&_args);
      args = &_args;
    }

    RTCRayQueryFlags flags = ((ivariant & VARIANT_COHERENT_INCOHERENT_MASK) == VARIANT_COHERENT) ? RTC_RAY_QUERY_FLAG_COHERENT :  RTC_RAY_QUERY_FLAG_INCOHERENT;
    args->flags = (RTCRayQueryFlags) (args->flags | flags);
    
    switch (mode) 
    {
    case MODE_INTERSECT_NONE: 
      break;
    case MODE_INTERSECT1:
    {
      switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
      case VARIANT_INTERSECT: for (size_t i=0; i<N; i++) rtcIntersect1(scene,&rays[i],args); break;
      case VARIANT_OCCLUDED : for (size_t i=0; i<N; i++) rtcOccluded1 (scene,(RTCRay*)&rays[i],(RTCOccludedArguments*)args); break;
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
        __aligned(16) RTCRayHit4 ray4;
        for (size_t j=0; j<4; j++) valid[j] = (j<M && rays[i+j].ray.tnear <= rays[i+j].ray.tfar) ? -1 : 0;
        for (size_t j=0; j<M; j++) setRay(ray4,j,rays[i+j]);
        for (size_t j=M; j<4; j++) setRay(ray4,j,makeRay(zero,zero,pos_inf,neg_inf));
        switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
        case VARIANT_INTERSECT: rtcIntersect4(valid,scene,&ray4,args); break;
        case VARIANT_OCCLUDED : rtcOccluded4 (valid,scene,(RTCRay4*)&ray4,(RTCOccludedArguments*)args); break;
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
        __aligned(32) RTCRayHit8 ray8;
        for (size_t j=0; j<8; j++) valid[j] = (j<M && rays[i+j].ray.tnear <= rays[i+j].ray.tfar) ? -1 : 0;
        for (size_t j=0; j<M; j++) setRay(ray8,j,rays[i+j]);
        for (size_t j=M; j<8; j++) setRay(ray8,j,makeRay(zero,zero,pos_inf,neg_inf));
        switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
        case VARIANT_INTERSECT: rtcIntersect8(valid,scene,&ray8,args); break;
        case VARIANT_OCCLUDED : rtcOccluded8 (valid,scene,(RTCRay8*)&ray8,(RTCOccludedArguments*)args); break;
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
        __aligned(64) RTCRayHit16 ray16;
        for (size_t j=0; j<16; j++) valid[j] = (j<M && rays[i+j].ray.tnear <= rays[i+j].ray.tfar) ? -1 : 0;
        for (size_t j=0; j<M ; j++) setRay(ray16,j,rays[i+j]);
        for (size_t j=M; j<16; j++) setRay(ray16,j,makeRay(zero,zero,pos_inf,neg_inf));
        switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
        case VARIANT_INTERSECT: rtcIntersect16(valid,scene,&ray16,args); break;
        case VARIANT_OCCLUDED : rtcOccluded16 (valid,scene,(RTCRay16*)&ray16,(RTCOccludedArguments*)args); break;
        default: assert(false);
        }
        for (size_t j=0; j<M; j++) rays[i+j] = getRay(ray16,j);
      }
      break;
    }
    }
  }

  void IntersectWithMode(IntersectMode mode, IntersectVariant ivariant, RTCScene scene, RTCRayHit* rays, unsigned int N, RTCIntersectArguments* args = nullptr)
  {
    /* verify occluded result against intersect */
    if ((ivariant & VARIANT_INTERSECT_OCCLUDED) == VARIANT_INTERSECT_OCCLUDED)
    {
      vector<bool> valid(N);
      vector_t<RTCRayHit,aligned_allocator<RTCRayHit,16>> rays2(N);
      for (size_t i=0; i<N; i++) {
        valid[i] = rays[i].ray.tnear <= rays[i].ray.tfar;
        rays2[i] = rays[i];
      }
      IntersectWithModeInternal(mode,IntersectVariant(ivariant & ~VARIANT_OCCLUDED),scene,rays,N,args);
      IntersectWithModeInternal(mode,IntersectVariant(ivariant & ~VARIANT_INTERSECT),scene,rays2.data(),N,args);
      for (size_t i=0; i<N; i++)
      {
        if (valid[i] && ((rays[i].hit.geomID == RTC_INVALID_GEOMETRY_ID) != (rays2[i].ray.tfar != float(neg_inf)))) {
          throw std::runtime_error("Intersect/Occluded mismatch");
        }
      }
    }
    else
      IntersectWithModeInternal(mode,ivariant,scene,rays,N,args);
  }

  enum GeometryType
  {
    TRIANGLE_MESH,
    TRIANGLE_MESH_MB,
    QUAD_MESH,
    QUAD_MESH_MB,
    GRID_MESH,
    GRID_MESH_MB,
    SUBDIV_MESH,
    SUBDIV_MESH_MB,
    BEZIER_GEOMETRY,
    BEZIER_GEOMETRY_MB,
    BSPLINE_GEOMETRY,
    BSPLINE_GEOMETRY_MB,
    CATMULL_GEOMETRY,
    CATMULL_GEOMETRY_MB,
    LINE_GEOMETRY,
    LINE_GEOMETRY_MB,
    SPHERE_GEOMETRY,
    SPHERE_GEOMETRY_MB,
    DISC_GEOMETRY,
    DISC_GEOMETRY_MB,
    ORIENTED_DISC_GEOMETRY,
    ORIENTED_DISC_GEOMETRY_MB
  };

  inline std::string to_string(GeometryType gtype)
  {
    switch (gtype) {
    case TRIANGLE_MESH    : return "triangles";
    case TRIANGLE_MESH_MB : return "triangles_mb";
    case QUAD_MESH        : return "quads";
    case QUAD_MESH_MB     : return "quads_mb";
    case GRID_MESH        : return "grids";
    case GRID_MESH_MB     : return "grids_mb";
    case SUBDIV_MESH      : return "subdivs";
    case SUBDIV_MESH_MB   : return "subdivs_mb";
    case BEZIER_GEOMETRY  : return "bezier";
    case BEZIER_GEOMETRY_MB: return "bezier_mb";
    case BSPLINE_GEOMETRY : return "bspline";
    case BSPLINE_GEOMETRY_MB: return "bspline_mb";
    case CATMULL_GEOMETRY : return "catmull";
    case CATMULL_GEOMETRY_MB : return "catmull_mb";
    case LINE_GEOMETRY    : return "lines";
    case LINE_GEOMETRY_MB : return "lines_mb";
    case SPHERE_GEOMETRY          : return "spheres";
    case SPHERE_GEOMETRY_MB       : return "spheres_mb";
    case DISC_GEOMETRY            : return "disc";
    case DISC_GEOMETRY_MB         : return "disc_mb";
    case ORIENTED_DISC_GEOMETRY   : return "oriented_disc";
    case ORIENTED_DISC_GEOMETRY_MB: return "oriented_disc_mb";
    }
    return "";
  }

  inline std::string to_string(SceneFlags sflags, IntersectMode imode) {
    return to_string(sflags) + "." + to_string(imode);
  }

  inline std::string to_string(SceneFlags sflags, IntersectMode imode, IntersectVariant ivariant) {
    return to_string(sflags) + "." + to_string(imode,ivariant);
  }

  inline std::string to_string(GeometryType gtype, SceneFlags sflags, IntersectMode imode, IntersectVariant ivariant) {
    return to_string(gtype) + "." + to_string(sflags) + "." + to_string(imode,ivariant);
  }

  /* error reporting function */
  void error_handler(void* userPtr, const RTCError code, const char* str = nullptr)
  {
    if (code == RTC_ERROR_NONE) 
      return;
    
    std::string errorStr;
    errorStr += "Embree: ";
    errorStr += string_of(code);
    if (str) errorStr += " (" + std::string(str) + ")";
    throw std::runtime_error(errorStr);
  }
}

