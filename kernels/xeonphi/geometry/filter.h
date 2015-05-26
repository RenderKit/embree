// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "../../common/geometry.h"
#include "../../common/ray.h"
#include "../../common/ray16.h"

namespace embree
{
  typedef void (*ISPCFilterFunc16)(void* ptr, RTCRay16& ray, __mmask16 valid);

  __forceinline bool runIntersectionFilter1(const Geometry* const geometry, Ray& ray, 
                                            const float16& u, const float16& v, const float16& t, const float16& Ngx, const float16& Ngy, const float16& Ngz, const bool16 wmask, 
                                            const int geomID, const int primID)
  {
 

    /* temporarily update hit information */
    const float  ray_tfar = ray.tfar;
    const Vec3fa ray_Ng   = ray.Ng;
    const Vec3fa ray_uv_ids = *(Vec3fa*)&ray.u;
    compactustore16f_low(wmask,&ray.tfar,t);
    compactustore16f_low(wmask,&ray.u,u); 
    compactustore16f_low(wmask,&ray.v,v); 
    compactustore16f_low(wmask,&ray.Ng.x,Ngx); 
    compactustore16f_low(wmask,&ray.Ng.y,Ngy); 
    compactustore16f_low(wmask,&ray.Ng.z,Ngz);
    ray.geomID = geomID;
    ray.primID = primID;

    /* invoke filter function */
    geometry->intersectionFilter1(geometry->userPtr,(RTCRay&)ray);
    
    /* restore hit if filter not passed */
    if (unlikely(ray.geomID == -1)) 
    {
      ray.tfar = ray_tfar;
      ray.Ng = ray_Ng;
      *(Vec3fa*)&ray.u = ray_uv_ids;
      return false;
    }
    return true;
  }

  __forceinline bool runOcclusionFilter1(const Geometry* const geometry, Ray& ray, 
                                         const float16& u, const float16& v, const float16& t, const float16& Ngx, const float16& Ngy, const float16& Ngz, const bool16 wmask, 
                                         const int geomID, const int primID)
  {
    /* temporarily update hit information */
    const float ray_tfar = ray.tfar;
    const int   ray_geomID = ray.geomID;
    compactustore16f_low(wmask,&ray.tfar,t);
    compactustore16f_low(wmask,&ray.u,u); 
    compactustore16f_low(wmask,&ray.v,v); 
    compactustore16f_low(wmask,&ray.Ng.x,Ngx); 
    compactustore16f_low(wmask,&ray.Ng.y,Ngy); 
    compactustore16f_low(wmask,&ray.Ng.z,Ngz);
    ray.geomID = geomID;
    ray.primID = primID;

    /* invoke filter function */
    geometry->occlusionFilter1(geometry->userPtr,(RTCRay&)ray);
    
    /* restore hit if filter not passed */
    if (unlikely(ray.geomID == -1)) 
    {
      ray.tfar = ray_tfar;
      ray.geomID = ray_geomID;
      return false;
    }
    return true;
  }

  __forceinline bool16 runIntersectionFilter16(const bool16& valid, const Geometry* const geometry, Ray16& ray, 
                                              const float16& u, const float16& v, const float16& t, const Vec3f16& Ng, const int16& geomID, const int16& primID)
  {
    /* temporarily update hit information */
    const float16 ray_u = ray.u;           store16f(valid,&ray.u,u);
    const float16 ray_v = ray.v;           store16f(valid,&ray.v,v);
    const float16 ray_tfar = ray.tfar;     store16f(valid,&ray.tfar,t);
    const int16 ray_geomID = ray.geomID; store16i(valid,&ray.geomID,geomID);
    const int16 ray_primID = ray.primID; store16i(valid,&ray.primID,primID);
    const float16 ray_Ng_x = ray.Ng.x;     store16f(valid,&ray.Ng.x,Ng.x);
    const float16 ray_Ng_y = ray.Ng.y;     store16f(valid,&ray.Ng.y,Ng.y);
    const float16 ray_Ng_z = ray.Ng.z;     store16f(valid,&ray.Ng.z,Ng.z);

    /* invoke filter function */
    RTCFilterFunc16  filter16     = (RTCFilterFunc16)  geometry->intersectionFilter16;
    ISPCFilterFunc16 ispcFilter16 = (ISPCFilterFunc16) geometry->intersectionFilter16;
    if (geometry->ispcIntersectionFilter16) ispcFilter16(geometry->userPtr,(RTCRay16&)ray,valid);
    else filter16(&valid,geometry->userPtr,(RTCRay16&)ray);
    const bool16 valid_failed = valid & (ray.geomID == int16(-1));
    const bool16 valid_passed = valid & (ray.geomID != int16(-1));

    /* restore hit if filter not passed */
    if (unlikely(any(valid_failed))) 
    {
      store16f(valid_failed,&ray.u,ray_u);
      store16f(valid_failed,&ray.v,ray_v);
      store16f(valid_failed,&ray.tfar,ray_tfar);
      store16i(valid_failed,&ray.geomID,ray_geomID);
      store16i(valid_failed,&ray.primID,ray_primID);
      store16f(valid_failed,&ray.Ng.x,ray_Ng_x);
      store16f(valid_failed,&ray.Ng.y,ray_Ng_y);
      store16f(valid_failed,&ray.Ng.z,ray_Ng_z);
    }
    return valid_passed;
  }

  __forceinline bool16 runOcclusionFilter16(const bool16& valid, const Geometry* const geometry, Ray16& ray, 
                                           const float16& u, const float16& v, const float16& t, const Vec3f16& Ng, const int16& geomID, const int16& primID)
  {
    /* temporarily update hit information */
    const float16 ray_tfar = ray.tfar; 
    const int16 ray_geomID = ray.geomID;
    store16f(valid,&ray.u,u);
    store16f(valid,&ray.v,v);
    store16f(valid,&ray.tfar,t);
    store16i(valid,&ray.geomID,geomID);
    store16i(valid,&ray.primID,primID);
    store16f(valid,&ray.Ng.x,Ng.x);
    store16f(valid,&ray.Ng.y,Ng.y);
    store16f(valid,&ray.Ng.z,Ng.z);

    /* invoke filter function */
    RTCFilterFunc16  filter16     = (RTCFilterFunc16)  geometry->occlusionFilter16;
    ISPCFilterFunc16 ispcFilter16 = (ISPCFilterFunc16) geometry->occlusionFilter16;
    if (geometry->ispcOcclusionFilter16) ispcFilter16(geometry->userPtr,(RTCRay16&)ray,valid);
    else filter16(&valid,geometry->userPtr,(RTCRay16&)ray);
    const bool16 valid_failed = valid & (ray.geomID == int16(-1));
    const bool16 valid_passed = valid & (ray.geomID != int16(-1));

    /* restore hit if filter not passed */
    store16f(valid_failed,&ray.tfar,ray_tfar);
    store16i(valid_failed,&ray.geomID,ray_geomID);
    return valid_passed;
  }

  __forceinline bool runIntersectionFilter16(const Geometry* const geometry, Ray16& ray, const size_t k,
                                             const float16& u, const float16& v, const float16& t, const float16& Ngx, const float16& Ngy, const float16& Ngz, const bool16 wmask, 
                                             const int geomID, const int primID)
  {
    /* temporarily update hit information */
    const float16 ray_u = ray.u;           compactustore16f_low(wmask,&ray.u[k],u); 
    const float16 ray_v = ray.v;           compactustore16f_low(wmask,&ray.v[k],v); 
    const float16 ray_tfar = ray.tfar;     compactustore16f_low(wmask,&ray.tfar[k],t);
    const int16 ray_geomID = ray.geomID; ray.geomID[k] = geomID;
    const int16 ray_primID = ray.primID; ray.primID[k] = primID;
    const float16 ray_Ng_x = ray.Ng.x;     compactustore16f_low(wmask,&ray.Ng.x[k],Ngx);
    const float16 ray_Ng_y = ray.Ng.y;     compactustore16f_low(wmask,&ray.Ng.y[k],Ngy);
    const float16 ray_Ng_z = ray.Ng.z;     compactustore16f_low(wmask,&ray.Ng.z[k],Ngz);

    /* invoke filter function */
    const bool16 valid(1 << k);
    RTCFilterFunc16  filter16     = (RTCFilterFunc16)  geometry->intersectionFilter16;
    ISPCFilterFunc16 ispcFilter16 = (ISPCFilterFunc16) geometry->intersectionFilter16;
    if (geometry->ispcIntersectionFilter16) ispcFilter16(geometry->userPtr,(RTCRay16&)ray,valid);
    else filter16(&valid,geometry->userPtr,(RTCRay16&)ray);
    const bool passed = ray.geomID[k] != -1;

    /* restore hit if filter not passed */
    if (unlikely(!passed)) {
      store16f(&ray.u,ray_u);
      store16f(&ray.v,ray_v);
      store16f(&ray.tfar,ray_tfar);
      store16i(&ray.geomID,ray_geomID);
      store16i(&ray.primID,ray_primID);
      store16f(&ray.Ng.x,ray_Ng_x);
      store16f(&ray.Ng.y,ray_Ng_y);
      store16f(&ray.Ng.z,ray_Ng_z);
    }
    return passed;
  }

  __forceinline bool runOcclusionFilter16(const Geometry* const geometry, Ray16& ray, const size_t k,
                                          const float16& u, const float16& v, const float16& t, const float16& Ngx, const float16& Ngy, const float16& Ngz, const bool16 wmask, 
                                          const int geomID, const int primID)
  {
    /* temporarily update hit information */
    const float16 ray_tfar = ray.tfar; 
    const int16 ray_geomID = ray.geomID;
    compactustore16f_low(wmask,&ray.u[k],u); 
    compactustore16f_low(wmask,&ray.v[k],v); 
    compactustore16f_low(wmask,&ray.tfar[k],t);
    ray.geomID[k] = geomID;
    ray.primID[k] = primID;
    compactustore16f_low(wmask,&ray.Ng.x[k],Ngx);
    compactustore16f_low(wmask,&ray.Ng.y[k],Ngy);
    compactustore16f_low(wmask,&ray.Ng.z[k],Ngz);

    /* invoke filter function */
    const bool16 valid(1 << k);
    RTCFilterFunc16  filter16     = (RTCFilterFunc16)  geometry->occlusionFilter16;
    ISPCFilterFunc16 ispcFilter16 = (ISPCFilterFunc16) geometry->occlusionFilter16;
    if (geometry->ispcOcclusionFilter16) ispcFilter16(geometry->userPtr,(RTCRay16&)ray,valid);
    else filter16(&valid,geometry->userPtr,(RTCRay16&)ray);
    const bool passed = ray.geomID[k] != -1;

    /* restore hit if filter not passed */
    if (unlikely(!passed)) {
      store16f(&ray.tfar,ray_tfar);
      store16i(&ray.geomID,ray_geomID);
    }
    return passed;
  }
}
