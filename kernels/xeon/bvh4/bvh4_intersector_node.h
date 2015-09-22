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

#include "bvh4.h"

namespace embree
{
  namespace isa
  {
    struct BVH4TravRay 
    {
      __forceinline BVH4TravRay () {}

      __forceinline BVH4TravRay(const Vec3fa& ray_org, const Vec3fa& ray_dir) 
        : org_xyz(ray_org), dir_xyz(ray_dir) 
      {
        const Vec3fa ray_rdir = rcp_safe(ray_dir);
        const Vec3fa ray_org_rdir = ray_org*ray_rdir;
        org = Vec3vf4(ray_org.x,ray_org.y,ray_org.z);
        dir = Vec3vf4(ray_dir.x,ray_dir.y,ray_dir.z);
        rdir = Vec3vf4(ray_rdir.x,ray_rdir.y,ray_rdir.z);
        org_rdir = Vec3vf4(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
        nearX = ray_rdir.x >= 0.0f ? 0*sizeof(vfloat4) : 1*sizeof(vfloat4);
        nearY = ray_rdir.y >= 0.0f ? 2*sizeof(vfloat4) : 3*sizeof(vfloat4);
        nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(vfloat4) : 5*sizeof(vfloat4);
        farX  = nearX ^ sizeof(vfloat4);
        farY  = nearY ^ sizeof(vfloat4);
        farZ  = nearZ ^ sizeof(vfloat4);
      }
      Vec3fa org_xyz, dir_xyz; // FIXME: store somewhere else
      Vec3vf4 org, dir, rdir, org_rdir; // FIXME: is org_rdir optimized away?
      size_t nearX, nearY, nearZ;
      size_t farX, farY, farZ;
    };

    /*! intersection with single rays */
    template<bool robust>
      __forceinline size_t intersect_node(const BVH4::Node* node, size_t nearX, size_t nearY, size_t nearZ,
                                          const Vec3vf4& org, const Vec3vf4& rdir, const Vec3vf4& org_rdir, const vfloat4& tnear, const vfloat4& tfar, 
                                          vfloat4& dist) 
    {
      const size_t farX  = nearX ^ sizeof(vfloat4), farY  = nearY ^ sizeof(vfloat4), farZ  = nearZ ^ sizeof(vfloat4);
#if defined (__AVX2__)
      const vfloat4 tNearX = msub(load4f((const char*)&node->lower_x+nearX), rdir.x, org_rdir.x);
      const vfloat4 tNearY = msub(load4f((const char*)&node->lower_x+nearY), rdir.y, org_rdir.y);
      const vfloat4 tNearZ = msub(load4f((const char*)&node->lower_x+nearZ), rdir.z, org_rdir.z);
      const vfloat4 tFarX  = msub(load4f((const char*)&node->lower_x+farX ), rdir.x, org_rdir.x);
      const vfloat4 tFarY  = msub(load4f((const char*)&node->lower_x+farY ), rdir.y, org_rdir.y);
      const vfloat4 tFarZ  = msub(load4f((const char*)&node->lower_x+farZ ), rdir.z, org_rdir.z);
#else
      const vfloat4 tNearX = (load4f((const char*)&node->lower_x+nearX) - org.x) * rdir.x;
      const vfloat4 tNearY = (load4f((const char*)&node->lower_x+nearY) - org.y) * rdir.y;
      const vfloat4 tNearZ = (load4f((const char*)&node->lower_x+nearZ) - org.z) * rdir.z;
      const vfloat4 tFarX  = (load4f((const char*)&node->lower_x+farX ) - org.x) * rdir.x;
      const vfloat4 tFarY  = (load4f((const char*)&node->lower_x+farY ) - org.y) * rdir.y;
      const vfloat4 tFarZ  = (load4f((const char*)&node->lower_x+farZ ) - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat4 tNear = max(tNearX,tNearY,tNearZ,tnear);
        const vfloat4 tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const vbool4 vmask = (round_down*tNear <= round_up*tFar);
        const size_t mask = movemask(vmask);
        dist = tNear;
        return mask;
      }
      
#if defined(__SSE4_1__)
      const vfloat4 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
      const vfloat4 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
      const vbool4 vmask = cast(tNear) > cast(tFar);
      const size_t mask = movemask(vmask)^0xf;
#else
      const vfloat4 tNear = max(tNearX,tNearY,tNearZ,tnear);
      const vfloat4 tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
      const vbool4 vmask = tNear <= tFar;
      const size_t mask = movemask(vmask);
#endif
      dist = tNear;
      return mask;
    }

    /*! intersection with single rays */
    template<bool robust>
      __forceinline size_t intersect_node(const BVH4::Node* node, const BVH4TravRay& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist) 
    {
#if defined (__AVX2__)
      const vfloat4 tNearX = msub(load4f((const char*)&node->lower_x+ray.nearX), ray.rdir.x, ray.org_rdir.x);
      const vfloat4 tNearY = msub(load4f((const char*)&node->lower_x+ray.nearY), ray.rdir.y, ray.org_rdir.y);
      const vfloat4 tNearZ = msub(load4f((const char*)&node->lower_x+ray.nearZ), ray.rdir.z, ray.org_rdir.z);
      const vfloat4 tFarX  = msub(load4f((const char*)&node->lower_x+ray.farX ), ray.rdir.x, ray.org_rdir.x);
      const vfloat4 tFarY  = msub(load4f((const char*)&node->lower_x+ray.farY ), ray.rdir.y, ray.org_rdir.y);
      const vfloat4 tFarZ  = msub(load4f((const char*)&node->lower_x+ray.farZ ), ray.rdir.z, ray.org_rdir.z);
#else
      const vfloat4 tNearX = (load4f((const char*)&node->lower_x+ray.nearX) - ray.org.x) * ray.rdir.x;
      const vfloat4 tNearY = (load4f((const char*)&node->lower_x+ray.nearY) - ray.org.y) * ray.rdir.y;
      const vfloat4 tNearZ = (load4f((const char*)&node->lower_x+ray.nearZ) - ray.org.z) * ray.rdir.z;
      const vfloat4 tFarX  = (load4f((const char*)&node->lower_x+ray.farX ) - ray.org.x) * ray.rdir.x;
      const vfloat4 tFarY  = (load4f((const char*)&node->lower_x+ray.farY ) - ray.org.y) * ray.rdir.y;
      const vfloat4 tFarZ  = (load4f((const char*)&node->lower_x+ray.farZ ) - ray.org.z) * ray.rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat4 tNear = max(tNearX,tNearY,tNearZ,tnear);
        const vfloat4 tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const vbool4 vmask = (round_down*tNear <= round_up*tFar);
        const size_t mask = movemask(vmask);
        dist = tNear;
        return mask;
      }
      
#if defined(__SSE4_1__)
      const vfloat4 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
      const vfloat4 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
      const vbool4 vmask = cast(tNear) > cast(tFar);
      const size_t mask = movemask(vmask)^0xf;
#else
      const vfloat4 tNear = max(tNearX,tNearY,tNearZ,tnear);
      const vfloat4 tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
      const vbool4 vmask = tNear <= tFar;
      const size_t mask = movemask(vmask);
#endif
      dist = tNear;
      return mask;
    }
    
    /*! intersection with ray packet of size 4 */
    template<bool robust>
      __forceinline vbool4 intersect_node(const BVH4::Node* node, size_t i, const Vec3vf4& org, const Vec3vf4& rdir, const Vec3vf4& org_rdir, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist)
    {
#if defined(__AVX2__)
      const vfloat4 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const vfloat4 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const vfloat4 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const vfloat4 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const vfloat4 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const vfloat4 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
#else
      const vfloat4 lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
      const vfloat4 lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
      const vfloat4 lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
      const vfloat4 lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
      const vfloat4 lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
      const vfloat4 lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat4 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const vfloat4 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const vbool4 lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
#if defined(__SSE4_1__)
      const vfloat4 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const vfloat4 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const vbool4 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      
      const vfloat4 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const vfloat4 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const vbool4 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
    
    /*! intersection with ray packet of size 8 */
#if defined(__AVX__)
    template<bool robust>
      __forceinline vbool8 intersect8_node(const BVH4::Node* node, size_t i, const Vec3vf8& org, const Vec3vf8& rdir, const Vec3vf8& org_rdir, const vfloat8& tnear, const vfloat8& tfar, vfloat8& dist) 
    {
#if defined(__AVX2__)
      const vfloat8 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const vfloat8 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const vfloat8 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const vfloat8 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const vfloat8 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const vfloat8 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
#else
      const vfloat8 lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
      const vfloat8 lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
      const vfloat8 lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
      const vfloat8 lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
      const vfloat8 lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
      const vfloat8 lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const vfloat8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const vbool8 lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
#if defined(__AVX2__)
      const vfloat8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const vfloat8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const vbool8 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      const vfloat8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const vfloat8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const vbool8 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
#endif


    /*! intersection with ray packet of size 8 */
#if defined(__AVX512F__)
    template<bool robust>
      __forceinline vbool16 intersect16_node(const BVH4::Node* node, size_t i, const Vec3vf16& org, const Vec3vf16& rdir, const Vec3vf16& org_rdir, const vfloat16& tnear, const vfloat16& tfar, vfloat16& dist) 
    {
      const vfloat16 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const vfloat16 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const vfloat16 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const vfloat16 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const vfloat16 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const vfloat16 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
      
      if (robust) { // FIXME: use per instruction rounding
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const vfloat16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const vbool16 lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
      const vfloat16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const vfloat16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const vbool16 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
      dist = lnearP;
      return lhit;
    }
#endif


    
    /*! intersection with single rays */
    __forceinline size_t intersect_node(const BVH4::NodeMB* node, size_t nearX, size_t nearY, size_t nearZ,
                                        const Vec3vf4& org, const Vec3vf4& rdir, const Vec3vf4& org_rdir, const vfloat4& tnear, const vfloat4& tfar, const float time,
                                        vfloat4& dist) 
    {
      const size_t farX  = nearX ^ sizeof(vfloat4), farY  = nearY ^ sizeof(vfloat4), farZ  = nearZ ^ sizeof(vfloat4);
      const vfloat4* pNearX = (const vfloat4*)((const char*)&node->lower_x+nearX);
      const vfloat4* pNearY = (const vfloat4*)((const char*)&node->lower_x+nearY);
      const vfloat4* pNearZ = (const vfloat4*)((const char*)&node->lower_x+nearZ);
      const vfloat4 tNearX = (vfloat4(pNearX[0]) + time*pNearX[6] - org.x) * rdir.x;
      const vfloat4 tNearY = (vfloat4(pNearY[0]) + time*pNearY[6] - org.y) * rdir.y;
      const vfloat4 tNearZ = (vfloat4(pNearZ[0]) + time*pNearZ[6] - org.z) * rdir.z;
      const vfloat4 tNear = max(tnear,tNearX,tNearY,tNearZ);
      const vfloat4* pFarX = (const vfloat4*)((const char*)&node->lower_x+farX);
      const vfloat4* pFarY = (const vfloat4*)((const char*)&node->lower_x+farY);
      const vfloat4* pFarZ = (const vfloat4*)((const char*)&node->lower_x+farZ);
      const vfloat4 tFarX = (vfloat4(pFarX[0]) + time*pFarX[6] - org.x) * rdir.x;
      const vfloat4 tFarY = (vfloat4(pFarY[0]) + time*pFarY[6] - org.y) * rdir.y;
      const vfloat4 tFarZ = (vfloat4(pFarZ[0]) + time*pFarZ[6] - org.z) * rdir.z;
      const vfloat4 tFar = min(tfar,tFarX,tFarY,tFarZ);
      const size_t mask = movemask(tNear <= tFar);
      dist = tNear;
      return mask;
    }
    
    /*! intersection with ray packet of size 4 */
    __forceinline vbool4 intersect_node(const BVH4::NodeMB* node, const size_t i, const Vec3vf4& org, const Vec3vf4& rdir, const Vec3vf4& org_rdir, const vfloat4& tnear, const vfloat4& tfar, 
                                      const vfloat4& time, vfloat4& dist) 
    {
      const vfloat4 vlower_x = vfloat4(node->lower_x[i]) + time * vfloat4(node->lower_dx[i]);
      const vfloat4 vlower_y = vfloat4(node->lower_y[i]) + time * vfloat4(node->lower_dy[i]);
      const vfloat4 vlower_z = vfloat4(node->lower_z[i]) + time * vfloat4(node->lower_dz[i]);
      const vfloat4 vupper_x = vfloat4(node->upper_x[i]) + time * vfloat4(node->upper_dx[i]);
      const vfloat4 vupper_y = vfloat4(node->upper_y[i]) + time * vfloat4(node->upper_dy[i]);
      const vfloat4 vupper_z = vfloat4(node->upper_z[i]) + time * vfloat4(node->upper_dz[i]);
      
#if defined(__AVX2__)
      const vfloat4 lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const vfloat4 lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const vfloat4 lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const vfloat4 lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const vfloat4 lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const vfloat4 lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
#else
      const vfloat4 lclipMinX = (vlower_x - org.x) * rdir.x;
      const vfloat4 lclipMinY = (vlower_y - org.y) * rdir.y;
      const vfloat4 lclipMinZ = (vlower_z - org.z) * rdir.z;
      const vfloat4 lclipMaxX = (vupper_x - org.x) * rdir.x;
      const vfloat4 lclipMaxY = (vupper_y - org.y) * rdir.y;
      const vfloat4 lclipMaxZ = (vupper_z - org.z) * rdir.z;
#endif
      
#if defined(__SSE4_1__)
      const vfloat4 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const vfloat4 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const vbool4 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      
      const vfloat4 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const vfloat4 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const vbool4 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
    
    /*! intersection with ray packet of size 8 */
#if defined(__AVX__)
    __forceinline vbool8 intersect_node(const BVH4::NodeMB* node, const size_t i, const Vec3vf8& org, const Vec3vf8& rdir, const Vec3vf8& org_rdir, const vfloat8& tnear, const vfloat8& tfar, const vfloat8& time, vfloat8& dist) 
    {
      const vfloat8 vlower_x = vfloat8(node->lower_x[i]) + time * vfloat8(node->lower_dx[i]);
      const vfloat8 vlower_y = vfloat8(node->lower_y[i]) + time * vfloat8(node->lower_dy[i]);
      const vfloat8 vlower_z = vfloat8(node->lower_z[i]) + time * vfloat8(node->lower_dz[i]);
      const vfloat8 vupper_x = vfloat8(node->upper_x[i]) + time * vfloat8(node->upper_dx[i]);
      const vfloat8 vupper_y = vfloat8(node->upper_y[i]) + time * vfloat8(node->upper_dy[i]);
      const vfloat8 vupper_z = vfloat8(node->upper_z[i]) + time * vfloat8(node->upper_dz[i]);
      
#if defined(__AVX2__)
      const vfloat8 lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const vfloat8 lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const vfloat8 lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const vfloat8 lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const vfloat8 lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const vfloat8 lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
      const vfloat8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const vfloat8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const vbool8 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      const vfloat8 lclipMinX = (vlower_x - org.x) * rdir.x;
      const vfloat8 lclipMinY = (vlower_y - org.y) * rdir.y;
      const vfloat8 lclipMinZ = (vlower_z - org.z) * rdir.z;
      const vfloat8 lclipMaxX = (vupper_x - org.x) * rdir.x;
      const vfloat8 lclipMaxY = (vupper_y - org.y) * rdir.y;
      const vfloat8 lclipMaxZ = (vupper_z - org.z) * rdir.z;
      const vfloat8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const vfloat8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const vbool8 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
#endif


    /*! intersection with ray packet of size 8 */
#if defined(__AVX512F__)
    __forceinline vbool16 intersect_node(const BVH4::NodeMB* node, const size_t i, const Vec3vf16& org, const Vec3vf16& rdir, const Vec3vf16& org_rdir, const vfloat16& tnear, const vfloat16& tfar, const vfloat16& time, vfloat16& dist) 
    {
      const vfloat16 vlower_x = vfloat16(node->lower_x[i]) + time * vfloat16(node->lower_dx[i]);
      const vfloat16 vlower_y = vfloat16(node->lower_y[i]) + time * vfloat16(node->lower_dy[i]);
      const vfloat16 vlower_z = vfloat16(node->lower_z[i]) + time * vfloat16(node->lower_dz[i]);
      const vfloat16 vupper_x = vfloat16(node->upper_x[i]) + time * vfloat16(node->upper_dx[i]);
      const vfloat16 vupper_y = vfloat16(node->upper_y[i]) + time * vfloat16(node->upper_dy[i]);
      const vfloat16 vupper_z = vfloat16(node->upper_z[i]) + time * vfloat16(node->upper_dz[i]);
      
      const vfloat16 lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const vfloat16 lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const vfloat16 lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const vfloat16 lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const vfloat16 lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const vfloat16 lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
      const vfloat16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const vfloat16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const vbool16 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
      dist = lnearP;
      return lhit;
    }
#endif


    /*! intersect 4 OBBs with single ray */
    __forceinline size_t intersect_node(const BVH4::UnalignedNode* node, const Vec3vf4& ray_org, const Vec3vf4& ray_dir, 
                                        const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist)
    {
      const Vec3vf4 dir = xfmVector(node->naabb,ray_dir);
      //const Vec3vf4 nrdir = Vec3vf4(vfloat4(-1.0f))/dir;
      const Vec3vf4 nrdir = Vec3vf4(vfloat4(-1.0f))*rcp_safe(dir);
      const Vec3vf4 org = xfmPoint(node->naabb,ray_org);
      const Vec3vf4 tLowerXYZ = org * nrdir;     // (Vec3fa(zero) - org) * rdir;
      const Vec3vf4 tUpperXYZ = tLowerXYZ - nrdir; // (Vec3fa(one ) - org) * rdir;
      
#if defined(__SSE4_1__)
      const vfloat4 tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const vfloat4 tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const vfloat4 tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const vfloat4 tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const vfloat4 tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const vfloat4 tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      const vfloat4 tNear  = max(tnear, tNearX,tNearY,tNearZ);
      const vfloat4 tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
      const vbool4 vmask = tNear <= tFar;
      dist = tNear;
      return movemask(vmask);
#else
      const vfloat4 tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
      const vfloat4 tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
      const vfloat4 tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
      const vfloat4 tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
      const vfloat4 tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
      const vfloat4 tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
      const vfloat4 tNear = max(tnear, tNearX,tNearY,tNearZ);
      const vfloat4 tFar  = min(tfar,  tFarX ,tFarY ,tFarZ );
      const vbool4 vmask = tNear <= tFar;
      dist = tNear;
      return movemask(vmask);
#endif
    }

    
    /*! intersect 4 OBBs with single ray */
    __forceinline size_t intersect_node(const BVH4::UnalignedNodeMB* node,
                                        const Vec3vf4& ray_org, const Vec3vf4& ray_dir, 
                                        const vfloat4& tnear, const vfloat4& tfar, const float time, vfloat4& dist)
      {
	const vfloat4 t0 = vfloat4(1.0f)-time, t1 = time;

	const AffineSpace3vf4 xfm = node->space0;
	const Vec3vf4 b0_lower = zero;
	const Vec3vf4 b0_upper = one;
	const Vec3vf4 lower = t0*b0_lower + t1*node->b1.lower;
	const Vec3vf4 upper = t0*b0_upper + t1*node->b1.upper;
	
	const BBox3vf4 bounds(lower,upper);
	const Vec3vf4 dir = xfmVector(xfm,ray_dir);
	const Vec3vf4 rdir = rcp_safe(dir); 
	const Vec3vf4 org = xfmPoint(xfm,ray_org);
	
	const Vec3vf4 tLowerXYZ = (bounds.lower - org) * rdir;
	const Vec3vf4 tUpperXYZ = (bounds.upper - org) * rdir;
	
#if defined(__SSE4_1__)
	const vfloat4 tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
	const vfloat4 tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
	const vfloat4 tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
	const vfloat4 tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
	const vfloat4 tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
	const vfloat4 tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
	const vfloat4 tNear  = max(tnear, tNearX,tNearY,tNearZ);
	const vfloat4 tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
	const vbool4 vmask = tNear <= tFar;
	dist = tNear;
	return movemask(vmask);
#else
	const vfloat4 tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
	const vfloat4 tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
	const vfloat4 tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
	const vfloat4 tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
	const vfloat4 tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
	const vfloat4 tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
	const vfloat4 tNear = max(tnear, tNearX,tNearY,tNearZ);
	const vfloat4 tFar  = min(tfar,  tFarX ,tFarY ,tFarZ );
	const vbool4 vmask = tNear <= tFar;
	dist = tNear;
	return movemask(vmask);
#endif
      }
    
  }
}

