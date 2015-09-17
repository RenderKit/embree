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
    /*! intersection with single rays */
    template<bool robust>
      __forceinline size_t intersect_node(const BVH4::Node* node, size_t nearX, size_t nearY, size_t nearZ,
                                          const Vec3f4& org, const Vec3f4& rdir, const Vec3f4& org_rdir, const float4& tnear, const float4& tfar, 
                                          float4& dist) 
    {
      const size_t farX  = nearX ^ sizeof(float4), farY  = nearY ^ sizeof(float4), farZ  = nearZ ^ sizeof(float4);
#if defined (__AVX2__)
      const float4 tNearX = msub(load4f((const char*)&node->lower_x+nearX), rdir.x, org_rdir.x);
      const float4 tNearY = msub(load4f((const char*)&node->lower_x+nearY), rdir.y, org_rdir.y);
      const float4 tNearZ = msub(load4f((const char*)&node->lower_x+nearZ), rdir.z, org_rdir.z);
      const float4 tFarX  = msub(load4f((const char*)&node->lower_x+farX ), rdir.x, org_rdir.x);
      const float4 tFarY  = msub(load4f((const char*)&node->lower_x+farY ), rdir.y, org_rdir.y);
      const float4 tFarZ  = msub(load4f((const char*)&node->lower_x+farZ ), rdir.z, org_rdir.z);
#else
      const float4 tNearX = (load4f((const char*)&node->lower_x+nearX) - org.x) * rdir.x;
      const float4 tNearY = (load4f((const char*)&node->lower_x+nearY) - org.y) * rdir.y;
      const float4 tNearZ = (load4f((const char*)&node->lower_x+nearZ) - org.z) * rdir.z;
      const float4 tFarX  = (load4f((const char*)&node->lower_x+farX ) - org.x) * rdir.x;
      const float4 tFarY  = (load4f((const char*)&node->lower_x+farY ) - org.y) * rdir.y;
      const float4 tFarZ  = (load4f((const char*)&node->lower_x+farZ ) - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const float4 tNear = max(tNearX,tNearY,tNearZ,tnear);
        const float4 tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const bool4 vmask = (round_down*tNear <= round_up*tFar);
        const size_t mask = movemask(vmask);
        dist = tNear;
        return mask;
      }
      
#if defined(__SSE4_1__)
      const float4 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
      const float4 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
      const bool4 vmask = cast(tNear) > cast(tFar);
      const size_t mask = movemask(vmask)^0xf;
#else
      const float4 tNear = max(tNearX,tNearY,tNearZ,tnear);
      const float4 tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
      const bool4 vmask = tNear <= tFar;
      const size_t mask = movemask(vmask);
#endif
      dist = tNear;
      return mask;
    }
    
    /*! intersection with ray packet of size 4 */
    template<bool robust>
      __forceinline bool4 intersect_node(const BVH4::Node* node, size_t i, const Vec3f4& org, const Vec3f4& rdir, const Vec3f4& org_rdir, const float4& tnear, const float4& tfar, float4& dist)
    {
#if defined(__AVX2__)
      const float4 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const float4 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const float4 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const float4 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const float4 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const float4 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
#else
      const float4 lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
      const float4 lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
      const float4 lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
      const float4 lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
      const float4 lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
      const float4 lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const float4 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const float4 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const bool4 lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
#if defined(__SSE4_1__)
      const float4 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const float4 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const bool4 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      
      const float4 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const float4 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const bool4 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
    
    /*! intersection with ray packet of size 8 */
#if defined(__AVX__)
    template<bool robust>
      __forceinline bool8 intersect8_node(const BVH4::Node* node, size_t i, const Vec3f8& org, const Vec3f8& rdir, const Vec3f8& org_rdir, const float8& tnear, const float8& tfar, float8& dist) 
    {
#if defined(__AVX2__)
      const float8 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const float8 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const float8 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const float8 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const float8 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const float8 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
#else
      const float8 lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
      const float8 lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
      const float8 lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
      const float8 lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
      const float8 lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
      const float8 lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const float8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const float8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const bool8 lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
#if defined(__AVX2__)
      const float8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const float8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const bool8 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      const float8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const float8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const bool8 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
#endif


    /*! intersection with ray packet of size 8 */
#if defined(__AVX512F__)
    template<bool robust>
      __forceinline bool16 intersect16_node(const BVH4::Node* node, size_t i, const Vec3f16& org, const Vec3f16& rdir, const Vec3f16& org_rdir, const float16& tnear, const float16& tfar, float16& dist) 
    {
      const float16 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const float16 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const float16 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const float16 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const float16 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const float16 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
      
      if (robust) { // FIXME: use per instruction rounding
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const float16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const float16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const bool16 lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
      const float16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const float16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const bool16 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
      dist = lnearP;
      return lhit;
    }
#endif


    
    /*! intersection with single rays */
    __forceinline size_t intersect_node(const BVH4::NodeMB* node, size_t nearX, size_t nearY, size_t nearZ,
                                        const Vec3f4& org, const Vec3f4& rdir, const Vec3f4& org_rdir, const float4& tnear, const float4& tfar, const float time,
                                        float4& dist) 
    {
      const size_t farX  = nearX ^ sizeof(float4), farY  = nearY ^ sizeof(float4), farZ  = nearZ ^ sizeof(float4);
      const float4* pNearX = (const float4*)((const char*)&node->lower_x+nearX);
      const float4* pNearY = (const float4*)((const char*)&node->lower_x+nearY);
      const float4* pNearZ = (const float4*)((const char*)&node->lower_x+nearZ);
      const float4 tNearX = (float4(pNearX[0]) + time*pNearX[6] - org.x) * rdir.x;
      const float4 tNearY = (float4(pNearY[0]) + time*pNearY[6] - org.y) * rdir.y;
      const float4 tNearZ = (float4(pNearZ[0]) + time*pNearZ[6] - org.z) * rdir.z;
      const float4 tNear = max(tnear,tNearX,tNearY,tNearZ);
      const float4* pFarX = (const float4*)((const char*)&node->lower_x+farX);
      const float4* pFarY = (const float4*)((const char*)&node->lower_x+farY);
      const float4* pFarZ = (const float4*)((const char*)&node->lower_x+farZ);
      const float4 tFarX = (float4(pFarX[0]) + time*pFarX[6] - org.x) * rdir.x;
      const float4 tFarY = (float4(pFarY[0]) + time*pFarY[6] - org.y) * rdir.y;
      const float4 tFarZ = (float4(pFarZ[0]) + time*pFarZ[6] - org.z) * rdir.z;
      const float4 tFar = min(tfar,tFarX,tFarY,tFarZ);
      const size_t mask = movemask(tNear <= tFar);
      dist = tNear;
      return mask;
    }
    
    /*! intersection with ray packet of size 4 */
    __forceinline bool4 intersect_node(const BVH4::NodeMB* node, const size_t i, const Vec3f4& org, const Vec3f4& rdir, const Vec3f4& org_rdir, const float4& tnear, const float4& tfar, 
                                      const float4& time, float4& dist) 
    {
      const float4 vlower_x = float4(node->lower_x[i]) + time * float4(node->lower_dx[i]);
      const float4 vlower_y = float4(node->lower_y[i]) + time * float4(node->lower_dy[i]);
      const float4 vlower_z = float4(node->lower_z[i]) + time * float4(node->lower_dz[i]);
      const float4 vupper_x = float4(node->upper_x[i]) + time * float4(node->upper_dx[i]);
      const float4 vupper_y = float4(node->upper_y[i]) + time * float4(node->upper_dy[i]);
      const float4 vupper_z = float4(node->upper_z[i]) + time * float4(node->upper_dz[i]);
      
#if defined(__AVX2__)
      const float4 lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const float4 lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const float4 lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const float4 lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const float4 lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const float4 lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
#else
      const float4 lclipMinX = (vlower_x - org.x) * rdir.x;
      const float4 lclipMinY = (vlower_y - org.y) * rdir.y;
      const float4 lclipMinZ = (vlower_z - org.z) * rdir.z;
      const float4 lclipMaxX = (vupper_x - org.x) * rdir.x;
      const float4 lclipMaxY = (vupper_y - org.y) * rdir.y;
      const float4 lclipMaxZ = (vupper_z - org.z) * rdir.z;
#endif
      
#if defined(__SSE4_1__)
      const float4 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const float4 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const bool4 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      
      const float4 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const float4 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const bool4 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
    
    /*! intersection with ray packet of size 8 */
#if defined(__AVX__)
    __forceinline bool8 intersect_node(const BVH4::NodeMB* node, const size_t i, const Vec3f8& org, const Vec3f8& rdir, const Vec3f8& org_rdir, const float8& tnear, const float8& tfar, const float8& time, float8& dist) 
    {
      const float8 vlower_x = float8(node->lower_x[i]) + time * float8(node->lower_dx[i]);
      const float8 vlower_y = float8(node->lower_y[i]) + time * float8(node->lower_dy[i]);
      const float8 vlower_z = float8(node->lower_z[i]) + time * float8(node->lower_dz[i]);
      const float8 vupper_x = float8(node->upper_x[i]) + time * float8(node->upper_dx[i]);
      const float8 vupper_y = float8(node->upper_y[i]) + time * float8(node->upper_dy[i]);
      const float8 vupper_z = float8(node->upper_z[i]) + time * float8(node->upper_dz[i]);
      
#if defined(__AVX2__)
      const float8 lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const float8 lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const float8 lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const float8 lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const float8 lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const float8 lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
      const float8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const float8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const bool8 lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      const float8 lclipMinX = (vlower_x - org.x) * rdir.x;
      const float8 lclipMinY = (vlower_y - org.y) * rdir.y;
      const float8 lclipMinZ = (vlower_z - org.z) * rdir.z;
      const float8 lclipMaxX = (vupper_x - org.x) * rdir.x;
      const float8 lclipMaxY = (vupper_y - org.y) * rdir.y;
      const float8 lclipMaxZ = (vupper_z - org.z) * rdir.z;
      const float8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const float8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const bool8 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
#endif


    /*! intersection with ray packet of size 8 */
#if defined(__AVX512F__)
    __forceinline bool16 intersect_node(const BVH4::NodeMB* node, const size_t i, const Vec3f16& org, const Vec3f16& rdir, const Vec3f16& org_rdir, const float16& tnear, const float16& tfar, const float16& time, float16& dist) 
    {
      const float16 vlower_x = float16(node->lower_x[i]) + time * float16(node->lower_dx[i]);
      const float16 vlower_y = float16(node->lower_y[i]) + time * float16(node->lower_dy[i]);
      const float16 vlower_z = float16(node->lower_z[i]) + time * float16(node->lower_dz[i]);
      const float16 vupper_x = float16(node->upper_x[i]) + time * float16(node->upper_dx[i]);
      const float16 vupper_y = float16(node->upper_y[i]) + time * float16(node->upper_dy[i]);
      const float16 vupper_z = float16(node->upper_z[i]) + time * float16(node->upper_dz[i]);
      
      const float16 lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const float16 lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const float16 lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const float16 lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const float16 lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const float16 lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
      const float16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const float16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const bool16 lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
      dist = lnearP;
      return lhit;
    }
#endif


    /*! intersect 4 OBBs with single ray */
    __forceinline size_t intersect_node(const BVH4::UnalignedNode* node, const Vec3f4& ray_org, const Vec3f4& ray_dir, 
                                        const float4& tnear, const float4& tfar, float4& dist)
    {
      const Vec3f4 dir = xfmVector(node->naabb,ray_dir);
      //const Vec3f4 nrdir = Vec3f4(float4(-1.0f))/dir;
      const Vec3f4 nrdir = Vec3f4(float4(-1.0f))*rcp_safe(dir);
      const Vec3f4 org = xfmPoint(node->naabb,ray_org);
      const Vec3f4 tLowerXYZ = org * nrdir;     // (Vec3fa(zero) - org) * rdir;
      const Vec3f4 tUpperXYZ = tLowerXYZ - nrdir; // (Vec3fa(one ) - org) * rdir;
      
#if defined(__SSE4_1__)
      const float4 tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const float4 tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const float4 tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const float4 tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const float4 tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const float4 tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      const float4 tNear  = max(tnear, tNearX,tNearY,tNearZ);
      const float4 tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
      const bool4 vmask = tNear <= tFar;
      dist = tNear;
      return movemask(vmask);
#else
      const float4 tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
      const float4 tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
      const float4 tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
      const float4 tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
      const float4 tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
      const float4 tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
      const float4 tNear = max(tnear, tNearX,tNearY,tNearZ);
      const float4 tFar  = min(tfar,  tFarX ,tFarY ,tFarZ );
      const bool4 vmask = tNear <= tFar;
      dist = tNear;
      return movemask(vmask);
#endif
    }

    
    /*! intersect 4 OBBs with single ray */
    __forceinline size_t intersect_node(const BVH4::UnalignedNodeMB* node,
                                        const Vec3f4& ray_org, const Vec3f4& ray_dir, 
                                        const float4& tnear, const float4& tfar, const float time, float4& dist)
      {
	const float4 t0 = float4(1.0f)-time, t1 = time;

	const AffineSpaceSSE3f xfm = node->space0;
	const Vec3f4 b0_lower = zero;
	const Vec3f4 b0_upper = one;
	const Vec3f4 lower = t0*b0_lower + t1*node->b1.lower;
	const Vec3f4 upper = t0*b0_upper + t1*node->b1.upper;
	
	const BBoxSSE3f bounds(lower,upper);
	const Vec3f4 dir = xfmVector(xfm,ray_dir);
	const Vec3f4 rdir = rcp_safe(dir); 
	const Vec3f4 org = xfmPoint(xfm,ray_org);
	
	const Vec3f4 tLowerXYZ = (bounds.lower - org) * rdir;
	const Vec3f4 tUpperXYZ = (bounds.upper - org) * rdir;
	
#if defined(__SSE4_1__)
	const float4 tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
	const float4 tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
	const float4 tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
	const float4 tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
	const float4 tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
	const float4 tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
	const float4 tNear  = max(tnear, tNearX,tNearY,tNearZ);
	const float4 tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
	const bool4 vmask = tNear <= tFar;
	dist = tNear;
	return movemask(vmask);
#else
	const float4 tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
	const float4 tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
	const float4 tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
	const float4 tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
	const float4 tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
	const float4 tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
	const float4 tNear = max(tnear, tNearX,tNearY,tNearZ);
	const float4 tFar  = min(tfar,  tFarX ,tFarY ,tFarZ );
	const bool4 vmask = tNear <= tFar;
	dist = tNear;
	return movemask(vmask);
#endif
      }
    
  }
}

