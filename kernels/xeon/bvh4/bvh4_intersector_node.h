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
                                          const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar, 
                                          ssef& dist) 
    {
      const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
#if defined (__AVX2__)
      const ssef tNearX = msub(load4f((const char*)&node->lower_x+nearX), rdir.x, org_rdir.x);
      const ssef tNearY = msub(load4f((const char*)&node->lower_x+nearY), rdir.y, org_rdir.y);
      const ssef tNearZ = msub(load4f((const char*)&node->lower_x+nearZ), rdir.z, org_rdir.z);
      const ssef tFarX  = msub(load4f((const char*)&node->lower_x+farX ), rdir.x, org_rdir.x);
      const ssef tFarY  = msub(load4f((const char*)&node->lower_x+farY ), rdir.y, org_rdir.y);
      const ssef tFarZ  = msub(load4f((const char*)&node->lower_x+farZ ), rdir.z, org_rdir.z);
#else
      const ssef tNearX = (load4f((const char*)&node->lower_x+nearX) - org.x) * rdir.x;
      const ssef tNearY = (load4f((const char*)&node->lower_x+nearY) - org.y) * rdir.y;
      const ssef tNearZ = (load4f((const char*)&node->lower_x+nearZ) - org.z) * rdir.z;
      const ssef tFarX  = (load4f((const char*)&node->lower_x+farX ) - org.x) * rdir.x;
      const ssef tFarY  = (load4f((const char*)&node->lower_x+farY ) - org.y) * rdir.y;
      const ssef tFarZ  = (load4f((const char*)&node->lower_x+farZ ) - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const ssef tNear = max(tNearX,tNearY,tNearZ,tnear);
        const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const sseb vmask = (round_down*tNear <= round_up*tFar);
        const size_t mask = movemask(vmask);
        dist = tNear;
        return mask;
      }
      
#if defined(__SSE4_1__)
      const ssef tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
      const ssef tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
      const sseb vmask = cast(tNear) > cast(tFar);
      const size_t mask = movemask(vmask)^0xf;
#else
      const ssef tNear = max(tNearX,tNearY,tNearZ,tnear);
      const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
      const sseb vmask = tNear <= tFar;
      const size_t mask = movemask(vmask);
#endif
      dist = tNear;
      return mask;
    }
    
    /*! intersection with ray packet of size 4 */
    template<bool robust>
      __forceinline sseb intersect_node(const BVH4::Node* node, size_t i, const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar, ssef& dist)
    {
#if defined(__AVX2__)
      const ssef lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const ssef lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const ssef lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const ssef lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const ssef lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const ssef lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
#else
      const ssef lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
      const ssef lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
      const ssef lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
      const ssef lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
      const ssef lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
      const ssef lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const ssef lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const ssef lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const sseb lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
#if defined(__SSE4_1__)
      const ssef lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const ssef lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const sseb lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      
      const ssef lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const ssef lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const sseb lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
    
    /*! intersection with ray packet of size 8 */
#if defined(__AVX__)
    template<bool robust>
      __forceinline avxb intersect8_node(const BVH4::Node* node, size_t i, const avx3f& org, const avx3f& rdir, const avx3f& org_rdir, const avxf& tnear, const avxf& tfar, avxf& dist) 
    {
#if defined(__AVX2__)
      const avxf lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const avxf lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const avxf lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const avxf lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const avxf lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const avxf lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
#else
      const avxf lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
      const avxf lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
      const avxf lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
      const avxf lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
      const avxf lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
      const avxf lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const avxb lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
#if defined(__AVX2__)
      const avxf lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const avxf lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const avxb lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const avxb lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
#endif
    
    /*! intersection with single rays */
    __forceinline size_t intersect_node(const BVH4::NodeMB* node, size_t nearX, size_t nearY, size_t nearZ,
                                        const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar, const float time,
                                        ssef& dist) 
    {
      const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
      const ssef* pNearX = (const ssef*)((const char*)&node->lower_x+nearX);
      const ssef* pNearY = (const ssef*)((const char*)&node->lower_x+nearY);
      const ssef* pNearZ = (const ssef*)((const char*)&node->lower_x+nearZ);
      const ssef tNearX = (ssef(pNearX[0]) + time*pNearX[6] - org.x) * rdir.x;
      const ssef tNearY = (ssef(pNearY[0]) + time*pNearY[6] - org.y) * rdir.y;
      const ssef tNearZ = (ssef(pNearZ[0]) + time*pNearZ[6] - org.z) * rdir.z;
      const ssef tNear = max(tnear,tNearX,tNearY,tNearZ);
      const ssef* pFarX = (const ssef*)((const char*)&node->lower_x+farX);
      const ssef* pFarY = (const ssef*)((const char*)&node->lower_x+farY);
      const ssef* pFarZ = (const ssef*)((const char*)&node->lower_x+farZ);
      const ssef tFarX = (ssef(pFarX[0]) + time*pFarX[6] - org.x) * rdir.x;
      const ssef tFarY = (ssef(pFarY[0]) + time*pFarY[6] - org.y) * rdir.y;
      const ssef tFarZ = (ssef(pFarZ[0]) + time*pFarZ[6] - org.z) * rdir.z;
      const ssef tFar = min(tfar,tFarX,tFarY,tFarZ);
      const size_t mask = movemask(tNear <= tFar);
      dist = tNear;
      return mask;
    }
    
    /*! intersection with ray packet of size 4 */
    __forceinline sseb intersect_node(const BVH4::NodeMB* node, const size_t i, const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar, 
                                      const ssef& time, ssef& dist) 
    {
      const ssef vlower_x = ssef(node->lower_x[i]) + time * ssef(node->lower_dx[i]);
      const ssef vlower_y = ssef(node->lower_y[i]) + time * ssef(node->lower_dy[i]);
      const ssef vlower_z = ssef(node->lower_z[i]) + time * ssef(node->lower_dz[i]);
      const ssef vupper_x = ssef(node->upper_x[i]) + time * ssef(node->upper_dx[i]);
      const ssef vupper_y = ssef(node->upper_y[i]) + time * ssef(node->upper_dy[i]);
      const ssef vupper_z = ssef(node->upper_z[i]) + time * ssef(node->upper_dz[i]);
      
#if defined(__AVX2__)
      const ssef lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const ssef lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const ssef lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const ssef lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const ssef lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const ssef lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
#else
      const ssef lclipMinX = (vlower_x - org.x) * rdir.x;
      const ssef lclipMinY = (vlower_y - org.y) * rdir.y;
      const ssef lclipMinZ = (vlower_z - org.z) * rdir.z;
      const ssef lclipMaxX = (vupper_x - org.x) * rdir.x;
      const ssef lclipMaxY = (vupper_y - org.y) * rdir.y;
      const ssef lclipMaxZ = (vupper_z - org.z) * rdir.z;
#endif
      
#if defined(__SSE4_1__)
      const ssef lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const ssef lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const sseb lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      
      const ssef lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const ssef lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const sseb lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
    
    /*! intersection with ray packet of size 8 */
#if defined(__AVX__)
    __forceinline avxb intersect_node(const BVH4::NodeMB* node, const size_t i, const avx3f& org, const avx3f& rdir, const avx3f& org_rdir, const avxf& tnear, const avxf& tfar, const avxf& time, avxf& dist) 
    {
      const avxf vlower_x = avxf(node->lower_x[i]) + time * avxf(node->lower_dx[i]);
      const avxf vlower_y = avxf(node->lower_y[i]) + time * avxf(node->lower_dy[i]);
      const avxf vlower_z = avxf(node->lower_z[i]) + time * avxf(node->lower_dz[i]);
      const avxf vupper_x = avxf(node->upper_x[i]) + time * avxf(node->upper_dx[i]);
      const avxf vupper_y = avxf(node->upper_y[i]) + time * avxf(node->upper_dy[i]);
      const avxf vupper_z = avxf(node->upper_z[i]) + time * avxf(node->upper_dz[i]);
      
#if defined(__AVX2__)
      const avxf lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const avxf lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const avxf lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const avxf lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const avxf lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const avxf lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
      const avxf lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const avxf lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const avxb lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
#else
      const avxf lclipMinX = (vlower_x - org.x) * rdir.x;
      const avxf lclipMinY = (vlower_y - org.y) * rdir.y;
      const avxf lclipMinZ = (vlower_z - org.z) * rdir.z;
      const avxf lclipMaxX = (vupper_x - org.x) * rdir.x;
      const avxf lclipMaxY = (vupper_y - org.y) * rdir.y;
      const avxf lclipMaxZ = (vupper_z - org.z) * rdir.z;
      const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
      const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
      const avxb lhit   = max(lnearP,tnear) <= min(lfarP,tfar);      
#endif
      dist = lnearP;
      return lhit;
    }
#endif

    /*! intersect 4 OBBs with single ray */
    __forceinline size_t intersect_node(const BVH4::UnalignedNode* node, const sse3f& ray_org, const sse3f& ray_dir, 
                                        const ssef& tnear, const ssef& tfar, ssef& dist)
    {
      const sse3f dir = xfmVector(node->naabb,ray_dir);
      //const sse3f nrdir = sse3f(ssef(-1.0f))/dir;
      const sse3f nrdir = sse3f(ssef(-1.0f))*rcp_safe(dir);
      const sse3f org = xfmPoint(node->naabb,ray_org);
      const sse3f tLowerXYZ = org * nrdir;     // (Vec3fa(zero) - org) * rdir;
      const sse3f tUpperXYZ = tLowerXYZ - nrdir; // (Vec3fa(one ) - org) * rdir;
      
#if defined(__SSE4_1__)
      const ssef tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tNear  = max(tnear, tNearX,tNearY,tNearZ);
      const ssef tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
      const sseb vmask = tNear <= tFar;
      dist = tNear;
      return movemask(vmask);
#else
      const ssef tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tNear = max(tnear, tNearX,tNearY,tNearZ);
      const ssef tFar  = min(tfar,  tFarX ,tFarY ,tFarZ );
      const sseb vmask = tNear <= tFar;
      dist = tNear;
      return movemask(vmask);
#endif
    }

    
    /*! intersect 4 OBBs with single ray */
    __forceinline size_t intersect_node(const BVH4::UnalignedNodeMB* node,
                                        const sse3f& ray_org, const sse3f& ray_dir, 
                                        const ssef& tnear, const ssef& tfar, const float time, ssef& dist)
      {
	const ssef t0 = ssef(1.0f)-time, t1 = time;

	const AffineSpaceSSE3f xfm = node->space0;
	const sse3f b0_lower = zero;
	const sse3f b0_upper = one;
	const sse3f lower = t0*b0_lower + t1*node->b1.lower;
	const sse3f upper = t0*b0_upper + t1*node->b1.upper;
	
	const BBoxSSE3f bounds(lower,upper);
	const sse3f dir = xfmVector(xfm,ray_dir);
	const sse3f rdir = rcp_safe(dir); 
	const sse3f org = xfmPoint(xfm,ray_org);
	
	const sse3f tLowerXYZ = (bounds.lower - org) * rdir;
	const sse3f tUpperXYZ = (bounds.upper - org) * rdir;
	
#if defined(__SSE4_1__)
	const ssef tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
	const ssef tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
	const ssef tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
	const ssef tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
	const ssef tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
	const ssef tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
	const ssef tNear  = max(tnear, tNearX,tNearY,tNearZ);
	const ssef tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
	const sseb vmask = tNear <= tFar;
	dist = tNear;
	return movemask(vmask);
#else
	const ssef tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
	const ssef tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
	const ssef tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
	const ssef tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
	const ssef tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
	const ssef tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
	const ssef tNear = max(tnear, tNearX,tNearY,tNearZ);
	const ssef tFar  = min(tfar,  tFarX ,tFarY ,tFarZ );
	const sseb vmask = tNear <= tFar;
	dist = tNear;
	return movemask(vmask);
#endif
      }
    
  }
}

