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

#include "../bvh/bvh.h"

namespace embree
{
  namespace isa
  {
    template<int N>
      struct TravRay 
    {
      __forceinline TravRay () {}

      __forceinline TravRay(const Vec3fa& ray_org, const Vec3fa& ray_dir) 
        : org_xyz(ray_org), dir_xyz(ray_dir) 
      {
        const Vec3fa ray_rdir = rcp_safe(ray_dir);
        const Vec3fa ray_org_rdir = ray_org*ray_rdir;
        org = Vec3vf4(ray_org.x,ray_org.y,ray_org.z);
        dir = Vec3vf4(ray_dir.x,ray_dir.y,ray_dir.z);
        rdir = Vec3vf4(ray_rdir.x,ray_rdir.y,ray_rdir.z);
        org_rdir = Vec3vf4(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
        nearX = ray_rdir.x >= 0.0f ? 0*sizeof(vfloat<N>) : 1*sizeof(vfloat<N>);
        nearY = ray_rdir.y >= 0.0f ? 2*sizeof(vfloat<N>) : 3*sizeof(vfloat<N>);
        nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(vfloat<N>) : 5*sizeof(vfloat<N>);
        farX  = nearX ^ sizeof(vfloat<N>);
        farY  = nearY ^ sizeof(vfloat<N>);
        farZ  = nearZ ^ sizeof(vfloat<N>);
      }
      Vec3fa org_xyz, dir_xyz; // FIXME: store somewhere else
      Vec3<vfloat<N>> org, dir, rdir, org_rdir; // FIXME: is org_rdir optimized away?
      size_t nearX, nearY, nearZ;
      size_t farX, farY, farZ;
    };

    /*! intersection with single rays */
    template<int N, bool robust>
      __forceinline size_t intersect_node(const BVH4::Node* node, size_t nearX, size_t nearY, size_t nearZ,
                                          const Vec3<vfloat<N>>& org, const Vec3<vfloat<N>>& rdir, const Vec3<vfloat<N>>& org_rdir, const vfloat<N>& tnear, const vfloat<N>& tfar, 
                                          vfloat<N>& dist) 
    {
      const size_t farX  = nearX ^ sizeof(vfloat<N>), farY  = nearY ^ sizeof(vfloat<N>), farZ  = nearZ ^ sizeof(vfloat<N>);
#if defined (__AVX2__)
      const vfloat<N> tNearX = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+nearX)), rdir.x, org_rdir.x);
      const vfloat<N> tNearY = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+nearY)), rdir.y, org_rdir.y);
      const vfloat<N> tNearZ = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+nearZ)), rdir.z, org_rdir.z);
      const vfloat<N> tFarX  = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+farX )), rdir.x, org_rdir.x);
      const vfloat<N> tFarY  = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+farY )), rdir.y, org_rdir.y);
      const vfloat<N> tFarZ  = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+farZ )), rdir.z, org_rdir.z);
#else
      const vfloat<N> tNearX = (vfloat<N>::load((float*)((const char*)&node->lower_x+nearX)) - org.x) * rdir.x;
      const vfloat<N> tNearY = (vfloat<N>::load((float*)((const char*)&node->lower_x+nearY)) - org.y) * rdir.y;
      const vfloat<N> tNearZ = (vfloat<N>::load((float*)((const char*)&node->lower_x+nearZ)) - org.z) * rdir.z;
      const vfloat<N> tFarX  = (vfloat<N>::load((float*)((const char*)&node->lower_x+farX )) - org.x) * rdir.x;
      const vfloat<N> tFarY  = (vfloat<N>::load((float*)((const char*)&node->lower_x+farY )) - org.y) * rdir.y;
      const vfloat<N> tFarZ  = (vfloat<N>::load((float*)((const char*)&node->lower_x+farZ )) - org.z) * rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat<N> tNear = max(tNearX,tNearY,tNearZ,tnear);
        const vfloat<N> tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const vbool4 vmask = (round_down*tNear <= round_up*tFar);
        const size_t mask = movemask(vmask);
        dist = tNear;
        return mask;
      }
      
#if defined(__SSE4_1__)
      const vfloat<N> tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
      const vfloat<N> tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
      const vbool4 vmask = cast(tNear) > cast(tFar);
      const size_t mask = movemask(vmask)^0xf;
#else
      const vfloat<N> tNear = max(tNearX,tNearY,tNearZ,tnear);
      const vfloat<N> tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
      const vbool4 vmask = tNear <= tFar;
      const size_t mask = movemask(vmask);
#endif
      dist = tNear;
      return mask;
    }

    /*! intersection with single rays */
    template<int N, bool robust>
      __forceinline size_t intersect_node(const BVH4::Node* node, const TravRay<N>& ray, const vfloat<N>& tnear, const vfloat<N>& tfar, vfloat<N>& dist) 
    {
#if defined (__AVX2__)
      const vfloat<N> tNearX = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+ray.nearX)), ray.rdir.x, ray.org_rdir.x);
      const vfloat<N> tNearY = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+ray.nearY)), ray.rdir.y, ray.org_rdir.y);
      const vfloat<N> tNearZ = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+ray.nearZ)), ray.rdir.z, ray.org_rdir.z);
      const vfloat<N> tFarX  = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+ray.farX )), ray.rdir.x, ray.org_rdir.x);
      const vfloat<N> tFarY  = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+ray.farY )), ray.rdir.y, ray.org_rdir.y);
      const vfloat<N> tFarZ  = msub(vfloat<N>::load((float*)((const char*)&node->lower_x+ray.farZ )), ray.rdir.z, ray.org_rdir.z);
#else
      const vfloat<N> tNearX = (vfloat<N>::load((float*)((const char*)&node->lower_x+ray.nearX)) - ray.org.x) * ray.rdir.x;
      const vfloat<N> tNearY = (vfloat<N>::load((float*)((const char*)&node->lower_x+ray.nearY)) - ray.org.y) * ray.rdir.y;
      const vfloat<N> tNearZ = (vfloat<N>::load((float*)((const char*)&node->lower_x+ray.nearZ)) - ray.org.z) * ray.rdir.z;
      const vfloat<N> tFarX  = (vfloat<N>::load((float*)((const char*)&node->lower_x+ray.farX )) - ray.org.x) * ray.rdir.x;
      const vfloat<N> tFarY  = (vfloat<N>::load((float*)((const char*)&node->lower_x+ray.farY )) - ray.org.y) * ray.rdir.y;
      const vfloat<N> tFarZ  = (vfloat<N>::load((float*)((const char*)&node->lower_x+ray.farZ )) - ray.org.z) * ray.rdir.z;
#endif
      
      if (robust) {
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat<N> tNear = max(tNearX,tNearY,tNearZ,tnear);
        const vfloat<N> tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const vbool4 vmask = (round_down*tNear <= round_up*tFar);
        const size_t mask = movemask(vmask);
        dist = tNear;
        return mask;
      }
      
#if defined(__SSE4_1__)
      const vfloat<N> tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
      const vfloat<N> tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
      const vbool4 vmask = cast(tNear) > cast(tFar);
      const size_t mask = movemask(vmask)^0xf;
#else
      const vfloat<N> tNear = max(tNearX,tNearY,tNearZ,tnear);
      const vfloat<N> tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
      const vbool4 vmask = tNear <= tFar;
      const size_t mask = movemask(vmask);
#endif
      dist = tNear;
      return mask;
    }
    
    /*! intersection with ray packet of size K */
    template<int K, bool robust>
      __forceinline vbool<K> intersect_node(const BVH4::Node* node, size_t i, const Vec3<vfloat<K>>& org, const Vec3<vfloat<K>>& rdir, const Vec3<vfloat<K>>& org_rdir, 
                                            const vfloat<K>& tnear, const vfloat<K>& tfar, vfloat<K>& dist)
    {
#if defined(__AVX2__)
      const vfloat<K> lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
      const vfloat<K> lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
      const vfloat<K> lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
      const vfloat<K> lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
      const vfloat<K> lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
      const vfloat<K> lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
#else
      const vfloat<K> lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
      const vfloat<K> lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
      const vfloat<K> lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
      const vfloat<K> lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
      const vfloat<K> lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
      const vfloat<K> lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
#endif
      
      if (robust) { // FIXME: use per instruction rounding
        const float round_down = 1.0f-2.0f*float(ulp);
        const float round_up   = 1.0f+2.0f*float(ulp);
        const vfloat<K> lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
        const vfloat<K> lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
        const vbool<K> lhit   = round_down*max(lnearP,tnear) <= round_up*min(lfarP,tfar);      
        dist = lnearP;
        return lhit;
      }
      
      const vfloat<K> lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const vfloat<K> lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const vbool<K> lhit    = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
      dist = lnearP;
      return lhit;
    }
    
    /*! intersection with single rays */
    template<int N>
    __forceinline size_t intersect_node(const BVH4::NodeMB* node, size_t nearX, size_t nearY, size_t nearZ,
                                        const Vec3<vfloat<N>>& org, const Vec3<vfloat<N>>& rdir, const Vec3<vfloat<N>>& org_rdir, const vfloat<N>& tnear, const vfloat<N>& tfar, const float time,
                                        vfloat<N>& dist) 
    {
      const size_t farX  = nearX ^ sizeof(vfloat<N>), farY  = nearY ^ sizeof(vfloat<N>), farZ  = nearZ ^ sizeof(vfloat<N>);
      const vfloat<N>* pNearX = (const vfloat<N>*)((const char*)&node->lower_x+nearX);
      const vfloat<N>* pNearY = (const vfloat<N>*)((const char*)&node->lower_x+nearY);
      const vfloat<N>* pNearZ = (const vfloat<N>*)((const char*)&node->lower_x+nearZ);
      const vfloat<N> tNearX = (vfloat<N>(pNearX[0]) + time*pNearX[6] - org.x) * rdir.x;
      const vfloat<N> tNearY = (vfloat<N>(pNearY[0]) + time*pNearY[6] - org.y) * rdir.y;
      const vfloat<N> tNearZ = (vfloat<N>(pNearZ[0]) + time*pNearZ[6] - org.z) * rdir.z;
      const vfloat<N> tNear = max(tnear,tNearX,tNearY,tNearZ);
      const vfloat<N>* pFarX = (const vfloat<N>*)((const char*)&node->lower_x+farX);
      const vfloat<N>* pFarY = (const vfloat<N>*)((const char*)&node->lower_x+farY);
      const vfloat<N>* pFarZ = (const vfloat<N>*)((const char*)&node->lower_x+farZ);
      const vfloat<N> tFarX = (vfloat<N>(pFarX[0]) + time*pFarX[6] - org.x) * rdir.x;
      const vfloat<N> tFarY = (vfloat<N>(pFarY[0]) + time*pFarY[6] - org.y) * rdir.y;
      const vfloat<N> tFarZ = (vfloat<N>(pFarZ[0]) + time*pFarZ[6] - org.z) * rdir.z;
      const vfloat<N> tFar = min(tfar,tFarX,tFarY,tFarZ);
      const size_t mask = movemask(tNear <= tFar);
      dist = tNear;
      return mask;
    }

    /*! intersection with single rays */
    template<int N>
    __forceinline size_t intersect_node(const BVH4::NodeMB* node, const TravRay<N>& ray, const vfloat<N>& tnear, const vfloat<N>& tfar, const float time, vfloat<N>& dist) {
      return intersect_node(node,ray.nearX,ray.nearY,ray.nearZ,ray.org,ray.rdir,ray.org_rdir,tnear,tfar,time,dist);
    }
    
    /*! intersection with ray packet of size K */
    template<int K>
    __forceinline vbool<K> intersect_node(const BVH4::NodeMB* node, const size_t i, const Vec3<vfloat<K>>& org, const Vec3<vfloat<K>>& rdir, const Vec3<vfloat<K>>& org_rdir, 
                                          const vfloat<K>& tnear, const vfloat<K>& tfar, const vfloat<K>& time, vfloat<K>& dist) 
    {
      const vfloat<K> vlower_x = vfloat<K>(node->lower_x[i]) + time * vfloat<K>(node->lower_dx[i]);
      const vfloat<K> vlower_y = vfloat<K>(node->lower_y[i]) + time * vfloat<K>(node->lower_dy[i]);
      const vfloat<K> vlower_z = vfloat<K>(node->lower_z[i]) + time * vfloat<K>(node->lower_dz[i]);
      const vfloat<K> vupper_x = vfloat<K>(node->upper_x[i]) + time * vfloat<K>(node->upper_dx[i]);
      const vfloat<K> vupper_y = vfloat<K>(node->upper_y[i]) + time * vfloat<K>(node->upper_dy[i]);
      const vfloat<K> vupper_z = vfloat<K>(node->upper_z[i]) + time * vfloat<K>(node->upper_dz[i]);
      
#if defined(__AVX2__)
      const vfloat<K> lclipMinX = msub(vlower_x,rdir.x,org_rdir.x);
      const vfloat<K> lclipMinY = msub(vlower_y,rdir.y,org_rdir.y);
      const vfloat<K> lclipMinZ = msub(vlower_z,rdir.z,org_rdir.z);
      const vfloat<K> lclipMaxX = msub(vupper_x,rdir.x,org_rdir.x);
      const vfloat<K> lclipMaxY = msub(vupper_y,rdir.y,org_rdir.y);
      const vfloat<K> lclipMaxZ = msub(vupper_z,rdir.z,org_rdir.z);
#else
      const vfloat<K> lclipMinX = (vlower_x - org.x) * rdir.x;
      const vfloat<K> lclipMinY = (vlower_y - org.y) * rdir.y;
      const vfloat<K> lclipMinZ = (vlower_z - org.z) * rdir.z;
      const vfloat<K> lclipMaxX = (vupper_x - org.x) * rdir.x;
      const vfloat<K> lclipMaxY = (vupper_y - org.y) * rdir.y;
      const vfloat<K> lclipMaxZ = (vupper_z - org.z) * rdir.z;
#endif
      
      const vfloat<K> lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
      const vfloat<K> lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
      const vbool<K> lhit   = maxi(lnearP,tnear) <= mini(lfarP,tfar);      
      dist = lnearP;
      return lhit;
    }
    
    /*! intersect N OBBs with single ray */
    template<int N>
      __forceinline size_t intersect_node(const BVH4::UnalignedNode* node, const Vec3<vfloat<N>>& ray_org, const Vec3<vfloat<N>>& ray_dir, 
                                          const vfloat<N>& tnear, const vfloat<N>& tfar, vfloat<N>& dist)
    {
      const Vec3<vfloat<N>> dir = xfmVector(node->naabb,ray_dir);
      //const Vec3<vfloat<N>> nrdir = Vec3<vfloat<N>>(vfloat<N>(-1.0f))/dir;
      const Vec3<vfloat<N>> nrdir = Vec3<vfloat<N>>(vfloat<N>(-1.0f))*rcp_safe(dir);
      const Vec3<vfloat<N>> org = xfmPoint(node->naabb,ray_org);
      const Vec3<vfloat<N>> tLowerXYZ = org * nrdir;     // (Vec3fa(zero) - org) * rdir;
      const Vec3<vfloat<N>> tUpperXYZ = tLowerXYZ - nrdir; // (Vec3fa(one ) - org) * rdir;
      
      const vfloat<N> tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const vfloat<N> tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const vfloat<N> tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const vfloat<N> tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const vfloat<N> tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const vfloat<N> tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      const vfloat<N> tNear  = max(tnear, tNearX,tNearY,tNearZ);
      const vfloat<N> tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
      const vbool<N> vmask = tNear <= tFar;
      dist = tNear;
      return movemask(vmask);
    }

    /*! intersect N OBBs with single ray */
    template<int N>
      __forceinline size_t intersect_node(const BVH4::UnalignedNode* node, const TravRay<N>& ray, const vfloat<N>& tnear, const vfloat<N>& tfar, vfloat<N>& dist) {
      return intersect_node(node,ray.org,ray.dir,tnear,tfar,dist);
    }
    
    /*! intersect N OBBs with single ray */
    template<int N>
    __forceinline size_t intersect_node(const BVH4::UnalignedNodeMB* node,
                                        const Vec3<vfloat<N>>& ray_org, const Vec3<vfloat<N>>& ray_dir, 
                                        const vfloat<N>& tnear, const vfloat<N>& tfar, const float time, vfloat<N>& dist)
      {
	const vfloat<N> t0 = vfloat<N>(1.0f)-time, t1 = time;

	const AffineSpace3vf4 xfm = node->space0;
	const Vec3<vfloat<N>> b0_lower = zero;
	const Vec3<vfloat<N>> b0_upper = one;
	const Vec3<vfloat<N>> lower = t0*b0_lower + t1*node->b1.lower;
	const Vec3<vfloat<N>> upper = t0*b0_upper + t1*node->b1.upper;
	
	const BBox3vf4 bounds(lower,upper);
	const Vec3<vfloat<N>> dir = xfmVector(xfm,ray_dir);
	const Vec3<vfloat<N>> rdir = rcp_safe(dir); 
	const Vec3<vfloat<N>> org = xfmPoint(xfm,ray_org);
	
	const Vec3<vfloat<N>> tLowerXYZ = (bounds.lower - org) * rdir;
	const Vec3<vfloat<N>> tUpperXYZ = (bounds.upper - org) * rdir;
	
	const vfloat<N> tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
	const vfloat<N> tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
	const vfloat<N> tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
	const vfloat<N> tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
	const vfloat<N> tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
	const vfloat<N> tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
	const vfloat<N> tNear  = max(tnear, tNearX,tNearY,tNearZ);
	const vfloat<N> tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
	const vbool<N> vmask = tNear <= tFar;
	dist = tNear;
	return movemask(vmask);
      }

     /*! intersect N OBBs with single ray */
    template<int N>
      __forceinline size_t intersect_node(const BVH4::UnalignedNodeMB* node, const TravRay<N>& ray, const vfloat<N>& tnear, const vfloat<N>& tfar, const float time, vfloat<N>& dist) {
      return intersect_node(node,ray.org,ray.dir,tnear,tfar,time,dist);
    }
  }
}

