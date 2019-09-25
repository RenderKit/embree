// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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

#if defined(EMBREE_DPCPP_SUPPORT)
#include "common.h"
#include "AABB.h"
#include "ray.h"

namespace embree
{
  namespace gpu
  {

    struct Quad1v
    {
      float4 v0,v2,v1,v3; //special optimized layout

      inline void init(const float4 &_v0,
		       const float4 &_v1,
		       const float4 &_v2,
		       const float4 &_v3,
		       const uint geomID,
		       const uint primID)
      {
	v0 = _v0;
	v1 = _v1;
	v2 = _v2;
	v3 = _v3;
	v0.w() = 0.0f;
	v2.w() = 0.0f;	
	v1.w() = as_float(geomID);
	v3.w() = as_float(primID);
      }

      inline AABB getBounds()
      {
	AABB aabb;
	aabb.init();
	aabb.extend(v0);
	aabb.extend(v1);
	aabb.extend(v2);
	aabb.extend(v3);
	return aabb;
      }
      
    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const Quad1v& quad) {
      return out << "v0 " << quad.v0
		 << "v1 " << quad.v1
		 << "v2 " << quad.v2
		 << "v3 " << quad.v3
		 << "geomID " << gpu::as_int((float)quad.v1.w())
		 << "primID0 " << gpu::as_int((float)quad.v0.w())
		 << "primID1 " << gpu::as_int((float)quad.v2.w());	      
    }

    struct Triangle1v
    {
      float4 v0,v1,v2; 

      inline void init(const float4 &_v0,
		       const float4 &_v1,
		       const float4 &_v2,
		       const uint geomID,
		       const uint primID)
      {
	v0 = _v0;
	v1 = _v1;
	v2 = _v2;
	v0.w() = as_float(primID);
	v1.w() = as_float(geomID);
	v2.w() = 0.0f;	
      }

      inline AABB getBounds()
      {
	AABB aabb;
	aabb.init();
	aabb.extend(v0);
	aabb.extend(v1);
	aabb.extend(v2);
	return aabb;
      }
      
    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const Triangle1v& tri) {
      return out << "v0 " << tri.v0
		 << "v1 " << tri.v1
		 << "v2 " << tri.v2
		 << "geomID " << gpu::as_int((float)tri.v1.w())
		 << "primID " << gpu::as_int((float)tri.v0.w());
    }
    
    inline float intersectPrimitive1v(const cl::sycl::intel::sub_group &sg,
				      const gpu::Quad1v *const quad1v,
				      const uint numQuads,
				      const float3 &org,
				      const float3 &dir,
				      const float &tnear,
				      const float &tfar,
				      gpu::RTCHitGPU &hit,
				      const unsigned int slotID)
    {
      float new_tfar = tfar;
      const uint quadID = slotID >> 1;
      
      if (slotID < numQuads*2)
	{
#if 0
	  const float4 _v0 = cselect((int)((slotID % 2) == 0),quad1v[quadID].v0,quad1v[quadID].v2);
#else	  
	  const float4 *const _v0_ptr = (float4*)&quad1v[quadID];
	  const float4 _v0 = _v0_ptr[slotID & 1];
#endif	  
	  const float4 _v1 = quad1v[quadID].v1;
	  const float4 _v2 = quad1v[quadID].v3;
	  const uint geomID = gpu::as_uint((float)_v1.w());
	  const uint primID = gpu::as_uint((float)_v2.w());	  	  
	  const float3 v0 = _v0.xyz();
	  const float3 v1 = _v1.xyz();
	  const float3 v2 = _v2.xyz();

	  /* moeller-trumbore test */	  
	  const float3 e1 = v0 - v1;
	  const float3 e2 = v2 - v0;
	  const float3 tri_Ng = cl::sycl::cross(e1,e2);
	  const float den = dot3(tri_Ng,dir);   			   
	  const float inv_den = cl::sycl::native::recip(den); 
	  const float3 tri_v0_org = v0 - org;
	  const float3 R = cl::sycl::cross(dir,tri_v0_org);
	  const float u = dot3(R,e2) * inv_den;
	  const float v = dot3(R,e1) * inv_den;
	  float t = dot3(tri_v0_org,tri_Ng) * inv_den; 
	  int m_hit = (u >= 0.0f) & (v >= 0.0f) & (u+v <= 1.0f);
	  //if (m_hit == 0) return; // early out
	  m_hit &= (tnear <= t) & (t <= tfar); // den != 0.0f &&
	  if (m_hit) 
	    {
	      new_tfar = t;
	      hit.Ng[0]  = tri_Ng.x();
	      hit.Ng[1]  = tri_Ng.y();
	      hit.Ng[2]  = tri_Ng.z();	      
	      hit.u      = u;
	      hit.v      = v;
	      hit.primID = primID;
	      hit.geomID = geomID;	      
	    }	  
	}
      return new_tfar;
    }


    inline float intersectPrimitive1v(const cl::sycl::intel::sub_group &sg,
				      const gpu::Triangle1v *const tri1v,
				      const uint numTris,
				      const float3 &org,
				      const float3 &dir,
				      const float &tnear,
				      const float &tfar,
				      gpu::RTCHitGPU &hit,
				      const unsigned int slotID)
    {
      float new_tfar = tfar;
      if (slotID < numTris)
	{
	  const float4 _v0 = tri1v[slotID].v0;
	  const float4 _v1 = tri1v[slotID].v1;
	  const float4 _v2 = tri1v[slotID].v2;
	  const uint primID = gpu::as_uint((float)_v0.w());	  
	  const uint geomID = gpu::as_uint((float)_v1.w());	    
	  const float3 v0 = _v0.xyz();
	  const float3 v1 = _v1.xyz();
	  const float3 v2 = _v2.xyz();

	  /* moeller-trumbore test */	  
	  const float3 e1 = v0 - v1;
	  const float3 e2 = v2 - v0;
	  const float3 tri_Ng = cl::sycl::cross(e1,e2);
	  const float den = dot3(tri_Ng,dir);   			   
	  const float inv_den = cl::sycl::native::recip(den); 
	  const float3 tri_v0_org = v0 - org;
	  const float3 R = cl::sycl::cross(dir,tri_v0_org);
	  const float u = dot3(R,e2) * inv_den;
	  const float v = dot3(R,e1) * inv_den;
	  float t = dot3(tri_v0_org,tri_Ng) * inv_den; 
	  int m_hit = (u >= 0.0f) & (v >= 0.0f) & (u+v <= 1.0f);
	  //if (m_hit == 0) return; // early out
	  m_hit &= (tnear <= t) & (t <= tfar); // den != 0.0f &&
	  if (m_hit) 
	    {
	      new_tfar = t;
	      hit.Ng[0]  = tri_Ng.x();
	      hit.Ng[1]  = tri_Ng.y();
	      hit.Ng[2]  = tri_Ng.z();	      
	      hit.u      = u;
	      hit.v      = v;
	      hit.primID = primID;
	      hit.geomID = geomID;	      
	    }	  
	}
      return new_tfar;
    }

  };
};

#endif
