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

namespace embree
{
  namespace gpu
  {
    // FIXME: can't use float3 here due to stupid size/alignment restriction
    
    /* Ray structure for a single ray */
    struct RTCRayGPU
    {
      float org[3];         // x,y,z coordinates of ray origin
      float tnear;          // start of ray segment
      float dir[3];         // x,y,z coordinate of ray direction
      float time;           // time of this ray for motion blur
      float tfar;           // end of ray segment (set to hit distance)
      unsigned int mask;    // ray mask
      unsigned int id;      // ray ID
      unsigned int flags;   // ray flags

      inline void init(RTCRay &source)
      {
	org[0] = source.org_x; 
	org[1] = source.org_y; 
	org[2] = source.org_z; 
	tnear  = source.tnear; 
	dir[0] = source.dir_x; 
	dir[1] = source.dir_y; 
	dir[2] = source.dir_z; 
	time   = source.time; 
	tfar   = source.tfar; 
	mask   = source.mask; 
	id     = source.id; 
	flags  = source.flags; 
      }

      inline cl::sycl::float3 broadcast_org(const cl::sycl::intel::sub_group &subgroup, const uint index) {
	return cl::sycl::float3(subgroup.broadcast<float>(org[0],index),
				subgroup.broadcast<float>(org[1],index),
				subgroup.broadcast<float>(org[2],index));	
      }

      inline cl::sycl::float3 broadcast_dir(const cl::sycl::intel::sub_group &subgroup, const uint index) {
	return cl::sycl::float3(subgroup.broadcast<float>(dir[0],index),
				subgroup.broadcast<float>(dir[1],index),
				subgroup.broadcast<float>(dir[2],index));	
      }

      inline float broadcast_tnear(const cl::sycl::intel::sub_group &subgroup, const uint index) { return subgroup.broadcast<float>(tnear,index); }
      inline float broadcast_tfar (const cl::sycl::intel::sub_group &subgroup, const uint index) { return subgroup.broadcast<float>(tfar ,index); }
      
      
      
    };

    /* Hit structure for a single ray */
    struct RTCHitGPU
    {
      float Ng[3];         // x,y,z coordinates of geometry normal
      float u;             // barycentric u coordinate of hit
      float v;             // barycentric v coordinate of hit
      unsigned int primID; // primitive ID
      unsigned int geomID; // geometry ID
      unsigned int instID; // instance ID

      inline void init()
      {
	primID = -1;
	geomID = -1;
	instID = 0;
	u = 0.0f;
	v = 0.0f;
	Ng[0] = Ng[1] = Ng[2] = 0.0f;
      }

      inline void broadcast(const cl::sycl::intel::sub_group &subgroup, const uint index)
      {
	Ng[0] = subgroup.broadcast<float>(Ng[0],index);
	Ng[1] = subgroup.broadcast<float>(Ng[1],index);
	Ng[2] = subgroup.broadcast<float>(Ng[2],index);
	u = subgroup.broadcast<float>(u,index);
	v = subgroup.broadcast<float>(v,index);
	primID = subgroup.broadcast<unsigned int>(primID,index);
	geomID = subgroup.broadcast<unsigned int>(geomID,index);
	instID = subgroup.broadcast<unsigned int>(instID,index);
      }
      
      inline void store(RTCHit &dest)
      {
	dest.Ng_x = Ng[0];
	dest.Ng_y = Ng[1];
	dest.Ng_z = Ng[2];
	dest.u     = u;
	dest.v     = v;
	dest.primID = primID;
	dest.geomID = geomID;
	//dest.instID[0] = instID[0];
      }      

    };

    /* Combined ray/hit structure for a single ray */
    struct RTCRayHitGPU
    {
      struct RTCRayGPU ray;
      struct RTCHitGPU hit;
    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const RTCHitGPU& hit) {
      return out << " Ng " << float3(hit.Ng[0],hit.Ng[1],hit.Ng[2])
		 << " u " << hit.u
		 << " v " << hit.v
		 << " primID " << hit.primID
		 << " geomID " << hit.geomID
		 << " instID " << hit.instID;
      
    }


    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const RTCRayGPU& ray) {
      return out << " org   " << float3(ray.org[0],ray.org[1],ray.org[2])
		 << " tnear " << ray.tnear
		 << " dir   " << float3(ray.dir[0],ray.dir[1],ray.dir[2])
		 << " tnear " << ray.tfar;
      
    }
    

  };
};

#endif
