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

#include "../common/ray.h"

#if defined(EMBREE_DPCPP_SUPPORT)
#include "common.h"

namespace embree
{
  namespace gpu
  {
    // FIXME: can't use float3 here due to stupid size/alignment restriction
    
    /* Ray structure for a single ray */
    struct RTCRayGPU : public Ray
    {
      inline Vec3f get_org() {
	return Vec3f(org.x,org.y,org.z);	
      }

      inline Vec3f get_dir() {
	return Vec3f(dir.x,dir.y,dir.z);	
      }
      
      inline Vec3f broadcast_org(const cl::sycl::intel::sub_group &subgroup, const uint index) {
	return Vec3f(subgroup.broadcast<float>(org.x,index),
				subgroup.broadcast<float>(org.y,index),
				subgroup.broadcast<float>(org.z,index));	
      }

      inline Vec3f broadcast_dir(const cl::sycl::intel::sub_group &subgroup, const uint index) {
	return Vec3f(subgroup.broadcast<float>(dir.x,index),
				subgroup.broadcast<float>(dir.y,index),
				subgroup.broadcast<float>(dir.z,index));	
      }

      inline float broadcast_tnear(const cl::sycl::intel::sub_group &subgroup, const uint index) { return subgroup.broadcast<float>(tnear(),index); }
      inline float broadcast_tfar (const cl::sycl::intel::sub_group &subgroup, const uint index) { return subgroup.broadcast<float>(tfar ,index); }
      
      
      
    };

    /* Hit structure for a single ray */
    struct RTCHitGPU : public Hit
    {
      inline void init()
      {
	primID = -1;
	geomID = -1;
      }
      
      inline void broadcast(const cl::sycl::intel::sub_group &subgroup, const uint index)
      {
	Ng.x = subgroup.broadcast<float>(Ng.x,index);
	Ng.y = subgroup.broadcast<float>(Ng.y,index);
	Ng.z = subgroup.broadcast<float>(Ng.z,index);
	u = subgroup.broadcast<float>(u,index);
	v = subgroup.broadcast<float>(v,index);
	primID = subgroup.broadcast<unsigned int>(primID,index);
	geomID = subgroup.broadcast<unsigned int>(geomID,index);
	//instID = subgroup.broadcast<unsigned int>(instID,index);
      }
      
      inline void store(RTCHit &dest)
      {
	dest.Ng_x = Ng.x;
	dest.Ng_y = Ng.y;
	dest.Ng_z = Ng.z;
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
      return out << " Ng " << float3(hit.Ng.x,hit.Ng.y,hit.Ng.z)
		 << " u " << hit.u
		 << " v " << hit.v
		 << " primID " << hit.primID
		 << " geomID " << hit.geomID
		 << " instID " << hit.instID;
      
    }


    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const RTCRayGPU& ray) {
      return out << " org   " << float3(ray.org.x,ray.org.y,ray.org.z)
		 << " tnear " << ray.tnear()
		 << " dir   " << float3(ray.dir.x,ray.dir.y,ray.dir.z)
		 << " tnear " << ray.tfar;
      
    }
    

  };
};

#endif
