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
    struct RTCRayGPU : public RTCRay
    {
      inline cl::sycl::float3 broadcast_org(const cl::sycl::intel::sub_group &subgroup, const uint index) {
	return cl::sycl::float3(subgroup.broadcast<float>(org_x,index),
				subgroup.broadcast<float>(org_y,index),
				subgroup.broadcast<float>(org_z,index));	
      }

      inline cl::sycl::float3 broadcast_dir(const cl::sycl::intel::sub_group &subgroup, const uint index) {
	return cl::sycl::float3(subgroup.broadcast<float>(dir_x,index),
				subgroup.broadcast<float>(dir_y,index),
				subgroup.broadcast<float>(dir_z,index));	
      }

      inline float broadcast_tnear(const cl::sycl::intel::sub_group &subgroup, const uint index) { return subgroup.broadcast<float>(tnear,index); }
      inline float broadcast_tfar (const cl::sycl::intel::sub_group &subgroup, const uint index) { return subgroup.broadcast<float>(tfar ,index); }
      
      
      
    };

    /* Hit structure for a single ray */
    struct RTCHitGPU : public RTCHit
    {
      inline void init()
      {
	primID = -1;
	geomID = -1;
	//instID[0] = 0;
	u = 0.0f;
	v = 0.0f;
	Ng_x = Ng_y = Ng_z = 0.0f;
      }

      inline void broadcast(const cl::sycl::intel::sub_group &subgroup, const uint index)
      {
	Ng_x = subgroup.broadcast<float>(Ng_x,index);
	Ng_y = subgroup.broadcast<float>(Ng_y,index);
	Ng_z = subgroup.broadcast<float>(Ng_z,index);
	u = subgroup.broadcast<float>(u,index);
	v = subgroup.broadcast<float>(v,index);
	primID = subgroup.broadcast<unsigned int>(primID,index);
	geomID = subgroup.broadcast<unsigned int>(geomID,index);
	//instID = subgroup.broadcast<unsigned int>(instID,index);
      }
      
      inline void store(RTCHit &dest)
      {
	dest.Ng_x = Ng_x;
	dest.Ng_y = Ng_y;
	dest.Ng_z = Ng_z;
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
      return out << " Ng " << float3(hit.Ng_x,hit.Ng_y,hit.Ng_z)
		 << " u " << hit.u
		 << " v " << hit.v
		 << " primID " << hit.primID
		 << " geomID " << hit.geomID
		 << " instID " << hit.instID;
      
    }


    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const RTCRayGPU& ray) {
      return out << " org   " << float3(ray.org_x,ray.org_y,ray.org_z)
		 << " tnear " << ray.tnear
		 << " dir   " << float3(ray.dir_x,ray.dir_y,ray.dir_z)
		 << " tnear " << ray.tfar;
      
    }
    

  };
};

#endif
