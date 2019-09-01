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
    };

    /* Combined ray/hit structure for a single ray */
    struct RTCRayHitGPU
    {
      struct RTCRayGPU ray;
      struct RTCHitGPU hit;
    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const RTCHitGPU& hit) {
      return out << " Ng " << cl::sycl::float3(hit.Ng[0],hit.Ng[1],hit.Ng[2])
		 << " u " << hit.u
		 << " v " << hit.v
		 << " primID " << hit.primID
		 << " geomID " << hit.geomID
		 << " instID " << hit.instID;
      
    }


  };
};

#endif
