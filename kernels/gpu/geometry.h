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
    

  };
};

#endif
