// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "subdivpatch1.h"
#include "common/ray16.h"
#include "geometry/filter.h"
#include "triangle1mc_intersector1_moeller.h"
#include "triangle1mc_intersector16_moeller.h"

namespace embree
{

  template< bool ENABLE_INTERSECTION_FILTER>
  struct SubdivPatchIntersector
  {
    typedef SubdivPatch1 Primitive;



    static __forceinline bool intersect16(Ray16& ray, 
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const size_t k, 
					  const SubdivPatch1& patch)
    {
      STAT3(normal.trav_prims,1,1,1);
      __aligned(64) Triangle1mc tri4[4];
      Vec3fa *__restrict__ vtx0 = (Vec3fa *)&patch.getQuadVertex(0);
      Vec3fa *__restrict__ vtx1 = (Vec3fa *)&patch.getQuadVertex(1);
      Vec3fa *__restrict__ vtx2 = (Vec3fa *)&patch.getQuadVertex(2);
      Vec3fa *__restrict__ vtx3 = (Vec3fa *)&patch.getQuadVertex(3);
      tri4[0] = Triangle1mc(vtx0,vtx1,vtx2,patch.geomID,patch.primID);
      tri4[1] = Triangle1mc(vtx2,vtx3,vtx0,patch.geomID,patch.primID);
      tri4[2] = Triangle1mc(vtx0,vtx1,vtx2,patch.geomID,patch.primID);
      tri4[3] = Triangle1mc(vtx2,vtx3,vtx0,patch.geomID,patch.primID);


      return true;
    }

    static __forceinline bool occluded16(const Ray16& ray, 
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const size_t k, 
					 const SubdivPatch1& patch) 
    {
      STAT3(shadow.trav_prims,1,1,1);
      return true;
    }


    // ==================================================================
    // ==================================================================
    // ==================================================================


    static __forceinline bool intersect1(Ray& ray, 
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const SubdivPatch1& patch)
    {
      STAT3(normal.trav_prims,1,1,1);

      return true;
    }

    static __forceinline bool occluded1(const Ray& ray, 
					const mic_f &dir_xyz,
					const mic_f &org_xyz,
					const SubdivPatch1& patch) 
    {
      STAT3(shadow.trav_prims,1,1,1);
      return true;
    }

  };
}
