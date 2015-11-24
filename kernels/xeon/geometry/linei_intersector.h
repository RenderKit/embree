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

#include "linei.h"
#include "line_intersector.h"

namespace embree
{
  namespace isa
  {
    template<int M, int Mx>
    struct Line4iIntersector1
    {
      typedef Line4i Primitive;
      typedef typename LineIntersector1<Mx>::Precalculations Precalculations;

      static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive& line, Scene* scene, const unsigned* geomID_to_instID)
      {
        //STAT3(normal.trav_prims,1,1,1);
        Vec4vf4 v0, v1; line.gather(v0,v1);
        LineIntersector1<Mx>::intersect(ray,pre,v0,v1,line.geomID(),line.primID(),scene,geomID_to_instID);
      }

      static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive& line, Scene* scene, const unsigned* geomID_to_instID)
      {
        //STAT3(shadow.trav_prims,1,1,1);
        Vec4vf4 v0, v1; line.gather(v0,v1);
        //return LineIntersector1<Mx>::intersect(ray,pre,v0,v1,line.geomID(),line.primID(),scene,geomID_to_instID);
        return false;
      }
    };
  }
}
