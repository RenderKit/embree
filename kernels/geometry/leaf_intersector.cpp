// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "../common/accel.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector.h"
#include "../geometry/trianglev_intersector.h"
#include "../geometry/trianglev_mb_intersector.h"
#include "../geometry/trianglei_intersector.h"
#include "../geometry/quadv_intersector.h"
#include "../geometry/quadi_intersector.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/linei_intersector.h"
#include "../geometry/subdivpatch1eager_intersector.h"
#include "../geometry/subdivpatch1cached_intersector.h"
#include "../geometry/object_intersector.h"

namespace embree
{
  namespace isa
  {
    LeafIntersector* LeafIntersectorFast()
    {
      typedef ArrayIntersector1<TriangleMIntersector1Moeller  <SIMD_MODE(4) COMMA true> > Slot0;
      typedef ArrayIntersector1<TriangleMiMBIntersector1Moeller <SIMD_MODE(4) COMMA true> > Slot1;
      typedef ArrayIntersector1<QuadMvIntersector1Moeller <4 COMMA true> > Slot2;
      typedef ArrayIntersector1<QuadMiMBIntersector1Moeller <4 COMMA true> > Slot3;
      typedef ArrayIntersector1<Bezier1vIntersector1> Slot4;
      typedef ArrayIntersector1<Bezier1iIntersector1MB> Slot5;
      typedef ArrayIntersector1<LineMiIntersector1<SIMD_MODE(4) COMMA true> > Slot6;
      typedef ArrayIntersector1<LineMiMBIntersector1<SIMD_MODE(4) COMMA true> > Slot7;

      static LeafIntersector intersector;
      intersector.vtable1[0].intersect = Slot0::vintersect;
      intersector.vtable1[0].occluded  = Slot0::voccluded;
      intersector.vtable1[1].intersect = Slot1::vintersect;
      intersector.vtable1[1].occluded  = Slot1::voccluded;
      intersector.vtable1[2].intersect = Slot2::vintersect;
      intersector.vtable1[2].occluded  = Slot2::voccluded;
      intersector.vtable1[3].intersect = Slot3::vintersect;
      intersector.vtable1[3].occluded  = Slot3::voccluded;
      intersector.vtable1[4].intersect = Slot4::vintersect;
      intersector.vtable1[4].occluded  = Slot4::voccluded;
      intersector.vtable1[5].intersect = Slot5::vintersect;
      intersector.vtable1[5].occluded  = Slot5::voccluded;
      intersector.vtable1[6].intersect = Slot6::vintersect;
      intersector.vtable1[6].occluded  = Slot6::voccluded;
      intersector.vtable1[7].intersect = Slot7::vintersect;
      intersector.vtable1[7].occluded  = Slot7::voccluded;
      return &intersector;
    }
  }
}
