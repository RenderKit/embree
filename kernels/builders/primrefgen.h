// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../common/scene.h"
#include "../common/primref.h"
#include "../common/primref_mb.h"
#include "priminfo.h"
#include "../geometry/bezier1v.h"
#include "bvh_builder_morton.h"

namespace embree
{
  namespace isa
  {
    template<typename Mesh>
      PrimInfo createPrimRefArray(Mesh* mesh, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template<typename Mesh>
      PrimInfo createGroupPrimRefArray(GeometryGroup* group, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template<typename Mesh, bool mblur>
      PrimInfo createPrimRefArray(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template<typename Mesh>
      PrimInfo createPrimRefArrayMBlur(size_t timeSegment, Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template<typename Mesh>
      PrimInfoMB createPrimRefArrayMSMBlur(Scene* scene, mvector<PrimRefMB>& prims, BuildProgressMonitor& progressMonitor, BBox1f t0t1 = BBox1f(0.0f,1.0f));

    template<typename Mesh>
      size_t createMortonCodeArray(Mesh* mesh, mvector<BVHBuilderMorton::BuildPrim>& morton, BuildProgressMonitor& progressMonitor);
  }
}

