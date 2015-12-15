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

#include "../../common/scene.h"
#include "../../common/primref.h"
#include "priminfo.h"
#include "../geometry/bezier1v.h"

namespace embree
{
  namespace isa
  {
    template<typename Mesh>
      PrimInfo createPrimRefArray(Mesh* mesh, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template<typename Mesh, size_t timeSteps>
      PrimInfo createPrimRefArray(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template<typename Mesh, size_t timeSteps>
      PrimInfo createPrimRefList(Scene* scene, PrimRefList& prims, BuildProgressMonitor& progressMonitor);

    template<size_t timeSteps>
      PrimInfo createBezierRefArray(Scene* scene, mvector<BezierPrim>& prims, BuildProgressMonitor& progressMonitor);
  }
}

