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

#include "../../common/ray.h"
#include "../../common/scene_subdiv_mesh.h"
#include "filter.h"
#include "../bvh4/bvh4.h"
#include "../../common/subdiv/tessellation.h"
#include "../../common/subdiv/tessellation_cache.h"
#include "subdivpatch1cached.h"

namespace embree
{
  namespace isa
  {
    class GridSOA
    {
    public:

      /*! GridSOA constructor */
      GridSOA(const SubdivPatch1Cached& patch, const SubdivMesh* const geom);

      static size_t lazyBuildPatch(SubdivPatch1Cached* const subdiv_patch, const Scene* scene);                  
      
      /*! Evaluates grid over patch and builds BVH4 tree over the grid. */
      static BVH4::NodeRef buildBVH(const SubdivPatch1Cached &patch, GridSOA* grid_array);
      
      /*! Create BVH4 tree over grid. */
      static BBox3fa createSubTreeCompact(BVH4::NodeRef &curNode,
					  float *const lazymem,
					  const SubdivPatch1Cached &patch,
					  const float *const grid_array,
					  const size_t grid_array_elements,
					  const GridRange &range,
					  unsigned int &localCounter);
    };
  }
}
