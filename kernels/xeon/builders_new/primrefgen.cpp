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

#include "primrefgen.h"

#include "algorithms/parallel_for_for.h"
#include "algorithms/parallel_for_for_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    template<typename Ty, size_t timeSteps>
    PrimInfo createPrimRefArray(Scene* scene, vector_t<PrimRef>& prims)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<Ty,timeSteps> iter(scene);
      
      /* first try */
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Ty* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
						    {
						      PrimInfo pinfo(empty);
						      for (ssize_t j=r.begin(); j<r.end(); j++)
							{
							  BBox3fa bounds = empty;
							  if (!mesh->valid(j,&bounds)) continue;
							  const PrimRef prim(bounds,mesh->id,j);
							  pinfo.add(prim.bounds(),prim.center2());
							  prims[k++] = prim;
							}
						      return pinfo;
						    }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
	{
	  pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Ty* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
					       {
						 k = base.begin;
						 PrimInfo pinfo(empty);
						 for (ssize_t j=r.begin(); j<r.end(); j++)
						   {
						     BBox3fa bounds = empty;
						     if (!mesh->valid(j,&bounds)) continue;
						     const PrimRef prim(bounds,mesh->id,j);
						     pinfo.add(prim.bounds(),prim.center2());
						     prims[k++] = prim;
						   }
						 return pinfo;
					       }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
	}
      return pinfo;
    }
    
    template PrimInfo createPrimRefArray<TriangleMesh,1>(Scene* scene, vector_t<PrimRef>& prims);
  }
}

