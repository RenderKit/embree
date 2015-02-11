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
    template<typename Mesh>
    PrimInfo createPrimRefArray(Mesh* mesh, vector_t<PrimRef>& prims)
    {
      ParallelPrefixSumState<PrimInfo> pstate;
      
      /* first try */
      PrimInfo pinfo = parallel_prefix_sum( pstate, size_t(0), mesh->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo
      {
        size_t k = r.begin();
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
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
        pinfo = parallel_prefix_sum( pstate, size_t(0), mesh->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo
        {
          size_t k = base.size();
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

    template<typename Mesh, size_t timeSteps>
    PrimInfo createPrimRefArray(Scene* scene, vector_t<PrimRef>& prims)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<Mesh,timeSteps> iter(scene);

      /* first try */
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
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
        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          k = base.size();
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
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

    template<typename Mesh, size_t timeSteps>
      PrimInfo createPrimRefList(Scene* scene, PrimRefList& prims_o)
    {
      Scene::Iterator<Mesh,timeSteps> iter(scene);
      PrimInfo pinfo = parallel_for_for_reduce( iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        PrimRefList::item* block = prims_o.insert(new PrimRefList::item); // FIXME: should store last block in thread local variable!?
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->valid(j,&bounds)) continue;
          const PrimRef prim(bounds,mesh->id,j);
          pinfo.add(prim.bounds(),prim.center2());
          if (likely(block->insert(prim))) continue; 
          block = prims_o.insert(new PrimRefList::item);
          block->insert(prim);
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      return pinfo;
    }

    template<size_t timeSteps>
    PrimInfo createBezierRefArray(Scene* scene, vector_t<BezierPrim>& prims)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<BezierCurves,timeSteps> iter(scene);

      /* first try */
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](BezierCurves* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          const int ofs = mesh->curve(j);
	  Vec3fa p0 = mesh->vertex(ofs+0,0);
	  Vec3fa p1 = mesh->vertex(ofs+1,0);
	  Vec3fa p2 = mesh->vertex(ofs+2,0);
	  Vec3fa p3 = mesh->vertex(ofs+3,0);
	  if (timeSteps == 2) {
	    p0 = 0.5f*(p0+mesh->vertex(ofs+0,1));
	    p1 = 0.5f*(p1+mesh->vertex(ofs+1,1));
	    p2 = 0.5f*(p2+mesh->vertex(ofs+2,1));
	    p3 = 0.5f*(p3+mesh->vertex(ofs+3,1));
	  }
          if (!inFloatRange(p0) || !inFloatRange(p1) || !inFloatRange(p2) || !inFloatRange(p3))
            continue;

	  const BezierPrim bezier(p0,p1,p2,p3,0,1,mesh->id,j,false);
          const BBox3fa bounds = bezier.bounds();
          pinfo.add(bounds);
          prims[k++] = bezier;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](BezierCurves* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          k = base.size();
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            const int ofs = mesh->curve(j);
            Vec3fa p0 = mesh->vertex(ofs+0,0);
            Vec3fa p1 = mesh->vertex(ofs+1,0);
            Vec3fa p2 = mesh->vertex(ofs+2,0);
            Vec3fa p3 = mesh->vertex(ofs+3,0);
            if (timeSteps == 2) {
              p0 = 0.5f*(p0+mesh->vertex(ofs+0,1));
              p1 = 0.5f*(p1+mesh->vertex(ofs+1,1));
              p2 = 0.5f*(p2+mesh->vertex(ofs+2,1));
              p3 = 0.5f*(p3+mesh->vertex(ofs+3,1));
            }
            if (!inFloatRange(p0) || !inFloatRange(p1) || !inFloatRange(p2) || !inFloatRange(p3))
              continue;
            
            const BezierPrim bezier(p0,p1,p2,p3,0,1,mesh->id,j,false);
            const BBox3fa bounds = bezier.bounds();
            pinfo.add(bounds);
            prims[k++] = bezier;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }
    
    template PrimInfo createPrimRefArray<TriangleMesh>(TriangleMesh* mesh, vector_t<PrimRef>& prims);
    template PrimInfo createPrimRefArray<BezierCurves>(BezierCurves* mesh, vector_t<PrimRef>& prims);
    template PrimInfo createPrimRefArray<UserGeometryBase>(UserGeometryBase* mesh, vector_t<PrimRef>& prims);

    template PrimInfo createPrimRefArray<TriangleMesh,1>(Scene* scene, vector_t<PrimRef>& prims);
    template PrimInfo createPrimRefArray<TriangleMesh,2>(Scene* scene, vector_t<PrimRef>& prims);
    template PrimInfo createPrimRefArray<BezierCurves,1>(Scene* scene, vector_t<PrimRef>& prims);
    template PrimInfo createPrimRefArray<SubdivMesh,1>(Scene* scene, vector_t<PrimRef>& prims);
    template PrimInfo createPrimRefArray<UserGeometryBase,1>(Scene* scene, vector_t<PrimRef>& prims);

    template PrimInfo createBezierRefArray<1>(Scene* scene, vector_t<BezierPrim>& prims);
    template PrimInfo createBezierRefArray<2>(Scene* scene, vector_t<BezierPrim>& prims);

    template PrimInfo createPrimRefList<TriangleMesh,1>(Scene* scene, PrimRefList& prims);
  }
}

