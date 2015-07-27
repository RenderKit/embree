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

#include "../../algorithms/parallel_for_for.h"
#include "../../algorithms/parallel_for_for_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    template<typename Mesh>
    PrimInfo createPrimRefArray(Mesh* mesh, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelPrefixSumState<PrimInfo> pstate;
      
      /* first try */
      progressMonitor(0);
      PrimInfo pinfo = parallel_prefix_sum( pstate, size_t(0), mesh->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo
      {
        size_t k = r.begin();
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->valid(j,&bounds)) continue;
          const PrimRef prim(bounds,mesh->id,j);
          pinfo.add(bounds,bounds.center2());
          prims[k++] = prim;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_prefix_sum( pstate, size_t(0), mesh->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo
        {
          size_t k = base.size();
          PrimInfo pinfo(empty);
          for (ssize_t j=r.begin(); j<r.end(); j++)
          {
            BBox3fa bounds = empty;
            if (!mesh->valid(j,&bounds)) continue;
            const PrimRef prim(bounds,mesh->id,j);
            pinfo.add(bounds,bounds.center2());
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    template<typename Mesh, size_t timeSteps>
    PrimInfo createPrimRefArray(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<Mesh,timeSteps> iter(scene);
      
      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->valid(j,&bounds)) continue;
          const PrimRef prim(bounds,mesh->id,j);
          pinfo.add(bounds,bounds.center2());
          prims[k++] = prim;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          k = base.size();
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            BBox3fa bounds = empty;
            if (!mesh->valid(j,&bounds)) continue;
            const PrimRef prim(bounds,mesh->id,j);
            pinfo.add(bounds,bounds.center2());
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    template<typename Mesh, size_t timeSteps>
      PrimInfo createPrimRefList(Scene* scene, PrimRefList& prims_o, BuildProgressMonitor& progressMonitor)
    {
      progressMonitor(0);
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
          pinfo.add(bounds,bounds.center2());
          if (likely(block->insert(prim))) continue; 
          block = prims_o.insert(new PrimRefList::item);
          block->insert(prim);
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      return pinfo;
    }

    template<size_t timeSteps>
    PrimInfo createBezierRefArray(Scene* scene, mvector<BezierPrim>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<BezierCurves,timeSteps> iter(scene);

      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](BezierCurves* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          const int ofs = mesh->curve(j);
          if (ofs+3 >= mesh->numVertices())
            continue;

          BBox3fa bounds = empty;
	  Vec3fa p0 = mesh->vertex(ofs+0,0); bounds.extend(p0);
	  Vec3fa p1 = mesh->vertex(ofs+1,0); bounds.extend(p1);
	  Vec3fa p2 = mesh->vertex(ofs+2,0); bounds.extend(p2);
	  Vec3fa p3 = mesh->vertex(ofs+3,0); bounds.extend(p3);
	  if (timeSteps == 2) {
            Vec3fa q0 = mesh->vertex(ofs+0,1); bounds.extend(q0); p0 = 0.5f*(p0+q0);
            Vec3fa q1 = mesh->vertex(ofs+1,1); bounds.extend(q1); p1 = 0.5f*(p1+q1);
	    Vec3fa q2 = mesh->vertex(ofs+2,1); bounds.extend(q2); p2 = 0.5f*(p2+q2);
	    Vec3fa q3 = mesh->vertex(ofs+3,1); bounds.extend(q3); p3 = 0.5f*(p3+q3);
	  }
          if (!isvalid((float4)p0) || !isvalid((float4)p1) || !isvalid((float4)p2) || !isvalid((float4)p3))
              continue;

	  const BezierPrim bezier(p0,p1,p2,p3,0,1,mesh->id,j,false);
          pinfo.add(bounds,bezier.bounds().center2());
          prims[k++] = bezier;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](BezierCurves* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          k = base.size();
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            const int ofs = mesh->curve(j);
            if (ofs+3 >= mesh->numVertices())
              continue;

            BBox3fa bounds = empty;
            Vec3fa p0 = mesh->vertex(ofs+0,0); bounds.extend(p0);
            Vec3fa p1 = mesh->vertex(ofs+1,0); bounds.extend(p1);
            Vec3fa p2 = mesh->vertex(ofs+2,0); bounds.extend(p2);
            Vec3fa p3 = mesh->vertex(ofs+3,0); bounds.extend(p3);
            if (timeSteps == 2) {
              Vec3fa q0 = mesh->vertex(ofs+0,1); bounds.extend(q0); p0 = 0.5f*(p0+q0);
              Vec3fa q1 = mesh->vertex(ofs+1,1); bounds.extend(q1); p1 = 0.5f*(p1+q1);
              Vec3fa q2 = mesh->vertex(ofs+2,1); bounds.extend(q2); p2 = 0.5f*(p2+q2);
              Vec3fa q3 = mesh->vertex(ofs+3,1); bounds.extend(q3); p3 = 0.5f*(p3+q3);
            }
            if (!isvalid((float4)p0) || !isvalid((float4)p1) || !isvalid((float4)p2) || !isvalid((float4)p3))
              continue;
            
            const BezierPrim bezier(p0,p1,p2,p3,0,1,mesh->id,j,false);
            pinfo.add(bounds,bezier.bounds().center2());
            prims[k++] = bezier;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }
    
    template PrimInfo createPrimRefArray<TriangleMesh>(TriangleMesh* mesh, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);
    template PrimInfo createPrimRefArray<BezierCurves>(BezierCurves* mesh, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);
    template PrimInfo createPrimRefArray<AccelSet>(AccelSet* mesh, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template PrimInfo createPrimRefArray<TriangleMesh,1>(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);
    template PrimInfo createPrimRefArray<TriangleMesh,2>(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);
    template PrimInfo createPrimRefArray<BezierCurves,1>(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);
    template PrimInfo createPrimRefArray<SubdivMesh,1>(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);
    template PrimInfo createPrimRefArray<AccelSet,1>(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

    template PrimInfo createBezierRefArray<1>(Scene* scene, mvector<BezierPrim>& prims, BuildProgressMonitor& progressMonitor);
    template PrimInfo createBezierRefArray<2>(Scene* scene, mvector<BezierPrim>& prims, BuildProgressMonitor& progressMonitor);

    template PrimInfo createPrimRefList<TriangleMesh,1>(Scene* scene, PrimRefList& prims, BuildProgressMonitor& progressMonitor);
  }
}

