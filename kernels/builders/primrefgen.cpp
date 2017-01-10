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

#include "primrefgen.h"

#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

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
          if (!mesh->buildBounds(j,&bounds)) continue;

          const PrimRef prim(bounds,mesh->id,unsigned(j));          
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
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            BBox3fa bounds = empty;
            if (!mesh->buildBounds(j,&bounds)) continue;
            const PrimRef prim(bounds,mesh->id,unsigned(j));
            pinfo.add(bounds,bounds.center2());
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    template<typename Mesh, bool mblur>
    PrimInfo createPrimRefArray(Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<Mesh,mblur> iter(scene);
      
      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->buildBounds(j,&bounds)) continue;
          const PrimRef prim(bounds,mesh->id,unsigned(j));
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
            if (!mesh->buildBounds(j,&bounds)) continue;
            const PrimRef prim(bounds,mesh->id,unsigned(j));
            pinfo.add(bounds,bounds.center2());
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    template<typename Mesh>
    PrimInfo createPrimRefArrayMBlur(size_t timeSegment, size_t numTimeSteps, Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<Mesh,true> iter(scene);

      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->buildBounds(j,timeSegment,numTimeSteps,bounds)) continue;
          const PrimRef prim(bounds,mesh->id,unsigned(j));
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
            if (!mesh->buildBounds(j,timeSegment,numTimeSteps,bounds)) continue;
            const PrimRef prim(bounds,mesh->id,unsigned(j));
            pinfo.add(bounds,bounds.center2());
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    PrimInfo createBezierRefArray(Scene* scene, mvector<BezierPrim>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<BezierCurves,false> iter(scene);

      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](BezierCurves* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          const size_t ofs = mesh->curve(j);
          if ((ssize_t)ofs < 0 || ofs+3 >= mesh->numVertices())
            continue;

	  Vec3fa p0 = mesh->vertex(ofs+0,0);
	  Vec3fa p1 = mesh->vertex(ofs+1,0);
	  Vec3fa p2 = mesh->vertex(ofs+2,0);
	  Vec3fa p3 = mesh->vertex(ofs+3,0);
          if (!isvalid((vfloat4)p0) || !isvalid((vfloat4)p1) || !isvalid((vfloat4)p2) || !isvalid((vfloat4)p3))
              continue;

	  const BezierPrim bezier(mesh->subtype,p0,p1,p2,p3,mesh->tessellationRate,mesh->id,unsigned(j));
          const BBox3fa bounds = bezier.bounds();
          pinfo.add(bounds);
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
            const size_t ofs = mesh->curve(j);
            if ((ssize_t)ofs < 0 || ofs+3 >= mesh->numVertices())
              continue;

            Vec3fa p0 = mesh->vertex(ofs+0,0);
            Vec3fa p1 = mesh->vertex(ofs+1,0);
            Vec3fa p2 = mesh->vertex(ofs+2,0);
            Vec3fa p3 = mesh->vertex(ofs+3,0);
            if (!isvalid((vfloat4)p0) || !isvalid((vfloat4)p1) || !isvalid((vfloat4)p2) || !isvalid((vfloat4)p3))
              continue;
            
            const BezierPrim bezier(mesh->subtype,p0,p1,p2,p3,mesh->tessellationRate,mesh->id,unsigned(j));
            const BBox3fa bounds = bezier.bounds();
            pinfo.add(bounds);
            prims[k++] = bezier;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    PrimInfo createBezierRefArrayMBlur(size_t timeSegment, size_t numTimeSteps, Scene* scene, mvector<BezierPrim>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<BezierCurves,true> iter(scene);

      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](BezierCurves* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          Vec3fa c0,c1,c2,c3;
          if (!mesh->buildPrim(j,timeSegment,numTimeSteps,c0,c1,c2,c3)) continue;
          const BezierPrim bezier(mesh->subtype,c0,c1,c2,c3,mesh->tessellationRate,mesh->id,unsigned(j));
          pinfo.add(bezier.bounds());
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
            Vec3fa c0,c1,c2,c3;
            if (!mesh->buildPrim(j,timeSegment,numTimeSteps,c0,c1,c2,c3)) continue;
            const BezierPrim bezier(mesh->subtype,c0,c1,c2,c3,mesh->tessellationRate,mesh->id,unsigned(j));
            pinfo.add(bezier.bounds());
            prims[k++] = bezier;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    IF_ENABLED_TRIS (template PrimInfo createPrimRefArray<TriangleMesh>(TriangleMesh* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArray<QuadMesh>(QuadMesh* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_HAIR (template PrimInfo createPrimRefArray<BezierCurves>(BezierCurves* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArray<LineSegments>(LineSegments* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER(template PrimInfo createPrimRefArray<AccelSet>(AccelSet* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));

    IF_ENABLED_TRIS (template PrimInfo createPrimRefArray<TriangleMesh COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_TRIS (template PrimInfo createPrimRefArray<TriangleMesh COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArray<QuadMesh COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArray<QuadMesh COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_HAIR (template PrimInfo createPrimRefArray<BezierCurves COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArray<LineSegments COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArray<LineSegments COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER(template PrimInfo createPrimRefArray<AccelSet COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER(template PrimInfo createPrimRefArray<AccelSet COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));

    IF_ENABLED_TRIS (template PrimInfo createPrimRefArrayMBlur<TriangleMesh>(size_t timeSegment COMMA size_t numTimeSteps COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArrayMBlur<QuadMesh>(size_t timeSegment COMMA size_t numTimeSteps COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArrayMBlur<LineSegments>(size_t timeSegment COMMA size_t numTimeSteps COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER(template PrimInfo createPrimRefArrayMBlur<AccelSet>(size_t timeSegment COMMA size_t numTimeSteps COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
  }
}

