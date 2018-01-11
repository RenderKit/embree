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
          const PrimRef prim(bounds,mesh->geomID,unsigned(j));          
          pinfo.add_center2(prim);
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
            const PrimRef prim(bounds,mesh->geomID,unsigned(j));
            pinfo.add_center2(prim);
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    template<typename Mesh>
    PrimInfo createGroupPrimRefArray(GeometryGroup* group, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      /* first try */
      progressMonitor(0);
      pstate.init(*group,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum0( pstate, *group, PrimInfo(empty), [&](Geometry* geom, const range<size_t>& r, size_t k) -> PrimInfo
      {
        Mesh* mesh = dynamic_cast<Mesh*>(geom);
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->buildBounds(j,&bounds)) continue;
          const PrimRef prim(bounds,mesh->geomID,unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_for_for_prefix_sum1( pstate, *group, PrimInfo(empty), [&](Geometry* geom, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          Mesh* mesh = dynamic_cast<Mesh*>(geom);
          k = base.size();
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            BBox3fa bounds = empty;
            if (!mesh->buildBounds(j,&bounds)) continue;
            const PrimRef prim(bounds,mesh->geomID,unsigned(j));
            pinfo.add_center2(prim);
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
      PrimInfo pinfo = parallel_for_for_prefix_sum0( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->buildBounds(j,&bounds)) continue;
          const PrimRef prim(bounds,mesh->geomID,unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_for_for_prefix_sum1( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          k = base.size();
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            BBox3fa bounds = empty;
            if (!mesh->buildBounds(j,&bounds)) continue;
            const PrimRef prim(bounds,mesh->geomID,unsigned(j));
            pinfo.add_center2(prim);
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    template<typename Mesh>
    PrimInfo createPrimRefArrayMBlur(size_t timeSegment, Scene* scene, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<Mesh,true> iter(scene);

      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum0( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->buildBounds(j,timeSegment,bounds)) continue;
          const PrimRef prim(bounds,mesh->geomID,unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_for_for_prefix_sum1( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          k = base.size();
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            BBox3fa bounds = empty;
            if (!mesh->buildBounds(j,timeSegment,bounds)) continue;
            const PrimRef prim(bounds,mesh->geomID,unsigned(j));
            pinfo.add_center2(prim);
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    template<typename Mesh>
    __noinline PrimInfoMB calculatePrimRefMB(mvector<PrimRefMB>& prims, const BBox1f& t0t1, Mesh* mesh, const range<size_t>& r, size_t k)
    {
      PrimInfoMB pinfo(empty);
      for (size_t j=r.begin(); j<r.end(); j++)
      {
        LBBox3fa bounds = empty;
        if (!mesh->linearBounds(j,t0t1,bounds)) continue;
        const PrimRefMB prim(bounds,mesh->numTimeSegments(),mesh->numTimeSegments(),mesh->geomID,unsigned(j));
        pinfo.add_primref(prim);
        prims[k++] = prim;
      }
      return pinfo;
    }
    
    template<typename Mesh>
    PrimInfoMB createPrimRefArrayMSMBlur(Scene* scene, mvector<PrimRefMB>& prims, BuildProgressMonitor& progressMonitor, BBox1f t0t1)
    {
      ParallelForForPrefixSumState<PrimInfoMB> pstate;
      Scene::Iterator<Mesh,true> iter(scene);
      
      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfoMB pinfo = parallel_for_for_prefix_sum0( pstate, iter, PrimInfoMB(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k) -> PrimInfoMB {
          return calculatePrimRefMB(prims,t0t1,mesh,r,k);
      }, [](const PrimInfoMB& a, const PrimInfoMB& b) -> PrimInfoMB { return PrimInfoMB::merge2(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_for_for_prefix_sum1( pstate, iter, PrimInfoMB(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfoMB& base) -> PrimInfoMB {
            return calculatePrimRefMB(prims,t0t1,mesh,r,base.size());
        }, [](const PrimInfoMB& a, const PrimInfoMB& b) -> PrimInfoMB { return PrimInfoMB::merge2(a,b); });
      }
      pinfo.time_range = t0t1;
      return pinfo;
    }

    template<typename Mesh>
    size_t createMortonCodeArray(Mesh* mesh, mvector<BVHBuilderMorton::BuildPrim>& morton, BuildProgressMonitor& progressMonitor)
    {
      size_t numPrimitives = morton.size();

      /* compute scene bounds */
      std::pair<size_t,BBox3fa> cb_empty(0,empty);
      auto cb = parallel_reduce 
        ( size_t(0), numPrimitives, size_t(1024), cb_empty, [&](const range<size_t>& r) -> std::pair<size_t,BBox3fa>
          {
            size_t num = 0;
            BBox3fa bounds = empty;
            
            for (size_t j=r.begin(); j<r.end(); j++)
            {
              BBox3fa prim_bounds = empty;
              if (unlikely(!mesh->buildBounds(j,&prim_bounds))) continue;
              bounds.extend(center2(prim_bounds));
              num++;
            }
            return std::make_pair(num,bounds);
          }, [] (const std::pair<size_t,BBox3fa>& a, const std::pair<size_t,BBox3fa>& b) {
          return std::make_pair(a.first + b.first,merge(a.second,b.second)); 
        });
      
      
      size_t numPrimitivesGen = cb.first;
      const BBox3fa centBounds = cb.second;
      
      /* compute morton codes */
      if (likely(numPrimitivesGen == numPrimitives))
      {
        /* fast path if all primitives were valid */
        BVHBuilderMorton::MortonCodeMapping mapping(centBounds);
        parallel_for( size_t(0), numPrimitives, size_t(1024), [&](const range<size_t>& r) -> void {
            BVHBuilderMorton::MortonCodeGenerator generator(mapping,&morton.data()[r.begin()]);
            for (size_t j=r.begin(); j<r.end(); j++)
              generator(mesh->bounds(j),unsigned(j));
          });
      }
      else
      {
        /* slow path, fallback in case some primitives were invalid */
        ParallelPrefixSumState<size_t> pstate;
        BVHBuilderMorton::MortonCodeMapping mapping(centBounds);
        parallel_prefix_sum( pstate, size_t(0), numPrimitives, size_t(1024), size_t(0), [&](const range<size_t>& r, const size_t base) -> size_t {
            size_t num = 0;
            BVHBuilderMorton::MortonCodeGenerator generator(mapping,&morton.data()[r.begin()]);
            for (size_t j=r.begin(); j<r.end(); j++)
            {
              BBox3fa bounds = empty;
              if (unlikely(!mesh->buildBounds(j,&bounds))) continue;
              generator(bounds,unsigned(j));
              num++;
            }
            return num;
          }, std::plus<size_t>());
        
        parallel_prefix_sum( pstate, size_t(0), numPrimitives, size_t(1024), size_t(0), [&](const range<size_t>& r, const size_t base) -> size_t {
            size_t num = 0;
            BVHBuilderMorton::MortonCodeGenerator generator(mapping,&morton.data()[base]);
            for (size_t j=r.begin(); j<r.end(); j++)
            {
              BBox3fa bounds = empty;
              if (!mesh->buildBounds(j,&bounds)) continue;
              generator(bounds,unsigned(j));
              num++;
            }
            return num;
          }, std::plus<size_t>());          
      }
      return numPrimitivesGen;
    }

    IF_ENABLED_TRIS (template PrimInfo createPrimRefArray<TriangleMesh>(TriangleMesh* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArray<QuadMesh>(QuadMesh* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_HAIR (template PrimInfo createPrimRefArray<NativeCurves>(NativeCurves* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArray<LineSegments>(LineSegments* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER (template PrimInfo createPrimRefArray<AccelSet>(AccelSet* mesh COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    
    IF_ENABLED_TRIS (template PrimInfo createGroupPrimRefArray<TriangleMesh>(GeometryGroup* group COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createGroupPrimRefArray<QuadMesh>(GeometryGroup* group COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_HAIR (template PrimInfo createGroupPrimRefArray<NativeCurves>(GeometryGroup* group COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createGroupPrimRefArray<LineSegments>(GeometryGroup* group COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER (template PrimInfo createGroupPrimRefArray<AccelSet>(GeometryGroup* group COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    
    IF_ENABLED_TRIS (template PrimInfo createPrimRefArray<TriangleMesh COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_TRIS (template PrimInfo createPrimRefArray<TriangleMesh COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArray<QuadMesh COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArray<QuadMesh COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_HAIR (template PrimInfo createPrimRefArray<NativeCurves COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArray<LineSegments COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArray<LineSegments COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER(template PrimInfo createPrimRefArray<AccelSet COMMA false>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER(template PrimInfo createPrimRefArray<AccelSet COMMA true>(Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));

    IF_ENABLED_TRIS (template PrimInfo createPrimRefArrayMBlur<TriangleMesh>(size_t timeSegment COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template PrimInfo createPrimRefArrayMBlur<QuadMesh>(size_t timeSegment COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_LINES(template PrimInfo createPrimRefArrayMBlur<LineSegments>(size_t timeSegment COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER(template PrimInfo createPrimRefArrayMBlur<AccelSet>(size_t timeSegment COMMA Scene* scene COMMA mvector<PrimRef>& prims COMMA BuildProgressMonitor& progressMonitor));

    template PrimInfoMB createPrimRefArrayMSMBlur<TriangleMesh>(Scene* scene, mvector<PrimRefMB>& prims, BuildProgressMonitor& progressMonitor, BBox1f t0t1);
    template PrimInfoMB createPrimRefArrayMSMBlur<QuadMesh>(Scene* scene, mvector<PrimRefMB>& prims, BuildProgressMonitor& progressMonitor, BBox1f t0t1);
    template PrimInfoMB createPrimRefArrayMSMBlur<NativeCurves>(Scene* scene, mvector<PrimRefMB>& prims, BuildProgressMonitor& progressMonitor, BBox1f t0t1);
    template PrimInfoMB createPrimRefArrayMSMBlur<LineSegments>(Scene* scene, mvector<PrimRefMB>& prims, BuildProgressMonitor& progressMonitor, BBox1f t0t1);
    template PrimInfoMB createPrimRefArrayMSMBlur<AccelSet>(Scene* scene, mvector<PrimRefMB>& prims, BuildProgressMonitor& progressMonitor, BBox1f t0t1);

    IF_ENABLED_TRIS (template size_t createMortonCodeArray<TriangleMesh>(TriangleMesh* mesh COMMA mvector<BVHBuilderMorton::BuildPrim>& morton COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_QUADS(template size_t createMortonCodeArray<QuadMesh>(QuadMesh* mesh COMMA mvector<BVHBuilderMorton::BuildPrim>& morton COMMA BuildProgressMonitor& progressMonitor));
    IF_ENABLED_USER (template size_t createMortonCodeArray<AccelSet>(AccelSet* mesh COMMA mvector<BVHBuilderMorton::BuildPrim>& morton COMMA BuildProgressMonitor& progressMonitor));
  }
}
