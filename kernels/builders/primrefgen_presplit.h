
// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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
#include "bvh_builder_morton.h"
#include "../builders/splitter.h"

#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

#define GRID_SIZE 1024

//TODO: Don't sort full array, keep track of min priority and find index

#define DBG_PRESPLIT(x) 

namespace embree
{  
  namespace isa
  {

    struct PresplitItem
    {
      float priority;    
      unsigned int index;

      __forceinline operator unsigned() const
      {
	return reinterpret_cast<const unsigned&>(priority);
      }
      __forceinline bool operator < (const PresplitItem& item) const
      {
	return (priority < item.priority);
      }

      template<typename Mesh>
      static float compute_priority(const PrimRef &ref, Scene *scene)
      {
	const unsigned int geomID = ref.geomID();
	const unsigned int primID = ref.primID();
	const float area_aabb  = halfArea(ref.bounds());
	const float priority = area_aabb;		
	const float area_prim  = ((Mesh*)scene->get(geomID))->projectedPrimitiveArea(primID);
	const float area_ratio = min(4.0f, area_aabb / max(1E-12f,area_prim));
	return priority * area_ratio;      
      }
    
    };

    inline std::ostream &operator<<(std::ostream &cout, const PresplitItem& item) {
      return cout << "index " << item.index << " priority " << item.priority;    
    };

    struct ProbStats
    {
      float p_min, p_max, p_sum;
      size_t p_nonzero;

      __forceinline ProbStats() {}

      __forceinline ProbStats( const float p_min, const float p_max, const float p_sum, const size_t p_nonzero) : p_min(p_min), p_max(p_max), p_sum(p_sum), p_nonzero(p_nonzero) {}
      __forceinline ProbStats( EmptyTy ) : p_min(pos_inf), p_max(neg_inf), p_sum(0.0f), p_nonzero(0) {}

      __forceinline void extend(const float f) {
	if (f > 0.0f)
	  {
	    p_min = min(p_min,f);
	    p_max = max(p_max,f);
	    p_sum += f;
	    p_nonzero++;
	  }
      }
      
      /*! merges two boxes */
      __forceinline static const ProbStats merge (const ProbStats& a, const ProbStats& b) {
	return ProbStats(min(a.p_min,b.p_min),max(a.p_max,b.p_max),a.p_sum+b.p_sum,a.p_nonzero+b.p_nonzero);
      }      
    };

    inline std::ostream &operator<<(std::ostream &cout, const ProbStats& p) {
      return cout << "p_min " << p.p_min << " p_max " << p.p_max << " p_sum " << p.p_sum << " p_nonzero " << p.p_nonzero;
    };

    
    
    template<typename Mesh, typename SplitterFactory>    
      PrimInfo createPrimRefArray_presplit(Geometry* geometry, size_t numPrimRefs, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelPrefixSumState<PrimInfo> pstate;
      
      /* first try */
      progressMonitor(0);
      PrimInfo pinfo = parallel_prefix_sum( pstate, size_t(0), geometry->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo {
	  return geometry->createPrimRefArray(prims,r,r.begin());
	}, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

      /* if we need to filter out geometry, run again */
      if (pinfo.size() != numPrimRefs)
	{
	  progressMonitor(0);
	  pinfo = parallel_prefix_sum( pstate, size_t(0), geometry->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo {
	      return geometry->createPrimRefArray(prims,r,base.size());
	    }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
	}
      return pinfo;	
    }
    
    __forceinline Vec2i computeMC(const Vec3fa &grid_base, const float grid_scale, const PrimRef &ref)
    {
      const Vec3fa lower = ref.lower;
      const Vec3fa upper = ref.upper;
      const Vec3fa glower = (lower-grid_base)*Vec3fa(grid_scale)+Vec3fa(0.2f);
      const Vec3fa gupper = (upper-grid_base)*Vec3fa(grid_scale)-Vec3fa(0.2f);
      Vec3ia ilower(floor(glower));
      Vec3ia iupper(floor(gupper));
      
      /* this ignores dimensions that are empty */
      if (glower.x >= gupper.x) iupper.x = ilower.x;
      if (glower.y >= gupper.y) iupper.y = ilower.y;
      if (glower.z >= gupper.z) iupper.z = ilower.z;
		
      /* compute a morton code for the lower and upper grid coordinates. */
      const unsigned int lower_code = bitInterleave(ilower.x,ilower.y,ilower.z);
      const unsigned int upper_code = bitInterleave(iupper.x,iupper.y,iupper.z);
      return Vec2i(lower_code,upper_code);
    }

    template<typename Mesh, typename SplitterFactory>    
      PrimInfo createPrimRefArray_presplit(Scene* scene, Geometry::GTypeMask types, bool mblur, size_t numPrimRefs, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {	
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator2 iter(scene,types,mblur);
      
      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum0( pstate, iter, PrimInfo(empty), [&](Geometry* mesh, const range<size_t>& r, size_t k) -> PrimInfo {
	  return mesh->createPrimRefArray(prims,r,k);
	}, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      
      /* if we need to filter out geometry, run again */
      if (pinfo.size() != numPrimRefs)
	{
	  progressMonitor(0);
	  pinfo = parallel_for_for_prefix_sum1( pstate, iter, PrimInfo(empty), [&](Geometry* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo {
	      return mesh->createPrimRefArray(prims,r,base.size());
	    }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
	}

      /* use correct number of primitives */
      size_t numPrimitives = pinfo.size();
      const size_t alloc_numPrimitives = prims.size(); 

      /* set up primitive splitter */
      SplitterFactory Splitter(scene);

      const size_t org_numPrimitivesToSplit = alloc_numPrimitives - numPrimitives;
      size_t numPrimitivesToSplit = org_numPrimitivesToSplit;
		
      DBG_PRESPLIT(PRINT(numPrimitivesToSplit));

      /* allocate double buffer presplit items */
      PresplitItem *presplitItem     = (PresplitItem*)alignedMalloc(sizeof(PresplitItem)*alloc_numPrimitives,64);
      PresplitItem *tmp_presplitItem = (PresplitItem*)alignedMalloc(sizeof(PresplitItem)*alloc_numPrimitives,64);

      /* compute grid */
      const Vec3fa grid_base    = pinfo.geomBounds.lower;
      const Vec3fa grid_diag    = pinfo.geomBounds.size();
      const float grid_extend   = max(grid_diag.x,max(grid_diag.y,grid_diag.z));		
      const float grid_scale    = grid_extend == 0.0f ? 0.0f : GRID_SIZE / grid_extend;
      const float inv_grid_size = 1.0f / GRID_SIZE;

      /* init presplit items */
      parallel_for( size_t(0), numPrimitives, size_t(1024), [&](const range<size_t>& r) -> void {
	  for (size_t i=r.begin(); i<r.end(); i++)
	    {		
	      presplitItem[i].index = (unsigned int)i;
              const Vec2i mc = computeMC(grid_base,grid_scale,prims[i]);
              /* if all bits are equal then we cannot split */
              if (mc.x != mc.y)
                presplitItem[i].priority = PresplitItem::compute_priority<Mesh>(prims[i],scene);
              else
                presplitItem[i].priority = 0.0f;
	    }
	});
      
      while(numPrimitivesToSplit)
	{
	  /* sort presplit items in ascending order */
	  radix_sort_u32(presplitItem,tmp_presplitItem,numPrimitives,1024);

	  DBG_PRESPLIT(
		       for (size_t i=1;i<numPrimitives;i++)
			 assert(presplitItem[i-1].priority <= presplitItem[i].priority);
		       );

#if 1	  
	  /* get sum of split priorities */
	  const ProbStats pstats = parallel_reduce(size_t(0),numPrimitives,ProbStats(empty), [&] (const range<size_t>& r) -> ProbStats {
	      ProbStats current(empty);
	      for (size_t i=r.begin(); i<r.end(); i++)
		current.extend(presplitItem[i].priority);			  
	      return current;
	    },[](const ProbStats& a, const ProbStats& b) -> ProbStats { return ProbStats::merge(a,b); });		    
	  const float priority_avg = pstats.p_sum / pstats.p_nonzero;
          DBG_PRESPLIT(PRINT( priority_avg ));

	  /* binary search to find index with priority >= priority_avg */
	  size_t l = 0;
	  size_t r = numPrimitives;		    
	  while(l+1 < r)
	    {
	      const size_t mid = (l+r)/2;
	      if (presplitItem[mid].priority < priority_avg) 
		l = mid;
	      else
		r = mid;
	    }
	  const size_t numPrimitivesToSplitStep = min(numPrimitives-r,numPrimitivesToSplit);	
#else
          /* binary search for finding start of elements with priority > 0.0f */
	  size_t l = 0;
	  size_t r = numPrimitives;		    
	  while(l+1 < r)
	    {
	      const size_t mid = (l+r)/2;
	      if (presplitItem[mid].priority == 0.0f) 
		l = mid;
	      else
		r = mid;
	    }
	  const size_t start_priorities = r;

          static const float iteration_factor = 0.25f;
	  const size_t numPrimitivesToSplitStep = min(numPrimitives-start_priorities,min((size_t)ceilf(iteration_factor*org_numPrimitivesToSplit),numPrimitivesToSplit));	
#endif

          DBG_PRESPLIT(PRINT( numPrimitivesToSplitStep ));
		    
	  __aligned(64) std::atomic<size_t> offset;
	  offset.store(0);
		    
	  parallel_for( size_t(0), numPrimitivesToSplitStep, size_t(128), [&](const range<size_t>& r) -> void {
	      for (size_t j=r.begin(); j<r.end(); j++)		    
		{
		  const size_t i = numPrimitives - 1 - j;
		  assert(presplitItem[i].priority > 0.0f);
		  const unsigned int  primrefID = presplitItem[i].index;		
		  const unsigned int   geomID   = prims[primrefID].geomID();
		  const unsigned int   primID   = prims[primrefID].primID();
		  
		  const Vec3fa lower = prims[primrefID].lower;
		  const Vec3fa upper = prims[primrefID].upper;
		  const Vec3fa glower = (lower-grid_base)*Vec3fa(grid_scale)+Vec3fa(0.2f);
		  const Vec3fa gupper = (upper-grid_base)*Vec3fa(grid_scale)-Vec3fa(0.2f);
		  Vec3ia ilower(floor(glower));
		  Vec3ia iupper(floor(gupper));
      
		  /* this ignores dimensions that are empty */
		  if (glower.x >= gupper.x) iupper.x = ilower.x;
		  if (glower.y >= gupper.y) iupper.y = ilower.y;
		  if (glower.z >= gupper.z) iupper.z = ilower.z;
		
		  /* compute a morton code for the lower and upper grid coordinates. */
		  const unsigned int lower_code = bitInterleave(ilower.x,ilower.y,ilower.z);
		  const unsigned int upper_code = bitInterleave(iupper.x,iupper.y,iupper.z);
			
		  /* if all bits are equal then we cannot split */
		  assert (lower_code != upper_code);
                  const unsigned int diff = 31 - lzcnt(lower_code^upper_code);
		    
                  /* compute octree level and dimension to perform the split in */
                  const unsigned int level = diff / 3;
                  const unsigned int dim   = diff % 3;
      
                  /* now we compute the grid position of the split */
                  const unsigned int isplit = iupper[dim] & ~((1<<level)-1);
			    
                  /* compute world space position of split */
                  const float fsplit = grid_base[dim] + isplit * inv_grid_size * grid_extend;

                  assert(prims[primrefID].bounds().lower[dim] <= fsplit &&
                         prims[primrefID].bounds().upper[dim] >= fsplit);
		
                  /* split primitive */
                  const auto splitter = Splitter(prims[primrefID]);
                  BBox3fa left,right;
                  splitter(prims[primrefID].bounds(),dim,fsplit,left,right);
			    
                  const size_t newID = numPrimitives + offset.fetch_add(1);
                  assert(newID < numPrimitives + numPrimitivesToSplitStep);
                  prims[primrefID] = PrimRef(left,geomID,primID);
                  prims[newID    ] = PrimRef(right,geomID,primID);

                  /* update presplititems */
                  presplitItem[i].index = primrefID;
                  presplitItem[i].priority = PresplitItem::compute_priority<Mesh>(prims[primrefID],scene);
                  presplitItem[newID].index = (unsigned int)newID;
                  presplitItem[newID].priority = PresplitItem::compute_priority<Mesh>(prims[newID],scene);

                  /* set priority to zero if we cannot split the new primitive */
                  const Vec2i mc0 = computeMC(grid_base,grid_scale,prims[primrefID]);
                  const Vec2i mc1 = computeMC(grid_base,grid_scale,prims[newID]);
                  if (mc0.x == mc0.y) presplitItem[i    ].priority = 0.0f;
                  if (mc1.x == mc1.y) presplitItem[newID].priority = 0.0f;
		}
	    });
	  assert(offset <= numPrimitivesToSplit);
		    
	  numPrimitives += offset;
	  numPrimitivesToSplit -= offset;

          DBG_PRESPLIT(
            PRINT(offset);		    
            PRINT(numPrimitives);
            PRINT(numPrimitivesToSplit);
            );
	  if (offset == 0) break;
	}
	
      /* recompute centroid bounding boxes */
      pinfo = parallel_reduce(size_t(0),numPrimitives,PrimInfo(empty), [&] (const range<size_t>& r) -> PrimInfo {
	  PrimInfo p(empty);
	  for (size_t j=r.begin(); j<r.end(); j++)
	    p.add_center2(prims[j]);
	  return p;
	}, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
  
      assert(pinfo.size() == numPrimitives);
      
      /* free double buffer presplit items */
      alignedFree(tmp_presplitItem);		
      alignedFree(presplitItem);
      
      return pinfo;	
    }
  }
}
