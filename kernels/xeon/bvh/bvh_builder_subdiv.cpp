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

#include "bvh.h"
#include "bvh_refit.h"
#include "bvh_builder.h"

#include "../builders/primrefgen.h"
#include "../builders/bvh_builder_sah.h"

#include "../../algorithms/parallel_for_for.h"
#include "../../algorithms/parallel_for_for_prefix_sum.h"

#include "../../common/subdiv/bezier_curve.h"
#include "../geometry/subdivpatch1cached_intersector1.h"

#include "../geometry/subdivpatch1cached.h"
#include "../geometry/grid_aos.h"
#include "../geometry/grid_soa.h"

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    template<int N>
    struct BVHNSubdivGridEagerBuilderBinnedSAHClass : public Builder
    {
      ALIGNED_STRUCT;

      typedef BVHN<N> BVH;

      BVH* bvh;
      Scene* scene;
      mvector<PrimRef> prims;
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      BVHNSubdivGridEagerBuilderBinnedSAHClass (BVH* bvh, Scene* scene)
        : bvh(bvh), scene(scene), prims(scene->device) {}

      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        const size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          parallel_for(size_t(0),iter.size(),[&](const range<size_t>& range) {
              for (size_t i=range.begin(); i<range.end(); i++)
                if (iter[i]) iter[i]->initializeHalfEdgeStructures();
            });
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }
        bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "SubdivGridEagerBuilderBinnedSAH");

        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        /* initialize allocator and parallel_for_for_prefix_sum */
        Scene::Iterator<SubdivMesh> iter(scene);
        pstate.init(iter,size_t(1024));

        PrimInfo pinfo1 = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        { 
          size_t p = 0;
          size_t g = 0;
          for (size_t f=r.begin(); f!=r.end(); ++f) {          
            if (!mesh->valid(f)) continue;
            patch_eval_subdivision(mesh->getHalfEdge(f),[&](const Vec2f uv[4], const int subdiv[4], const float edge_level[4], int subPatch)
            {
              float level[4]; SubdivPatch1Base::computeEdgeLevels(edge_level,subdiv,level);
              Vec2i grid = SubdivPatch1Base::computeGridSize(level);
              size_t num = GridAOS::getNumEagerLeaves(grid.x-1,grid.y-1);
              g+=num;
              p++;
            });
          }
          return PrimInfo(p,g,empty,empty);
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.begin+b.begin,a.end+b.end,empty,empty); });
        size_t numSubPatches = pinfo1.begin;
        if (numSubPatches == 0) {
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }

        prims.resize(pinfo1.end);
        if (pinfo1.end == 0) {
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }

        PrimInfo pinfo3 = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          FastAllocator::ThreadLocal& alloc = *bvh->alloc.threadLocal();
          
          PrimInfo s(empty);
          for (size_t f=r.begin(); f!=r.end(); ++f) {
            if (!mesh->valid(f)) continue;
            
            patch_eval_subdivision(mesh->getHalfEdge(f),[&](const Vec2f uv[4], const int subdiv[4], const float edge_level[4], int subPatch)
            {
              SubdivPatch1Base patch(mesh->id,f,subPatch,mesh,uv,edge_level,subdiv,VSIZEX);
              size_t num = GridAOS::createEager<N>(patch,scene,mesh,f,alloc,&prims[base.end+s.end]);
              assert(num == GridAOS::getNumEagerLeaves(patch.grid_u_res-1,patch.grid_v_res-1));
              for (size_t i=0; i<num; i++)
                s.add(prims[base.end+s.end].bounds());
              s.begin++;
            });
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a, b); });

        PrimInfo pinfo(pinfo3.end,pinfo3.geomBounds,pinfo3.centBounds);
        
        auto createLeaf =  [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> int {
          assert(current.pinfo.size() == 1);
          *current.parent = (size_t) prims[current.prims.begin()].ID();
          return 0;
        };
       
        BVHNBuilder<N>::build(bvh,createLeaf,virtualprogress,prims.data(),pinfo,N,1,1,1.0f,1.0f);
        
	/* clear temporary data for static geometry */
	if (scene->isStatic()) {
          prims.clear();
          bvh->shrink();
        }
        bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    template<int N>
    struct BVHNSubdivPatch1CachedBuilderBinnedSAHClass : public Builder, public BVHNRefitter<N>::LeafBoundsInterface
    {
      ALIGNED_STRUCT;

      typedef BVHN<N> BVH;

      BVH* bvh;
      BVHNRefitter<N>* refitter;
      Scene* scene;
      mvector<PrimRef> prims; 
      mvector<BBox3fa> bounds; 
      ParallelForForPrefixSumState<PrimInfo> pstate;
      size_t numSubdivEnableDisableEvents;

      BVHNSubdivPatch1CachedBuilderBinnedSAHClass (BVH* bvh, Scene* scene)
        : bvh(bvh), refitter(nullptr), scene(scene), prims(scene->device), bounds(scene->device), numSubdivEnableDisableEvents(0) {}
      
      virtual const BBox3fa leafBounds (typename BVH::NodeRef& ref) const
      {
        if (ref == BVH::emptyNode) return BBox3fa(empty);
        size_t num; SubdivPatch1Cached *sptr = (SubdivPatch1Cached*)ref.leaf(num);
        const size_t index = ((size_t)sptr - (size_t)this->bvh->data_mem) / sizeof(SubdivPatch1Cached);
        return prims[index].bounds(); 
      }

      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        bool fastUpdateMode = true;
        size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) 
        {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          fastUpdateMode = parallel_reduce(size_t(0),iter.size(),true,[&](const range<size_t>& range)
          {
            bool fastUpdate = true;
            for (size_t i=range.begin(); i<range.end(); i++)
            {
              if (!iter[i]) continue;
              fastUpdate &= !iter[i]->vertexIndices.isModified(); 
              fastUpdate &= !iter[i]->faceVertices.isModified();
              fastUpdate &= !iter[i]->holes.isModified();
              fastUpdate &= !iter[i]->edge_creases.isModified();
              fastUpdate &= !iter[i]->edge_crease_weights.isModified();
              fastUpdate &= !iter[i]->vertex_creases.isModified();
              fastUpdate &= !iter[i]->vertex_crease_weights.isModified(); 
              fastUpdate &= iter[i]->levels.isModified();
              iter[i]->initializeHalfEdgeStructures();
              iter[i]->patch_eval_trees.resize(iter[i]->size());
            }
            return fastUpdate;
          }, [](const bool a, const bool b) { return a && b; });
        }

        /* only enable fast mode of no subdiv mesh got enabled or disabled since last run */
        fastUpdateMode &= numSubdivEnableDisableEvents == scene->numSubdivEnableDisableEvents;
        numSubdivEnableDisableEvents = scene->numSubdivEnableDisableEvents;

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bounds.resize(numPrimitives);
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }

        /* only invalidate old grids and BVH if we have to recalculate */
        if (!fastUpdateMode)
          bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "SubdivPatch1CachedBuilderBinnedSAH");

        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        /* initialize allocator and parallel_for_for_prefix_sum */
        Scene::Iterator<SubdivMesh> iter(scene);
        pstate.init(iter,size_t(1024));
        PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        { 
          size_t s = 0;
          for (size_t f=r.begin(); f!=r.end(); ++f) 
          {          
            if (!mesh->valid(f)) continue;
            s += patch_eval_subdivision_count (mesh->getHalfEdge(f)); 
            if (unlikely(!fastUpdateMode)) {
              auto alloc = [&] (size_t bytes) { return bvh->alloc.threadLocal()->malloc(bytes); };
              mesh->patch_eval_trees[f] = Patch3fa::create(alloc, mesh->getHalfEdge(f), mesh->getVertexBuffer().getPtr(), mesh->getVertexBuffer().getStride());
            }
          }
          return PrimInfo(s,empty,empty);
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });
        numPrimitives = pinfo.size();
        
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bounds.resize(numPrimitives);
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }
        
        prims.resize(numPrimitives);
        bounds.resize(numPrimitives);
        
        /* Allocate memory for gregory and b-spline patches */
        if (this->bvh->size_data_mem < sizeof(SubdivPatch1Cached) * numPrimitives) 
        {
          if (this->bvh->data_mem) os_free( this->bvh->data_mem, this->bvh->size_data_mem );
          this->bvh->data_mem      = nullptr;
          this->bvh->size_data_mem = 0;
        }
        
        if (bvh->data_mem == nullptr)
        {
          this->bvh->size_data_mem = sizeof(SubdivPatch1Cached) * numPrimitives;
          if ( this->bvh->size_data_mem != 0) this->bvh->data_mem = os_malloc( this->bvh->size_data_mem );        
          else                                this->bvh->data_mem = nullptr;
        }
        assert(this->bvh->data_mem);
        SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;
        
        //atomic_t numChanged = 0;
        //atomic_t numUnchanged = 0;
        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          PrimInfo s(empty);
          for (size_t f=r.begin(); f!=r.end(); ++f) 
          {
            if (!mesh->valid(f)) continue;
            
            patch_eval_subdivision(mesh->getHalfEdge(f),[&](const Vec2f uv[4], const int subdiv[4], const float edge_level[4], int subPatch)
            {
              const unsigned int patchIndex = base.size()+s.size();
              assert(patchIndex < numPrimitives);
              SubdivPatch1Base& patch = subdiv_patches[patchIndex];
              BBox3fa bound = empty;
              
              if (likely(fastUpdateMode)) {
                bool grid_changed = patch.updateEdgeLevels(edge_level,subdiv,mesh,VSIZEX);
                //grid_changed = true;
                //if (grid_changed) atomic_add(&numChanged,1); else atomic_add(&numUnchanged,1);
                if (grid_changed) {
                  patch.resetRootRef();
                  bound = evalGridBounds(patch,0,patch.grid_u_res-1,0,patch.grid_v_res-1,patch.grid_u_res,patch.grid_v_res,mesh);
                }
                else {
                  bound = bounds[patchIndex];
                }
              }
              else {
                new (&patch) SubdivPatch1Cached(mesh->id,f,subPatch,mesh,uv,edge_level,subdiv,VSIZEX);
                bound = evalGridBounds(patch,0,patch.grid_u_res-1,0,patch.grid_v_res-1,patch.grid_u_res,patch.grid_v_res,mesh);
                //patch.root_ref.data = (int64_t) GridSOA::create(&patch,scene,[&](size_t bytes) { return (*bvh->alloc.threadLocal())(bytes); });
              }
              bounds[patchIndex] = bound;
              prims[patchIndex] = PrimRef(bound,patchIndex);
              s.add(bound);
            });
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a, b); });
        //PRINT(numChanged);
        //PRINT(numUnchanged);

        if (fastUpdateMode)
        {
          if (refitter == nullptr)
            refitter = new BVHNRefitter<N>(bvh,*(typename BVHNRefitter<N>::LeafBoundsInterface*)this);

          refitter->refit();
        }
        else
        {
          auto createLeaf = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> int {
            size_t items = current.pinfo.size();
            assert(items == 1);
            const unsigned int patchIndex = prims[current.prims.begin()].ID();
            SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;
            *current.parent = bvh->encodeLeaf((char*)&subdiv_patches[patchIndex],1);
            return 0;
          };
          
          BVHNBuilder<N>::build(bvh,createLeaf,virtualprogress,prims.data(),pinfo,N,1,1,1.0f,1.0f);

          delete refitter; refitter = nullptr;
        }
        
	/* clear temporary data for static geometry */
	if (scene->isStatic()) {
          prims.clear();
          bvh->shrink();
        }
        bvh->cleanup();
        bvh->postBuild(t0);
      }
      
      void clear() {
        prims.clear();
      }
    };
    
    /* entry functions for the scene builder */
    Builder* BVH4SubdivGridEagerBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVHNSubdivGridEagerBuilderBinnedSAHClass<4>((BVH4*)bvh,scene); }
    Builder* BVH4SubdivPatch1CachedBuilderBinnedSAH(void* bvh, Scene* scene, size_t mode) { return new BVHNSubdivPatch1CachedBuilderBinnedSAHClass<4>((BVH4*)bvh,scene); }

#if defined(__AVX__)
    Builder* BVH8SubdivGridEagerBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVHNSubdivGridEagerBuilderBinnedSAHClass<8>((BVH8*)bvh,scene); }
#endif
  }
}
