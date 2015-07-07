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

#include "bvh4.h"

#include "../builders/primrefgen.h"
#include "../builders/bvh_builder_sah.h"

#include "../../algorithms/parallel_for_for.h"
#include "../../algorithms/parallel_for_for_prefix_sum.h"

#include "../../common/subdiv/feature_adaptive_gregory.h"
#include "../../common/subdiv/feature_adaptive_bspline.h"
#include "../../common/subdiv/bezier_curve.h"
#include "../geometry/subdivpatch1cached_intersector1.h"

#include "../geometry/grid.h"
#include "../geometry/subdivpatch1.h"
#include "../geometry/subdivpatch1cached.h"

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    template<typename Primitive>
    struct CreateBVH4SubdivLeaf
    {
      __forceinline CreateBVH4SubdivLeaf (BVH4* bvh, PrimRef* prims) 
        : bvh(bvh), prims(prims) {}
      
      __forceinline int operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return 1;
      }

      BVH4* bvh;
      PrimRef* prims;
    };

    struct BVH4SubdivPatch1BuilderBinnedSAHClass : public Builder
    {
      ALIGNED_STRUCT;

      BVH4* bvh;
      Scene* scene;
      mvector<PrimRef> prims;
      
      BVH4SubdivPatch1BuilderBinnedSAHClass (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        const size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          for (size_t i=0; i<iter.size(); i++) // FIXME: parallelize
            if (iter[i]) iter[i]->initializeHalfEdgeStructures();
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4SubdivPatch1BuilderBinnedSAH");

        prims.resize(numPrimitives);
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);
        const PrimInfo pinfo = createPrimRefArray<SubdivMesh,1>(scene,prims,virtualprogress);
        BVH4::NodeRef root;
        BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
          (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),BVH4::NoRotate(),CreateBVH4SubdivLeaf<SubdivPatch1>(bvh,prims.data()),virtualprogress,
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1,1.0f,1.0f);
        bvh->set(root,pinfo.geomBounds,pinfo.size());

	/* clear temporary data for static geometry */
	bool staticGeom = scene->isStatic();
	if (staticGeom) prims.clear();
        bvh->alloc.cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };
    
    struct BVH4SubdivGridBuilderBinnedSAHClass : public Builder
    {
      ALIGNED_STRUCT;

      BVH4* bvh;
      Scene* scene;
      mvector<PrimRef> prims;
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      BVH4SubdivGridBuilderBinnedSAHClass (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        const size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          for (size_t i=0; i<iter.size(); i++) // FIXME: parallelize
            if (iter[i]) iter[i]->initializeHalfEdgeStructures();
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4SubdivGridBuilderBinnedSAH");

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
            
            feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                                 [&](const CatmullClarkPatch3fa& patch, const int depth, const Vec2f uv[4], const int subdiv[4], const BezierCurve3fa *border, const int border_flags)
	    {
              //if (!patch.isRegular()) { s++; return; }
              const float l0 = patch.ring[0].edge_level;
              const float l1 = patch.ring[1].edge_level;
              const float l2 = patch.ring[2].edge_level;
              const float l3 = patch.ring[3].edge_level;
              const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
              const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
              const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
              const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
              const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
              const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
              s += Grid::getNumEagerLeaves(pattern_x.size(),pattern_y.size());
            });
          }
          return PrimInfo(s,empty,empty);
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });

        prims.resize(pinfo.size());
        if (pinfo.size() == 0) {
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }

        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          FastAllocator::ThreadLocal& alloc = *bvh->alloc.threadLocal();
          
          PrimInfo s(empty);
          for (size_t f=r.begin(); f!=r.end(); ++f) {
            if (!mesh->valid(f)) continue;
            
            feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                                 [&](const CatmullClarkPatch3fa& patch, const int depth, const Vec2f uv[4], const int subdiv[4], const BezierCurve3fa *border, const int border_flags)
            {
              /*if (!patch.isRegular())
                {
                Grid* leaf = (Grid*) bvh->alloc.malloc(sizeof(Grid),16);
                new (leaf) Grid(id,mesh->id,f);3
                const BBox3fa bounds = leaf->quad(scene,patch,uv[0],uv[1],uv[2],uv[3]);
                prims[base.size()+s.size()] = PrimRef(bounds,BVH4::encodeTypedLeaf(leaf,0));
                s.add(bounds);
                return;
                }*/
              
              const float l0 = patch.ring[0].edge_level;
              const float l1 = patch.ring[1].edge_level;
              const float l2 = patch.ring[2].edge_level;
              const float l3 = patch.ring[3].edge_level;
              const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
              const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
              const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
              const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
              const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
              const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
              const int nx = pattern_x.size();
              const int ny = pattern_y.size();
              
              GregoryPatch3fa patcheval; patcheval.init(patch);
              //BSplinePatch patcheval; patcheval.init(patch);
              size_t N = Grid::createEager(mesh->id,f,scene,patcheval,alloc,&prims[base.size()+s.size()],0,nx,0,ny,uv,pattern0,pattern1,pattern2,pattern3,pattern_x,pattern_y);
              assert(N == Grid::getNumEagerLeaves(nx,ny));
              for (size_t i=0; i<N; i++)
                s.add(prims[base.size()+s.size()].bounds());
            });
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
        
        BVH4::NodeRef root;
        BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
          (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),BVH4::NoRotate(),
           [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> int {
            if (current.pinfo.size() != 1) THROW_RUNTIME_ERROR("bvh4_builder_subdiv: internal error");
            *current.parent = (size_t) prims[current.prims.begin()].ID();
            return 0;
          },
           progress,
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1,1.0f,1.0f);
        bvh->set(root,pinfo.geomBounds,pinfo.size());
        
	/* clear temporary data for static geometry */
	bool staticGeom = scene->isStatic();
	if (staticGeom) prims.clear();
        bvh->alloc.cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    struct BVH4SubdivGridEagerBuilderBinnedSAHClass : public Builder
    {
      ALIGNED_STRUCT;

      BVH4* bvh;
      Scene* scene;
      mvector<PrimRef> prims;
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      BVH4SubdivGridEagerBuilderBinnedSAHClass (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        const size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          for (size_t i=0; i<iter.size(); i++) // FIXME: parallelize
            if (iter[i]) iter[i]->initializeHalfEdgeStructures();
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4SubdivGridEagerBuilderBinnedSAH");

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
            
            feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
					    [&](const CatmullClarkPatch3fa& patch, const Vec2f uv[4], const int subdiv[4], const int id)
            {
              const float l0 = patch.ring[0].edge_level;
              const float l1 = patch.ring[1].edge_level;
              const float l2 = patch.ring[2].edge_level;
              const float l3 = patch.ring[3].edge_level;
              const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
              const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
              const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
              const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
              const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
              const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
              s += Grid::getNumEagerLeaves(pattern_x.size(),pattern_y.size());
            });
          }
          return PrimInfo(s,empty,empty);
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });
        
        prims.resize(pinfo.size());
        if (pinfo.size() == 0) {
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }

        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          FastAllocator::ThreadLocal& alloc = *bvh->alloc.threadLocal();
          
          PrimInfo s(empty);
          for (size_t f=r.begin(); f!=r.end(); ++f) {
            if (!mesh->valid(f)) continue;
            
            feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                              [&](const CatmullClarkPatch3fa& patch, const Vec2f uv[4], const int subdiv[4], const int id)
            {
              const float l0 = patch.ring[0].edge_level;
              const float l1 = patch.ring[1].edge_level;
              const float l2 = patch.ring[2].edge_level;
              const float l3 = patch.ring[3].edge_level;
              const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
              const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
              const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
              const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
              const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
              const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
              const int nx = pattern_x.size();
              const int ny = pattern_y.size();
              size_t N = Grid::createEager(mesh->id,f,scene,patch,alloc,&prims[base.size()+s.size()],0,nx,0,ny,uv,pattern0,pattern1,pattern2,pattern3,pattern_x,pattern_y);
              assert(N == Grid::getNumEagerLeaves(nx,ny));
              for (size_t i=0; i<N; i++)
                s.add(prims[base.size()+s.size()].bounds());
            });
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
        
        BVH4::NodeRef root;
        BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
          (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),BVH4::NoRotate(),
           [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> int {
             if (current.pinfo.size() != 1) THROW_RUNTIME_ERROR("bvh4_builder_subdiv: internal error");
             *current.parent = (size_t) prims[current.prims.begin()].ID();
             return 0;
           },
           progress,
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1,1.0f,1.0f);
        bvh->set(root,pinfo.geomBounds,pinfo.size());
        
	/* clear temporary data for static geometry */
	bool staticGeom = scene->isStatic();
	if (staticGeom) prims.clear();
        bvh->alloc.cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    struct BVH4SubdivGridLazyBuilderBinnedSAHClass : public Builder
    {
      ALIGNED_STRUCT;

      BVH4* bvh;
      Scene* scene;
      mvector<PrimRef> prims; 
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      BVH4SubdivGridLazyBuilderBinnedSAHClass (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        const size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          for (size_t i=0; i<iter.size(); i++) // FIXME: parallelize
            if (iter[i]) iter[i]->initializeHalfEdgeStructures();
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4SubdivGridLazyBuilderBinnedSAH");

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
            
            feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                              [&](const CatmullClarkPatch3fa& patch, const Vec2f uv[4], const int subdiv[4], const int id)
	    {
              const float l0 = patch.ring[0].edge_level;
              const float l1 = patch.ring[1].edge_level;
              const float l2 = patch.ring[2].edge_level;
              const float l3 = patch.ring[3].edge_level;
              const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
              const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
              const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
              const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
              const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
              const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
              s += Grid::getNumLazyLeaves(pattern_x.size(),pattern_y.size());
            });
          }
          return PrimInfo(s,empty,empty);
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });
        
        prims.resize(pinfo.size());
        if (pinfo.size() == 0) {
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }

        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          FastAllocator::ThreadLocal& alloc = *bvh->alloc.threadLocal();
          
          PrimInfo s(empty);
          for (size_t f=r.begin(); f!=r.end(); ++f) {
            if (!mesh->valid(f)) continue;
            
            feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                              [&](const CatmullClarkPatch3fa& patch, const Vec2f uv[4], const int subdiv[4], const int id)
	    {
              const float l0 = patch.ring[0].edge_level;
              const float l1 = patch.ring[1].edge_level;
              const float l2 = patch.ring[2].edge_level;
              const float l3 = patch.ring[3].edge_level;
              const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
              const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
              const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
              const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
              const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
              const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
              const int nx = pattern_x.size();
              const int ny = pattern_y.size();
              size_t N = Grid::createLazy(bvh,nullptr,mesh->id,f,id,nx,ny,alloc,&prims[base.size()+s.size()]);
              assert(N == Grid::getNumLazyLeaves(nx,ny));
              for (size_t i=0; i<N; i++)
                s.add(prims[base.size()+s.size()].bounds());
            });
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
        
        BVH4::NodeRef root;
        BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
          (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),BVH4::NoRotate(),
           [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> int {
            assert(current.pinfo.size() == 1);
            BVH4::NodeRef node = prims[current.prims.begin()].ID();
            size_t ty = 0;
            Grid::LazyLeaf* leaf = (Grid::LazyLeaf*) node.leaf(ty);
            leaf->parent = (BVH4::NodeRef*) current.parent;
            *current.parent = node;
            assert(ty == 1);
            return 0;
          },
          progress,
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1,1.0f,1.0f);
        bvh->set(root,pinfo.geomBounds,pinfo.size());
        
	/* clear temporary data for static geometry */
	bool staticGeom = scene->isStatic();
	if (staticGeom) prims.clear();
        bvh->alloc.cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

#define DBG_CACHE_BUILDER(x) 

    BBox3fa getBounds1(const SubdivPatch1Base &patch, const SubdivMesh* const mesh)
    {
#if FORCE_TESSELLATION_BOUNDS == 1

#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
      __aligned(64) float grid_x[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_y[(patch.grid_size_simd_blocks+1)*8];
      __aligned(64) float grid_z[(patch.grid_size_simd_blocks+1)*8];         
      __aligned(64) float grid_u[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_v[(patch.grid_size_simd_blocks+1)*8];

#else
      const size_t array_elements = (patch.grid_size_simd_blocks + 1) * 8;
      float *const ptr = (float*)_malloca(5 * array_elements * sizeof(float) + 64);
      float *const grid_arrays = (float*)ALIGN_PTR(ptr,64);

      float *grid_x = &grid_arrays[array_elements * 0];
      float *grid_y = &grid_arrays[array_elements * 1];
      float *grid_z = &grid_arrays[array_elements * 2];
      float *grid_u = &grid_arrays[array_elements * 3];
      float *grid_v = &grid_arrays[array_elements * 4];
#endif
      //PRINT2(patch.grid_u_res,patch.grid_v_res);
      evalGrid(patch,grid_x,grid_y,grid_z,grid_u,grid_v,mesh);
      
      /*PING;
      for (size_t y=0; y<patch.grid_v_res; y++) {
        for (size_t x=0; x<patch.grid_u_res; x++) {
        Vec3f v(grid_x[y*patch.grid_u_res+x],grid_y[y*patch.grid_u_res+x],grid_z[y*patch.grid_u_res+x]);
          PRINT3(x,y,v);
        }
        }*/

      BBox3fa b(empty);
      assert(patch.grid_size_simd_blocks >= 1);

#if !defined(__AVX__)      
      float4 bounds_min_x = pos_inf;
      float4 bounds_min_y = pos_inf;
      float4 bounds_min_z = pos_inf;
      float4 bounds_max_x = neg_inf;
      float4 bounds_max_y = neg_inf;
      float4 bounds_max_z = neg_inf;
      for (size_t i = 0; i<patch.grid_size_simd_blocks * 2; i++)
        {
          float4 x = load4f(&grid_x[i * 4]);
          float4 y = load4f(&grid_y[i * 4]);
          float4 z = load4f(&grid_z[i * 4]);
	  bounds_min_x = min(bounds_min_x,x);
	  bounds_min_y = min(bounds_min_y,y);
	  bounds_min_z = min(bounds_min_z,z);

	  bounds_max_x = max(bounds_max_x,x);
	  bounds_max_y = max(bounds_max_y,y);
	  bounds_max_z = max(bounds_max_z,z);
        }

      b.lower.x = reduce_min(bounds_min_x);
      b.lower.y = reduce_min(bounds_min_y);
      b.lower.z = reduce_min(bounds_min_z);
      b.upper.x = reduce_max(bounds_max_x);
      b.upper.y = reduce_max(bounds_max_y);
      b.upper.z = reduce_max(bounds_max_z);
#else
      float8 bounds_min_x = pos_inf;
      float8 bounds_min_y = pos_inf;
      float8 bounds_min_z = pos_inf;
      float8 bounds_max_x = neg_inf;
      float8 bounds_max_y = neg_inf;
      float8 bounds_max_z = neg_inf;
      for (size_t i = 0; i<patch.grid_size_simd_blocks; i++)
        {
          float8 x = load8f(&grid_x[i * 8]);
          float8 y = load8f(&grid_y[i * 8]);
          float8 z = load8f(&grid_z[i * 8]);
	  bounds_min_x = min(bounds_min_x,x);
	  bounds_min_y = min(bounds_min_y,y);
	  bounds_min_z = min(bounds_min_z,z);

	  bounds_max_x = max(bounds_max_x,x);
	  bounds_max_y = max(bounds_max_y,y);
	  bounds_max_z = max(bounds_max_z,z);
        }

      b.lower.x = reduce_min(bounds_min_x);
      b.lower.y = reduce_min(bounds_min_y);
      b.lower.z = reduce_min(bounds_min_z);
      b.upper.x = reduce_max(bounds_max_x);
      b.upper.y = reduce_max(bounds_max_y);
      b.upper.z = reduce_max(bounds_max_z);
#endif

      b.lower.a = 0.0f;
      b.upper.a = 0.0f;

      assert( std::isfinite(b.lower.x) );
      assert( std::isfinite(b.lower.y) );
      assert( std::isfinite(b.lower.z) );

      assert( std::isfinite(b.upper.x) );
      assert( std::isfinite(b.upper.y) );
      assert( std::isfinite(b.upper.z) );


      assert(b.lower.x <= b.upper.x);
      assert(b.lower.y <= b.upper.y);
      assert(b.lower.z <= b.upper.z);

#else
      BBox3fa b = patch.bounds();
      if (unlikely(isGregoryPatch()))
        {
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 0));
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 1));
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 2));
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 3));
        }
#endif

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
      _freea(ptr);
#endif
      return b;
    }

#if USE_RANGE_EVAL

    struct BVH4SubdivPatch1CachedBuilderBinnedSAHClass : public Builder
    {
      ALIGNED_STRUCT;

      BVH4* bvh;
      Scene* scene;
      mvector<PrimRef> prims; 
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      BVH4SubdivPatch1CachedBuilderBinnedSAHClass (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          for (size_t i=0; i<iter.size(); i++) // FIXME: parallelize
            if (iter[i]) iter[i]->initializeHalfEdgeStructures();
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4SubdivPatch1CachedBuilderBinnedSAH");

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
            patch_eval_subdivision(mesh->getHalfEdge(f),[&](const Vec2f uv[4], const int subdiv[4], const float edge_level[4], int subPatch) { s++; });
          }
          return PrimInfo(s,empty,empty);
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });
        numPrimitives = pinfo.size();
        
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        
        prims.resize(numPrimitives);
        
        /* Allocate memory for gregory and b-spline patches */
        if (this->bvh->size_data_mem < sizeof(SubdivPatch1Cached) * numPrimitives) 
        {
          DBG_CACHE_BUILDER(std::cout << "DEALLOCATING SUBDIVPATCH1CACHED MEMORY" << std::endl);
          if (this->bvh->data_mem) os_free( this->bvh->data_mem, this->bvh->size_data_mem );
          this->bvh->data_mem      = nullptr;
          this->bvh->size_data_mem = 0;
        }
        
        if (bvh->data_mem == nullptr)
        {
          DBG_CACHE_BUILDER(std::cout << "ALLOCATING SUBDIVPATCH1CACHED MEMORY FOR " << numPrimitives << " PRIMITIVES" << std::endl);
          this->bvh->size_data_mem = sizeof(SubdivPatch1Cached) * numPrimitives;
          if ( this->bvh->size_data_mem != 0) this->bvh->data_mem = os_malloc( this->bvh->size_data_mem );        
          else                                this->bvh->data_mem = nullptr;
        }
        assert(this->bvh->data_mem);
        SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;
        
        //double T0 = getSeconds();
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
              subdiv_patches[patchIndex] = SubdivPatch1Cached(mesh->id,f,subPatch,mesh,uv,edge_level,subdiv);
#if 0
              BBox3fa bounds;
              size_t new_root_ref = SubdivPatch1CachedIntersector1::buildSubdivPatchTreeCompact(subdiv_patches[patchIndex],SharedLazyTessellationCache::threadState(),mesh,&bounds);
              const size_t combinedTime = SharedLazyTessellationCache::sharedLazyTessellationCache.getTime(scene->commitCounter+1);
              subdiv_patches[patchIndex].root_ref = SharedLazyTessellationCache::Tag((void*)new_root_ref,combinedTime);
              SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(SharedLazyTessellationCache::threadState());
#else
              const BBox3fa bounds = getBounds1(subdiv_patches[patchIndex],mesh);
#endif
              prims[patchIndex] = PrimRef(bounds,patchIndex);
              s.add(bounds);
            });
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a, b); });
        //double T1 = getSeconds();
        //PRINT(1000.0f*(T1-T0))

        DBG_CACHE_BUILDER(std::cout << "create prims in " << 1000.0f*t0 << "ms " << std::endl);
        DBG_CACHE_BUILDER(std::cout << "pinfo.bounds " << pinfo << std::endl);
        
        if (numPrimitives)
        {
          DBG_CACHE_BUILDER(std::cout << "start building..." << std::endl);
          
          BVH4::NodeRef root;
          BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
            (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),BVH4::NoRotate(),
             [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> int {
              size_t items = current.pinfo.size();
              assert(items == 1);
              const unsigned int patchIndex = prims[current.prims.begin()].ID();
              SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;
              *current.parent = bvh->encodeLeaf((char*)&subdiv_patches[patchIndex],1);
              return 0;
            },
             progress,
             prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1,1.0f,1.0f);
          bvh->set(root,pinfo.geomBounds,pinfo.size());
          DBG_CACHE_BUILDER(std::cout << "finsihed building" << std::endl);
          
        }
        else
          bvh->set(BVH4::emptyNode,empty,0);	  
        
	/* clear temporary data for static geometry */
	bool staticGeom = scene->isStatic();
	if (staticGeom) prims.clear();
        bvh->alloc.cleanup();
        bvh->postBuild(t0);
      }
      
      void clear() {
        prims.clear();
      }
    };

#else

    struct BVH4SubdivPatch1CachedBuilderBinnedSAHClass : public Builder
    {
      ALIGNED_STRUCT;

      BVH4* bvh;
      Scene* scene;
      mvector<PrimRef> prims; 
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      BVH4SubdivPatch1CachedBuilderBinnedSAHClass (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      BBox3fa refit(BVH4::NodeRef& ref)
      {
        /* this is a empty node */
        if (unlikely(ref == BVH4::emptyNode))
          return BBox3fa( empty );
        
        assert(ref != BVH4::invalidNode);
        
        /* this is a leaf node */
        if (unlikely(ref.isLeaf()))
        {
          size_t num;
          SubdivPatch1Cached *sptr = (SubdivPatch1Cached*)ref.leaf(num);
          const size_t index = ((size_t)sptr - (size_t)this->bvh->data_mem) / sizeof(SubdivPatch1Cached);
          //assert(index < numPrimitives);
          return prims[index].bounds(); 
        }
      
        /* recurse if this is an internal node */
        BVH4::Node* node = ref.node();
        const BBox3fa bounds0 = refit(node->child(0));
        const BBox3fa bounds1 = refit(node->child(1));
        const BBox3fa bounds2 = refit(node->child(2));
        const BBox3fa bounds3 = refit(node->child(3));
        
        /* AOS to SOA transform */
        BBox<Vec3f4> bounds;
        transpose((float4&)bounds0.lower,(float4&)bounds1.lower,(float4&)bounds2.lower,(float4&)bounds3.lower,bounds.lower.x,bounds.lower.y,bounds.lower.z);
        transpose((float4&)bounds0.upper,(float4&)bounds1.upper,(float4&)bounds2.upper,(float4&)bounds3.upper,bounds.upper.x,bounds.upper.y,bounds.upper.z);
        
        /* set new bounds */
        node->lower_x = bounds.lower.x;
        node->lower_y = bounds.lower.y;
        node->lower_z = bounds.lower.z;
        node->upper_x = bounds.upper.x;
        node->upper_y = bounds.upper.y;
        node->upper_z = bounds.upper.z;
        
        /* return merged bounds */
        const float lower_x = reduce_min(bounds.lower.x);
        const float lower_y = reduce_min(bounds.lower.y);
        const float lower_z = reduce_min(bounds.lower.z);
        const float upper_x = reduce_max(bounds.upper.x);
        const float upper_y = reduce_max(bounds.upper.y);
        const float upper_z = reduce_max(bounds.upper.z);
        return BBox3fa(Vec3fa(lower_x,lower_y,lower_z),
                       Vec3fa(upper_x,upper_y,upper_z));
      }
      
      void build(size_t, size_t) 
      {
        /* initialize all half edge structures */
        size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives > 0 || scene->isInterpolatable()) {
          Scene::Iterator<SubdivMesh> iter(scene,scene->isInterpolatable());
          for (size_t i=0; i<iter.size(); i++) // FIXME: parallelize
            if (iter[i]) iter[i]->initializeHalfEdgeStructures();
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        bvh->alloc.reset();

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4SubdivPatch1CachedBuilderBinnedSAH");

        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        /* if only edge levels have changed, we use fastUpdateMode */
        bool fastUpdateMode = true;
        size_t fastUpdateMode_numFaces = 0;
        Scene::Iterator<SubdivMesh> iter(scene);
        for (size_t i=0; i<iter.size(); i++)
          if (iter[i]) {
            fastUpdateMode_numFaces += iter[i]->size(); // FIXME: same as numPrimitives above?
            if (!iter[i]->checkLevelUpdate()) fastUpdateMode = false;
          }
        if (bvh->numPrimitives == 0 || bvh->numPrimitives != fastUpdateMode_numFaces || bvh->root == BVH4::emptyNode)
          fastUpdateMode = false;

        /* initialize allocator and parallel_for_for_prefix_sum */
        pstate.init(iter,size_t(1024));
        numPrimitives = fastUpdateMode_numFaces;
        if (!fastUpdateMode) 
        {
          PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
          { 
            size_t s = 0;
            for (size_t f=r.begin(); f!=r.end(); ++f) 
            {          
              if (!mesh->valid(f)) continue;
	      feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                                   [&](const CatmullClarkPatch3fa& patch, const int depth, const Vec2f uv[4], const int subdiv[4], 
                                                       const BezierCurve3fa *border, const int border_flags) { s++; });
            }
            return PrimInfo(s,empty,empty);
          }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });
          numPrimitives = pinfo.size();
        }
        
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        
        prims.resize(numPrimitives);
        
        /* Allocate memory for gregory and b-spline patches */
        if (this->bvh->size_data_mem < sizeof(SubdivPatch1Cached) * numPrimitives) 
        {
          DBG_CACHE_BUILDER(std::cout << "DEALLOCATING SUBDIVPATCH1CACHED MEMORY" << std::endl);
          if (this->bvh->data_mem) os_free( this->bvh->data_mem, this->bvh->size_data_mem );
          this->bvh->data_mem      = nullptr;
          this->bvh->size_data_mem = 0;
        }
        
        if (bvh->data_mem == nullptr)
        {
          DBG_CACHE_BUILDER(std::cout << "ALLOCATING SUBDIVPATCH1CACHED MEMORY FOR " << numPrimitives << " PRIMITIVES" << std::endl);
          this->bvh->size_data_mem = sizeof(SubdivPatch1Cached) * numPrimitives;
          if ( this->bvh->size_data_mem != 0) this->bvh->data_mem = os_malloc( this->bvh->size_data_mem );        
          else                                this->bvh->data_mem = nullptr;
        }
        assert(this->bvh->data_mem);
        SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;
        
        //double T0 = getSeconds();
        PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          PrimInfo s(empty);
          for (size_t f=r.begin(); f!=r.end(); ++f) 
          {
            if (!mesh->valid(f)) continue;
            
            if (unlikely(fastUpdateMode == false))
            {
              feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),[&](const CatmullClarkPatch3fa& ipatch, const int depth, const Vec2f uv[4], const int subdiv[4], 
                                                                                                      const BezierCurve3fa *border, const int border_flags)
              {
                float edge_level[4] = {
                  ipatch.ring[0].edge_level,
                  ipatch.ring[1].edge_level,
                  ipatch.ring[2].edge_level,
                  ipatch.ring[3].edge_level
                };
                
                const unsigned int patchIndex = base.size()+s.size();
                assert(patchIndex < numPrimitives);
                subdiv_patches[patchIndex] = SubdivPatch1Cached(ipatch,depth,mesh->id,f,mesh,uv,edge_level,subdiv,border,border_flags);
                
                /* compute patch bounds */
                const BBox3fa bounds = getBounds1(subdiv_patches[patchIndex],mesh);
                prims[patchIndex] = PrimRef(bounds,patchIndex);
                s.add(bounds);
              });
            }
            else
            {
              const SubdivMesh::HalfEdge* first_half_edge = mesh->getHalfEdge(f);
              const size_t numEdges = first_half_edge->numEdges();
              
              float edge_level[4] = {
                first_half_edge[0].edge_level,
                first_half_edge[1].edge_level,
                first_half_edge[2].edge_level,
                first_half_edge[3].edge_level
              };

              const int neighborSubdiv[4] = {
                feature_adaptive_gregory_neighbor_subdiv(first_half_edge[0]),
                feature_adaptive_gregory_neighbor_subdiv(first_half_edge[1]),
                feature_adaptive_gregory_neighbor_subdiv(first_half_edge[2]),
                feature_adaptive_gregory_neighbor_subdiv(first_half_edge[3])
              };

              /* updating triangular bezier patch */
              if (numEdges == 3) { // FIXME: buggy, for triangle the above code accesses half edge array out of bounds
                edge_level[3] = first_half_edge[1].edge_level;
              }
              
              const unsigned int patchIndex = base.size()+s.size();
              subdiv_patches[patchIndex].updateEdgeLevels(edge_level,neighborSubdiv,mesh);
              subdiv_patches[patchIndex].resetRootRef();
              
              /* compute patch bounds */
              const BBox3fa bounds = getBounds1(subdiv_patches[patchIndex],mesh);
              prims[patchIndex] = PrimRef(bounds,patchIndex);
              s.add(bounds);	      
            }
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a, b); });
        //double T1 = getSeconds();
        //PRINT(1000.0f*(T1-T0));

        DBG_CACHE_BUILDER(std::cout << "create prims in " << 1000.0f*t0 << "ms " << std::endl);
        DBG_CACHE_BUILDER(std::cout << "pinfo.bounds " << pinfo << std::endl);
        
        if (fastUpdateMode)
        {
          if (bvh->root != BVH4::emptyNode)
            refit(bvh->root);
        }
        else
        {
          if (numPrimitives)
	  {
	    DBG_CACHE_BUILDER(std::cout << "start building..." << std::endl);
            
	    BVH4::NodeRef root;
            BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
	      (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),BVH4::NoRotate(),
	       [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> int {
		size_t items = current.pinfo.size();
		assert(items == 1);
		const unsigned int patchIndex = prims[current.prims.begin()].ID();
		SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;
		*current.parent = bvh->encodeLeaf((char*)&subdiv_patches[patchIndex],1);
		return 0;
	      },
	       progress,
	       prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1,1.0f,1.0f);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());
	    DBG_CACHE_BUILDER(std::cout << "finsihed building" << std::endl);
            
	  }
          else
            bvh->set(BVH4::emptyNode,empty,0);	  
        }
        
	/* clear temporary data for static geometry */
	bool staticGeom = scene->isStatic();
	if (staticGeom) prims.clear();
        bvh->alloc.cleanup();
        bvh->postBuild(t0);
      }
      
      void clear() {
        prims.clear();
      }
    };

#endif
    
    /* entry functions for the scene builder */
    Builder* BVH4SubdivPatch1BuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4SubdivPatch1BuilderBinnedSAHClass((BVH4*)bvh,scene); }
    Builder* BVH4SubdivGridBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4SubdivGridBuilderBinnedSAHClass((BVH4*)bvh,scene); }
    Builder* BVH4SubdivGridEagerBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4SubdivGridEagerBuilderBinnedSAHClass((BVH4*)bvh,scene); }
    Builder* BVH4SubdivGridLazyBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4SubdivGridLazyBuilderBinnedSAHClass((BVH4*)bvh,scene); }
    Builder* BVH4SubdivPatch1CachedBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4SubdivPatch1CachedBuilderBinnedSAHClass((BVH4*)bvh,scene); }
  }
}
