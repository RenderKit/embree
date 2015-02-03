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
#include "common/profile.h"

#include "builders_new/primrefgen.h"
#include "builders_new/bvh_builder.h"

#include "algorithms/parallel_for_for.h"
#include "algorithms/parallel_for_for_prefix_sum.h"

#include "common/subdiv/feature_adaptive_gregory.h"
#include "common/subdiv/feature_adaptive_bspline.h"

#include "geometry/grid.h"
#include "geometry/subdivpatch1.h"
#include "geometry/subdivpatch1cached.h"

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    struct CreateAlloc
    {
      __forceinline CreateAlloc (BVH4* bvh) : bvh(bvh) {}
      __forceinline Allocator* operator() () const { return bvh->alloc2.threadLocal2();  }

      BVH4* bvh;
    };

    struct CreateBVH4Node
    {
      __forceinline CreateBVH4Node (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline int operator() (const isa::BuildRecord<BVH4::NodeRef>& current, BuildRecord<BVH4::NodeRef>** children, const size_t N, Allocator* alloc) 
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i]->geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return 0;
      }

      BVH4* bvh;
    };

    struct BVH4BuilderSubdivBinnedSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      ParallelForForPrefixSumState<PrimInfo> pstate;
      
      BVH4BuilderSubdivBinnedSAH (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t, size_t) 
      {
        /* skip build for empty scene */
	const size_t numPrimitives = scene->getNumPrimitives<SubdivMesh,1>();
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        bvh->alloc2.reset();

        /* verbose mode */
        if (g_verbose >= 1)
	  std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderSubdivBinnedSAH ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
        if (g_verbose >= 1) t0 = getSeconds();

        /* initialize all half edge structures */
        Scene::Iterator<SubdivMesh> iter(scene);
        for (size_t i=0; i<iter.size(); i++) // FIXME: parallelize
          if (iter[i]) iter[i]->initializeHalfEdgeStructures();
        
        /* initialize allocator and parallel_for_for_prefix_sum */
        pstate.init(iter,size_t(1024));
        PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          size_t s = 0;
          for (size_t f=r.begin(); f!=r.end(); ++f) 
          {
            if (!mesh->valid(f)) continue;
            
            feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                                 [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4])
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

        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          FastAllocator::ThreadLocal& alloc = *bvh->alloc2.threadLocal();
          
          PrimInfo s(empty);
          for (size_t f=r.begin(); f!=r.end(); ++f) {
            if (!mesh->valid(f)) continue;
            
            feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                                 [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4])
            {
              /*if (!patch.isRegular())
                {
                Grid* leaf = (Grid*) bvh->alloc2.malloc(sizeof(Grid),16);
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
              
              GregoryPatch patcheval; patcheval.init(patch);
              //BSplinePatch patcheval; patcheval.init(patch);
              size_t N = Grid::createEager(mesh->id,f,scene,patcheval,alloc,&prims[base.size()+s.size()],0,nx,0,ny,uv,pattern0,pattern1,pattern2,pattern3,pattern_x,pattern_y);
              assert(N == Grid::getNumEagerLeaves(nx,ny));
              for (size_t i=0; i<N; i++)
                s.add(prims[base.size()+s.size()].bounds());
            });
          }
          return s;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
        
        BVH4::NodeRef root = bvh_builder_binned_sah_internal<BVH4::NodeRef>
          (CreateAlloc(bvh),CreateBVH4Node(bvh),
           [&] (const BuildRecord<BVH4::NodeRef>& current, PrimRef* prims, Allocator* alloc) {
             if (current.size() != 1) THROW_RUNTIME_ERROR("bvh4_builder_subdiv: internal error");
             *current.parent = (BVH4::NodeRef) prims[current.begin].ID();
             return 0;
           },
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1);
        bvh->set(root,pinfo.geomBounds,pinfo.size());
        
        if (g_verbose >= 1) dt = getSeconds()-t0;

	/* clear temporary data for static geometry */
	bool staticGeom = scene->isStatic();
	if (staticGeom) prims.resize(0,true);
        bvh->alloc2.cleanup();
	
	/* verbose mode */
	if (g_verbose >= 1)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mprim/s)" << std::endl;
	if (g_verbose >= 2)
	  bvh->printStatistics();
      }
    };
    
    /* entry functions for the scene builder */
    Builder* BVH4SubdivGridBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSubdivBinnedSAH((BVH4*)bvh,scene); }
  }
}
