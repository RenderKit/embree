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
#include "bvh4_rotate.h"
#include "bvh4_statistics.h"
#include "common/profile.h"

#include "builders_new/primrefgen.h"
#include "builders_new/presplit.h"
#include "builders_new/bvh_builder.h"

#include "geometry/bezier1v.h"
#include "geometry/bezier1i.h"
#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"
#include "geometry/virtual_accel.h"

#define ROTATE_TREE 0
#define PROFILE 0

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
      
      __forceinline BVH4::Node* operator() (const isa::BuildRecord<BVH4::NodeRef>& current, BuildRecord<BVH4::NodeRef>** children, const size_t N, Allocator* alloc) 
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i]->geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH4* bvh;
    };

    template<typename Primitive>
    struct CreateLeaf
    {
      __forceinline CreateLeaf (BVH4* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline int operator() (const BuildRecord<BVH4::NodeRef>& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t n = current.prims.size();
        size_t items = Primitive::blocks(n);
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return n;
      }

      BVH4* bvh;
      PrimRef* prims;
    };
	  
    template<typename Mesh, typename Primitive>
    struct BVH4BuilderBinnedSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      vector<PrimRef> prims; 
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH4BuilderBinnedSAH (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)), 
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH4BuilderBinnedSAH (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(mesh->parent), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)), 
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}
      
      void build(size_t, size_t)
      {
        /* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          prims.resize(0,true);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));

        /* reduction function */
	auto rotate = [&] (BVH4::Node* node, const size_t* counts, const size_t N) -> size_t
	{
          size_t n = 0;
#if ROTATE_TREE
	  assert(N <= BVH4::N);
          for (size_t i=0; i<N; i++) 
            n += counts[i];
          if (n >= 4096) {
            for (size_t i=0; i<N; i++) {
              if (counts[i] < 4096) {
                for (int j=0; j<ROTATE_TREE; j++) 
                  BVH4Rotate::rotate(bvh,node->child(i)); 
                node->child(i).setBarrier();
              }
            }
          }
#endif
	  return n;
	};

        /* verbose mode */
        if (g_verbose >= 1)
	  std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderBinnedSAH " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;

#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
	    if (g_verbose >= 1) t0 = getSeconds();
	    
	    prims.resize(numSplitPrimitives);
	    bvh->alloc2.init(numSplitPrimitives*sizeof(PrimRef),numSplitPrimitives*sizeof(BVH4::Node));  // FIXME: better estimate
            
            auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
            auto virtualprogress = BuildProgressMonitorFromClosure(progress);
	    PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) 
              : createPrimRefArray<Mesh,1>(scene,prims,virtualprogress);

            if (presplitFactor > 1.0f)
              pinfo = presplit<Mesh>(scene, pinfo, prims);

	    BVH4::NodeRef root = bvh_builder_reduce_binned_sah_internal<BVH4::NodeRef>
	      (CreateAlloc(bvh),size_t(0),CreateBVH4Node(bvh),rotate,CreateLeaf<Primitive>(bvh,prims.data()),
               progress,
	       prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());
            
#if ROTATE_TREE
            for (int i=0; i<ROTATE_TREE; i++) 
              BVH4Rotate::rotate(bvh,bvh->root);
            bvh->clearBarrier(bvh->root);
#endif

            bvh->layoutLargeNodes(pinfo.size()*0.005f);

            //timer("bvh4_builder_binned_sah");

	    if (g_verbose >= 1) dt = getSeconds()-t0;
	    
#if PROFILE
      dt = timer.avg();
      }); 
#endif

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) prims.resize(0,true);
        bvh->alloc2.cleanup();
	
	/* verbose mode */
	if (g_verbose >= 1)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mprim/s)" << std::endl;
	if (g_verbose >= 2)
	  bvh->printStatistics();

        /* benchmark mode */
        if (g_benchmark) {
          BVH4Statistics stat(bvh);
          std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
        }
      }

      void clear() {
        prims.clear();
      }
    };
    
    /* entry functions for the scene builder */
    Builder* BVH4Bezier1vSceneBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<BezierCurves,Bezier1v>((BVH4*)bvh,scene,1,1,1.0f,1,1,mode); }
    Builder* BVH4Bezier1iSceneBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<BezierCurves,Bezier1i>((BVH4*)bvh,scene,1,1,1.0f,1,1,mode); }
    Builder* BVH4Triangle1SceneBuilderBinnedSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1>((BVH4*)bvh,scene,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4SceneBuilderBinnedSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8SceneBuilderBinnedSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle8>((BVH4*)bvh,scene,8,4,1.0f,8,inf,mode); }
#endif
    Builder* BVH4Triangle1vSceneBuilderBinnedSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1v>((BVH4*)bvh,scene,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderBinnedSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4v>((BVH4*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderBinnedSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH4VirtualSceneBuilderBinnedSAH    (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<UserGeometryBase,AccelSetItem>((BVH4*)bvh,scene,1,1,1.0f,1,1,mode); }

    /* entry functions for the mesh builders */
    Builder* BVH4Triangle1MeshBuilderBinnedSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1>((BVH4*)bvh,mesh,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4MeshBuilderBinnedSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4>((BVH4*)bvh,mesh,4,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderBinnedSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle8>((BVH4*)bvh,mesh,8,4,1.0f,8,inf,mode); }
#endif
    Builder* BVH4Triangle1vMeshBuilderBinnedSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1v>((BVH4*)bvh,mesh,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4vMeshBuilderBinnedSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,4,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iMeshBuilderBinnedSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,4,4,1.0f,4,inf,mode); }
  }
}
