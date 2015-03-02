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

#include "bvh8.h"
#include "common/profile.h"

#include "builders_new/primrefgen.h"
#include "builders_new/presplit.h"
#include "builders_new/bvh_builder2.h"

#include "geometry/triangle4.h"
#include "geometry/triangle8.h"

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    struct CreateAlloc
    {
      __forceinline CreateAlloc (BVH8* bvh) : bvh(bvh) {}
      __forceinline Allocator* operator() () const { return bvh->alloc2.threadLocal2();  }

      BVH8* bvh;
    };

    struct CreateBVH8Node
    {
      __forceinline CreateBVH8Node (BVH8* bvh) : bvh(bvh) {}
      
      __forceinline int operator() (const isa::BuildRecord2<BVH8::NodeRef>& current, BuildRecord2<BVH8::NodeRef>** children, const size_t N, Allocator* alloc) 
      {
        BVH8::Node* node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i]->pinfo.geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return 0;
      }

      BVH8* bvh;
    };

    template<typename Primitive>
    struct CreateLeaf
    {
      __forceinline CreateLeaf (BVH8* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline int operator() (const BuildRecord2<BVH8::NodeRef>& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH8::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return 1;
      }

      BVH8* bvh;
      PrimRef* prims;
    };
    
    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Mesh, typename Primitive>
    struct BVH8BuilderBinnedSAH2 : public Builder
    {
      BVH8* bvh;
      Scene* scene;
      Mesh* mesh;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH8BuilderBinnedSAH2 (BVH8* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH8BuilderBinnedSAH2 (BVH8* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(NULL), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH8::emptyNode,empty,0);
          return;
        }
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
      
        /* verbose mode */
        if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderBinnedSAH2 " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
	//profile("BVH8BuilderBinnedSAH2",2,20,numPrimitives,[&] () {
	    
	    if (g_verbose >= 1 && mesh == NULL) t0 = getSeconds();
	    
	    bvh->alloc2.init(numSplitPrimitives*sizeof(PrimRef),numSplitPrimitives*sizeof(BVH8::Node));  // FIXME: better estimate
	    prims.resize(numSplitPrimitives);
	    PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
            if (presplitFactor > 1.0f)
              pinfo = presplit<Mesh>(scene, pinfo, prims);
            
	    BVH8::NodeRef root = bvh_builder_binned_sah2_internal<BVH8::NodeRef>
	      (CreateAlloc(bvh),CreateBVH8Node(bvh),CreateLeaf<Primitive>(bvh,prims.data()),
	       prims.data(),pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());

	    if (g_verbose >= 1 && mesh == NULL) dt = getSeconds()-t0;
	    
	    //});

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) prims.resize(0,true);
	bvh->alloc2.cleanup();

	/* verbose mode */
	if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
	if (g_verbose >= 2 && mesh == NULL)
	  bvh->printStatistics();
      }
    };

    /* entry functions for the scene builder */
    Builder* BVH8Triangle4SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderBinnedSAH2<TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle8SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderBinnedSAH2<TriangleMesh,Triangle8>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }
  }
}
