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
#include "bvh8_statistics.h"
//#include "common/profile.h"

#include "../builders/primrefgen.h"
#include "../builders/presplit.h"
#include "../builders/bvh_builder_bonsai.h"

#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"

#define PROFILE 0

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    struct CreateBVH8Alloc
    {
      __forceinline CreateBVH8Alloc (BVH8* bvh) : bvh(bvh) {}
      __forceinline Allocator* operator() () const { return bvh->alloc2.threadLocal2();  }

      BVH8* bvh;
    };

    struct CreateBVH8Node
    {
      __forceinline CreateBVH8Node (BVH8* bvh) : bvh(bvh) {}

      __forceinline int operator() (const isa::BVHBuilderBonsai::BuildRecord& current, BVHBuilderBonsai::BuildRecord* children, const size_t N, Allocator* alloc)
      {
        BVH8::Node* node = nullptr;
        node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node), 1 << BVH8::alignment);
        node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i].pinfo.geomBounds);
          children[i].parent = (size_t*) &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);

		return 0;
      }

		__forceinline int operator() (const isa::BVHBuilderBonsai::BuildRecord& current, BVHBuilderBonsai::BuildRecord* children, const size_t N, Allocator* alloc, int& underFilledNodes)
		{
			BVH8::Node* node = nullptr;
			node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node), 1 << BVH8::alignment);
			node->clear();
			for (size_t i=0; i<N; i++) {
				node->set(i,children[i].pinfo.geomBounds);
				node->child(i) = BVH8::NodeRef(*children[i].parent);
			}
			*current.parent = bvh->encodeNode(node);
			return 0;
		}

      BVH8* bvh;
    };

    template<typename Primitive>
    struct CreateBVH8Leaf
    {
      __forceinline CreateBVH8Leaf (BVH8* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}

      __forceinline int operator() (const BVHBuilderBonsai::BuildRecord& current, Allocator* alloc)
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive), 1 << BVH8::alignment);
        BVH8::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return 1;
      }

		__forceinline int operator() (const BVHBuilderBonsai::BuildRecord& current, PrimRef* gatheredRefs, Allocator* alloc)
		{
			size_t items = Primitive::blocks(current.prims.size());
			size_t start = 0; //< gathered refs start at 0 instead of current.prims.begin();
			Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive), 1 << BVH8::alignment);
			BVH8::NodeRef node = bvh->encodeLeaf((char*)accel,items);
			for (size_t i=0; i<items; i++) {
				accel[i].fill(gatheredRefs,start,current.prims.size(),bvh->scene,false); //< last ref is prims.size() instead of prims.end()
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
    struct BVH8BuilderBonsai : public Builder
    {
      BVH8* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH8BuilderBonsai (BVH8* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
		presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH8BuilderBonsai (BVH8* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
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
        if (bvh->device->verbosity(1) && mesh == nullptr)
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderBonsai " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

    bool splitTriangles = false; // not currently used

	double t0 = 0.0f, dt = 0.0f;
#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif

          if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) t0 = getSeconds();

          auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
          auto virtualprogress = BuildProgressMonitorFromClosure(progress);

	    bvh->alloc2.init_estimate(numSplitPrimitives*sizeof(PrimRef));
	    if (splitTriangles)
        prims.resize(numSplitPrimitives*4);
      else
        prims.resize(numSplitPrimitives);

      PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) : createPrimRefArray<Mesh,1>(scene,prims,virtualprogress);
            if (presplitFactor > 1.0f)
              pinfo = presplit<Mesh>(scene, pinfo, prims);
	    BVH8::NodeRef root;
            BVHBuilderBonsai::build<BVH8::NodeRef>
              (root,CreateBVH8Alloc(bvh),CreateBVH8Node(bvh),CreateBVH8Leaf<Primitive>(bvh,prims.data()), progress,
               prims.data(),pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost, bvh->scene, splitTriangles, pinfo.size());

            bvh->set(root,pinfo.geomBounds,pinfo.size());
            //bvh->layoutLargeNodes(numSplitPrimitives*0.005f);

	    if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) dt = getSeconds()-t0;

#if PROFILE
           dt = timer.avg();
        });
#endif

//  exit(0);
	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) prims.clear();
	bvh->alloc2.cleanup();

	/* verbose mode */
	if (bvh->device->verbosity(1) && mesh == nullptr)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
	if (bvh->device->verbosity(2) && mesh == nullptr)
	  bvh->printStatistics();

        /* benchmark mode */
          if (bvh->device->benchmark) {
          BVH8Statistics stat(bvh);
          std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
        }
      }

      void clear() {
        prims.clear();
      }
    };

    /* entry functions for the scene builder */
    Builder* BVH8Triangle4SceneBuilderBonsai  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderBonsai<TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle8SceneBuilderBonsai  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderBonsai<TriangleMesh,Triangle8>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
#if 0
    struct CreateListBVH8Node // FIXME: merge with above class
    {
      __forceinline CreateListBVH8Node (BVH8* bvh) : bvh(bvh) {}

      __forceinline BVH8::Node* operator() (const isa::BVHBuilderBinnedSpatialSAH::BuildRecord& current, BVHBuilderBinnedSpatialSAH::BuildRecord* children, const size_t N, Allocator* alloc)
      {
        BVH8::Node* node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node), 1 << BVH8::alignment); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i].pinfo.geomBounds);
          children[i].parent = (size_t*) &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH8* bvh;
    };


    template<typename Primitive>
    struct CreateBVH8ListLeaf
    {
      __forceinline CreateBVH8ListLeaf (BVH8* bvh) : bvh(bvh) {}

      __forceinline size_t operator() (BVHBuilderBinnedSpatialSAH::BuildRecord& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t n = current.pinfo.size();
        size_t N = Primitive::blocks(n);
        Primitive* leaf = (Primitive*) alloc->alloc1.malloc(N*sizeof(Primitive), 1 << BVH8::alignment);
        BVH8::NodeRef node = bvh->encodeLeaf((char*)leaf,N);

        /*PrimRefList::block_iterator_unsafe iter1(current.prims);
        while (iter1) {
          const int i = iter1->lower.a;
          iter1->lower.a = geomIDprimID[i].first;
          iter1->upper.a = geomIDprimID[i].second;
          iter1++;
          }*/
        PrimRefList::block_iterator_unsafe iter1(current.prims);
        while (iter1) {
          iter1->lower.a &= 0x00FFFFFF; // FIXME: hack
          iter1++;
        }

        /* insert all triangles */
        PrimRefList::block_iterator_unsafe iter(current.prims);
        for (size_t i=0; i<N; i++) leaf[i].fill(iter,bvh->scene,false);
        assert(!iter);

        /* free all primitive blocks */
        while (PrimRefList::item* block = current.prims.take())
          delete block;

        *current.parent = node;
	return n;
      }

      BVH8* bvh;
    };
#endif
  }
}
