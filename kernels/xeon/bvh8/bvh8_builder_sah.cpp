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

#include "../builders/primrefgen.h"
#include "../builders/presplit.h"
#include "../builders/bvh_builder_sah.h"

#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle8v.h"
#include "../geometry/trianglepairs8.h"

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
      
      __forceinline int operator() (const isa::BVHBuilderBinnedSAH::BuildRecord& current, BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, Allocator* alloc) 
      {
        BVH8::Node* node = nullptr;
        //if (current.pinfo.size() > 4096) node = (BVH8::Node*)   bvh->alloc2.malloc(sizeof(BVH8::Node),sizeof(BVH8::Node));
        //else
        node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node), 1 << BVH8::alignment); 
        node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i].pinfo.geomBounds);
          children[i].parent = (size_t*) &node->child(i);
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
      
      __forceinline int operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc)
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

      BVH8* bvh;
      PrimRef* prims;
    };
    
    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Mesh, typename Primitive>
    struct BVH8BuilderSAH : public Builder
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

      BVH8BuilderSAH (BVH8* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH8BuilderSAH (BVH8* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
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
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderSAH " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
	    
          if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) t0 = getSeconds();
          
          auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
          auto virtualprogress = BuildProgressMonitorFromClosure(progress);
          
	    bvh->alloc2.init_estimate(numSplitPrimitives*sizeof(PrimRef));
	    prims.resize(numSplitPrimitives);
	    PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) : createPrimRefArray<Mesh,1>(scene,prims,virtualprogress);
            if (presplitFactor > 1.0f)
              pinfo = presplit<Mesh>(scene, pinfo, prims);
	    BVH8::NodeRef root; 
            BVHBuilderBinnedSAH::build<BVH8::NodeRef>
              (root,CreateBVH8Alloc(bvh),CreateBVH8Node(bvh),CreateBVH8Leaf<Primitive>(bvh,prims.data()), progress,
               prims.data(),pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);

            bvh->set(root,pinfo.geomBounds,pinfo.size());
            bvh->layoutLargeNodes(numSplitPrimitives*0.005f);

	    if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) dt = getSeconds()-t0;

#if PROFILE
           dt = timer.avg();
        }); 
#endif	

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          prims.clear();
          bvh->alloc2.shrink();
        }
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
    Builder* BVH8Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSAH<TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle8SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSAH<TriangleMesh,Triangle8>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }

    //Builder* BVH8Triangle8vSceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSAH<TriangleMesh,Triangle8v>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }

    Builder* BVH8TrianglePairs8SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSAH<TriangleMesh,Triangle8>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }


    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

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

    template<typename Mesh, typename Primitive>
    struct BVH8BuilderSpatialSAH : public Builder
    {
      BVH8* bvh;
      Scene* scene;
      Mesh* mesh;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH8BuilderSpatialSAH (BVH8* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH8BuilderSpatialSAH (BVH8* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          //prims.resize(numPrimitives);
          bvh->set(BVH8::emptyNode,empty,0);
          return;
        }
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
      
        /* reduction function */
	auto rotate = [&] (BVH8::Node* node, const size_t* counts, const size_t N) -> size_t
	{
          size_t n = 0;
#if ROTATE_TREE
	  assert(N <= BVH8::N);
          for (size_t i=0; i<N; i++) 
            n += counts[i];
          if (n >= 4096) {
            for (size_t i=0; i<N; i++) {
              if (counts[i] < 4096) {
                for (int j=0; j<ROTATE_TREE; j++) 
                  BVH8Rotate::rotate(bvh,node->child(i)); 
                node->child(i).setBarrier();
              }
            }
          }
#endif
	  return n;
	};

        /* verbose mode */
        if (bvh->device->verbosity(1) && mesh == nullptr)
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderSAH (spatial)" << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
	    
          if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) t0 = getSeconds();
	    
            auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
            auto virtualprogress = BuildProgressMonitorFromClosure(progress);

	    bvh->alloc2.init_estimate(numSplitPrimitives*sizeof(PrimRef));
	    //prims.resize(numSplitPrimitives);
	    //PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
            PrimRefList prims;
            PrimInfo pinfo = createPrimRefList<Mesh,1>(scene,prims,virtualprogress);
            //PRINT(pinfo.size());

            //SpatialSplitHeuristic heuristic(scene);

            /* calculate total surface area */
            PrimRefList::iterator iter = prims;
            const size_t threadCount = TaskSchedulerTBB::threadCount();
            const double A = parallel_reduce(size_t(0),threadCount,0.0, [&] (const range<size_t>& r) -> double // FIXME: this sum is not deterministic
            {
              double A = 0.0f;
              while (PrimRefList::item* block = iter.next()) {
                for (size_t i=0; i<block->size(); i++) 
                  A += area(block->at(i).bounds());
                //A += heuristic(block->at(i));
              }
              return A;
            },std::plus<double>());

            /* calculate number of maximal spatial splits per primitive */
            float f = 10.0f;
            iter = prims;
            const size_t N = parallel_reduce(size_t(0),threadCount,size_t(0), [&] (const range<size_t>& r) -> size_t
            {
              size_t N = 0;
              while (PrimRefList::item* block = iter.next()) {
                for (size_t i=0; i<block->size(); i++) {
                  PrimRef& prim = block->at(i);
                  assert((prim.lower.a & 0xFF000000) == 0);
                  const float nf = ceil(f*pinfo.size()*area(prim.bounds())/A);
                  //const size_t n = 16;
                  const size_t n = 4+min(ssize_t(127-4), max(ssize_t(1), ssize_t(nf)));
                  N += n;
                  prim.lower.a |= n << 24;
                }
              }
              return N;
            },std::plus<size_t>());

	    BVH8::NodeRef root;
            BVHBuilderBinnedSpatialSAH::build_reduce<BVH8::NodeRef>
	      (root,CreateBVH8Alloc(bvh),size_t(0),CreateListBVH8Node(bvh),rotate,CreateBVH8ListLeaf<Primitive>(bvh),
               [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o)
               {
                TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF); 
                TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
                const Vec3fa v0 = mesh->vertex(tri.v[0]);
                const Vec3fa v1 = mesh->vertex(tri.v[1]);
                const Vec3fa v2 = mesh->vertex(tri.v[2]);
                splitTriangle(prim,dim,pos,v0,v1,v2,left_o,right_o);
              },
               progress,
	       prims,pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());
            
#if ROTATE_TREE
            for (int i=0; i<ROTATE_TREE; i++) 
              BVH8Rotate::rotate(bvh,bvh->root);
            bvh->clearBarrier(bvh->root);
#endif
            
            bvh->layoutLargeNodes(pinfo.size()*0.005f);

            if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) dt = getSeconds()-t0;
            
#if PROFILE
            dt = timer.avg();
        }); 
#endif	

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          bvh->alloc2.shrink();
        }
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
        //prims.clear();
      }
    };

    /* entry functions for the scene builder */
    Builder* BVH8Triangle4SceneBuilderSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSpatialSAH<TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle8SceneBuilderSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSpatialSAH<TriangleMesh,Triangle8>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }
    //Builder* BVH8Triangle8vSceneBuilderSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSpatialSAH<TriangleMesh,Triangle8v>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }
  }
}
