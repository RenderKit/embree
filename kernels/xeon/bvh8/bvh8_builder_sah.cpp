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
#include "bvh8_builder.h"

#include "../builders/primrefgen.h"
#include "../builders/presplit.h"
#include "../builders/bvh_builder_sah.h"

#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/trianglepairs4.h"

#include "../../algorithms/parallel_for_for_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

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
	    
        if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) t0 = getSeconds();
        
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        bvh->alloc2.init_estimate(numSplitPrimitives*sizeof(PrimRef));
        prims.resize(numSplitPrimitives);
        PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) : createPrimRefArray<Mesh,1>(scene,prims,virtualprogress);
        if (presplitFactor > 1.0f)
          pinfo = presplit<Mesh>(scene, pinfo, prims);

        BVH8Builder::build(bvh,CreateBVH8Leaf<Primitive>(bvh,prims.data()),virtualprogress,
                           prims.data(),pinfo,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);
        
        if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) dt = getSeconds()-t0;
        
	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          prims.clear();
          bvh->alloc2.shrink();
        }
	bvh->alloc2.cleanup();

	/* verbose mode */
	if (bvh->device->verbosity(1) && mesh == nullptr) {
          const size_t usedBytes = bvh->alloc2.getUsedBytes();
	  std::cout << "[DONE] " << 1000.0f*dt << "ms, " << numPrimitives/dt*1E-6 << " Mtris/s, " << usedBytes/dt*1E-9 << " GB/s"  << std::endl;
        }
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


    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Mesh, typename Primitive>
    struct BVH8BuilderSAHTrianglePairs : public Builder
    {
      BVH8* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;

      BVH8BuilderSAHTrianglePairs (BVH8* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)) 
      {}

      BVH8BuilderSAHTrianglePairs (BVH8* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numOriginalPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numOriginalPrimitives == 0) {
          prims.resize(numOriginalPrimitives);
          bvh->set(BVH8::emptyNode,empty,0);
          return;
        }

      
        /* verbose mode */
        if (bvh->device->verbosity(1) && mesh == nullptr)
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderSAH " << " ... " << std::flush;


	double t0 = 0.0f, dt = 0.0f;
	    
        if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) t0 = getSeconds();

        size_t numPrimitives = 0;

        /* compute number of triangle pairs */
        {
          ParallelForForPrefixSumState<size_t> pstate;
          Scene::Iterator<Mesh,1> iter(scene);
          pstate.init(iter,size_t(1024));
          numPrimitives = parallel_for_for_prefix_sum( pstate, iter, size_t(0), [&](Mesh* mesh, const range<size_t>& r, size_t k, const size_t base) -> size_t
                                                       {
                                                         size_t prims = 0;
                                                         for (size_t j=r.begin(); j<r.end(); j++)
                                                         {
                                                           BBox3fa bounds = empty;
                                                           if (!mesh->valid(j,&bounds)) continue;
                                                           const PrimRef prim(bounds,mesh->id,j);
                                                           prims++;
                                                           if (j+1 < r.end())
                                                           {
                                                             if (!mesh->valid(j+1)) continue;
                                                              TriangleMesh* trimesh = (TriangleMesh*)mesh;
                                                              if (TriangleMesh::sharedEdge(trimesh->triangle(j),
                                                                                           trimesh->triangle(j+1)) != -1)
                                                                j++;
                                                           }
                                                         }
                                                         return prims;
                                                       }, [](const size_t a, const size_t b) { return a+b; });
          
        }
        PRINT(numPrimitives);
        PRINT(numOriginalPrimitives);
        PRINT(100*numPrimitives / double(numOriginalPrimitives));

        exit(0);

        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        bvh->alloc2.init_estimate(numPrimitives*sizeof(PrimRef));
        prims.resize(numPrimitives);
        assert(!mesh);

        /* compute prim refs for triangle pairs */
        //PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) : createPrimRefArray<Mesh,1>(scene,prims,virtualprogress);


        ParallelForForPrefixSumState<PrimInfo> pstate;
        Scene::Iterator<Mesh,1> iter(scene);

        pstate.init(iter,size_t(1024));
        PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
                                                      {
                                                        // 1.)  need to determine k !!!!
                                                        // 2.)  encode non-pairs with flagged geomID
                                                        PrimInfo pinfo(empty);
                                                        for (size_t j=r.begin(); j<r.end(); j++)
                                                        {
                                                          BBox3fa bounds = empty;
                                                          if (!mesh->valid(j,&bounds)) continue;
                                                          const PrimRef prim(bounds,mesh->id,j);
                                                          pinfo.add(bounds,bounds.center2());

                                                          TriangleMesh* trimesh = (TriangleMesh*)mesh;

                                                          if (j+1 < r.end())
                                                          {
                                                            BBox3fa bounds_second = empty;
                                                            if (!mesh->valid(j+1,&bounds_second)) continue;
                                                            TriangleMesh* trimesh = (TriangleMesh*)mesh;
                                                            if (TriangleMesh::sharedEdge(trimesh->triangle(j),
                                                                                         trimesh->triangle(j+1)) != -1)
                                                            {
                                                              j++;
                                                            }
                                                              
                                                          }

                                                        }
                                                        return pinfo;
                                                      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });


        // ============================================

        BVH8Builder::build(bvh,CreateBVH8Leaf<Primitive>(bvh,prims.data()),virtualprogress,
                           prims.data(),pinfo,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);
        
        if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) dt = getSeconds()-t0;
        
	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          prims.clear();
          bvh->alloc2.shrink();
        }
	bvh->alloc2.cleanup();

	/* verbose mode */
	if (bvh->device->verbosity(1) && mesh == nullptr) {
          const size_t usedBytes = bvh->alloc2.getUsedBytes();
	  std::cout << "[DONE] " << 1000.0f*dt << "ms, " << numPrimitives/dt*1E-6 << " Mtris/s, " << usedBytes/dt*1E-9 << " GB/s"  << std::endl;
        }
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
    Builder* BVH8TrianglePairs4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSAHTrianglePairs<TriangleMesh,TrianglePairs4>((BVH8*)bvh,scene,4,4,1.0f,4,inf,mode); }


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
      
        /* verbose mode */
        if (bvh->device->verbosity(1) && mesh == nullptr)
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderSAH (spatial)" << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
	    
        if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) t0 = getSeconds();
	
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);
        
        bvh->alloc2.init_estimate(numSplitPrimitives*sizeof(PrimRef));
        //prims.resize(numSplitPrimitives);
        //PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
        PrimRefList prims;
        PrimInfo pinfo = createPrimRefList<Mesh,1>(scene,prims,virtualprogress);
        
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

        auto splitPrimitive = [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) {
            TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF); 
            TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
            const Vec3fa v0 = mesh->vertex(tri.v[0]);
            const Vec3fa v1 = mesh->vertex(tri.v[1]);
            const Vec3fa v2 = mesh->vertex(tri.v[2]);
            splitTriangle(prim,dim,pos,v0,v1,v2,left_o,right_o);
          };
        
        BVH8BuilderSpatial::build(bvh,splitPrimitive,CreateBVH8ListLeaf<Primitive>(bvh),
                                  virtualprogress,prims,pinfo,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);
        
        if ((bvh->device->benchmark || bvh->device->verbosity(1)) && mesh == nullptr) dt = getSeconds()-t0;
            
	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          bvh->alloc2.shrink();
        }
	bvh->alloc2.cleanup();

        /* verbose mode */
	if (bvh->device->verbosity(1) && mesh == nullptr) {
          const size_t usedBytes = bvh->alloc2.getUsedBytes();
	  std::cout << "[DONE] " << 1000.0f*dt << "ms, " << numPrimitives/dt*1E-6 << " Mtris/s, " << usedBytes/dt*1E-9 << " GB/s"  << std::endl;
        }
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
  }
}
