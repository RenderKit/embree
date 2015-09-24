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
#include "bvh4_builder.h"
#include "bvh4_rotate.h"

#include "../builders/primrefgen.h"
#include "../builders/presplit.h"
#include "../builders/bvh_builder_sah.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle4v.h"
#include "../geometry/triangle4i.h"
#include "../geometry/triangle4v_mb.h"
#include "../geometry/object.h"

#define ROTATE_TREE 0
#define PROFILE 0

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    template<typename Primitive>
    struct CreateBVH4Leaf
    {
      __forceinline CreateBVH4Leaf (BVH4* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline size_t operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t n = current.prims.size();
        size_t items = Primitive::blocks(n);
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = BVH4::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return n;
      }

      BVH4* bvh;
      PrimRef* prims;
    };
    
    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Mesh, typename Primitive>
    struct BVH4BuilderSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH4BuilderSAH (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH4BuilderSAH (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }
        
        double t0 = bvh->preBuild(mesh ? nullptr : TOSTRING(isa) "::BVH4BuilderSAH");

        /* create primref array */
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
        prims.resize(numSplitPrimitives);
        PrimInfo pinfo = mesh ? 
          createPrimRefArray<Mesh>  (mesh ,prims,scene->progressInterface) : 
          createPrimRefArray<Mesh,1>(scene,prims,scene->progressInterface);
        
        /* perform pre-splitting */
        if (presplitFactor > 1.0f) 
          pinfo = presplit<Mesh>(scene, pinfo, prims);
        
        /* call BVH builder */
        BVH4Builder::build(bvh,CreateBVH4Leaf<Primitive>(bvh,prims.data()),scene->progressInterface,prims.data(),pinfo,sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
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
    Builder* BVH4Bezier1vSceneBuilderSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSAH<BezierCurves,Bezier1v>((BVH4*)bvh,scene,1,1,1.0f,1,1,mode); }
    Builder* BVH4Bezier1iSceneBuilderSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSAH<BezierCurves,Bezier1i>((BVH4*)bvh,scene,1,1,1.0f,1,1,mode); }

    Builder* BVH4Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle8>((BVH4*)bvh,scene,8,4,1.0f,8,inf,mode); }
#endif
    Builder* BVH4Triangle4vSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle4v>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle4i>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }
    
    Builder* BVH4VirtualSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSAH<AccelSet,Object>((BVH4*)bvh,scene,1,1,1.0f,1,1,mode); }

    /* entry functions for the mesh builders */
    Builder* BVH4Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle4>((BVH4*)bvh,mesh,4,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle8>((BVH4*)bvh,mesh,8,4,1.0f,8,inf,mode); }
#endif
    Builder* BVH4Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,2,2,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderSAH<TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,2,2,1.0f,4,inf,mode); }

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Primitive>
    struct CreateBVH4ListLeaf
    {
      __forceinline CreateBVH4ListLeaf (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline size_t operator() (BVHBuilderBinnedSpatialSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t n = current.pinfo.size();
        size_t N = Primitive::blocks(n);
        Primitive* leaf = (Primitive*) alloc->alloc1.malloc(N*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)leaf,N);

        PrimRefList::block_iterator_unsafe iter1(current.prims);
        while (iter1) {
          iter1->lower.a &= 0x00FFFFFF;
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

      BVH4* bvh;
    };

    template<typename Mesh, typename Primitive>
    struct BVH4BuilderSpatialSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH4BuilderSpatialSAH (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, 
                             const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          bvh->clear();
          return;
        }
      
        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4BuilderSpatialSAH");
        
        /* progress interface */
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        /* create primref list */
        PrimRefList prims;
        PrimInfo pinfo = createPrimRefList<Mesh,1>(scene,prims,virtualprogress);
        
        /* calculate total surface area */
        PrimRefList::iterator iter = prims;
        const size_t threadCount = TaskSchedulerTBB::threadCount();
        const double A = parallel_reduce(size_t(0),threadCount,0.0, [&] (const range<size_t>& r) -> double // FIXME: this sum is not deterministic
        {
          double A = 0.0f;
          while (PrimRefList::item* block = iter.next()) {
            for (size_t i=0; i<block->size(); i++) 
              A += area(block->at(i).bounds());
          }
          return A;
        },std::plus<double>());
        
        /* calculate maxmal number of spatial splits per primitive */
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
              //const size_t n = 64;
              const size_t n = min(ssize_t(127), max(ssize_t(1), ssize_t(nf)));
              N += n;
              prim.lower.a |= n << 24;
            }
          }
          return N;
        },std::plus<size_t>());
        
        /* function that splits a primitive at some position and dimension */
        auto splitPrimitive = [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) {
          TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF); 
          TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
          const Vec3fa v0 = mesh->vertex(tri.v[0]);
          const Vec3fa v1 = mesh->vertex(tri.v[1]);
          const Vec3fa v2 = mesh->vertex(tri.v[2]);
          splitTriangle(prim,dim,pos,v0,v1,v2,left_o,right_o);
        };
             
        /* call BVH builder */
        BVH4BuilderSpatial::build(bvh,splitPrimitive,CreateBVH4ListLeaf<Primitive>(bvh),virtualprogress,prims,pinfo,
                                  sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
        
        /* clear temporary data for static geometry */
	if (scene->isStatic()) bvh->shrink();
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
      }
    };

    /* entry functions for the scene builder */
    Builder* BVH4Triangle4SceneBuilderSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSpatialSAH<TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8SceneBuilderSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSpatialSAH<TriangleMesh,Triangle8>((BVH4*)bvh,scene,8,4,1.0f,8,inf,mode); }
#endif
    Builder* BVH4Triangle4vSceneBuilderSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSpatialSAH<TriangleMesh,Triangle4v>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderSpatialSAH<TriangleMesh,Triangle4i>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }


    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Primitive>
    struct CreateBVH4LeafMB
    {
      __forceinline CreateBVH4LeafMB (BVH4* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline std::pair<BBox3fa,BBox3fa> operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
	BBox3fa bounds0 = empty;
	BBox3fa bounds1 = empty;
        for (size_t i=0; i<items; i++) {
          auto bounds = accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
	  bounds0.extend(bounds.first);
	  bounds1.extend(bounds.second);
        }
        *current.parent = node;
	return std::make_pair(bounds0,bounds1);
      }

      BVH4* bvh;
      PrimRef* prims;
    };

    template<typename Mesh, typename Primitive>
    struct BVH4BuilderMblurSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims; 
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;

      BVH4BuilderMblurSAH (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)) {}

      BVH4BuilderMblurSAH (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(nullptr), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,2>();
        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }
      
        double t0 = bvh->preBuild(mesh ? nullptr : TOSTRING(isa) "::BVH4BuilderMblurSAH");

        if (bvh->device->verbosity(1)) t0 = getSeconds();
	    
        bvh->alloc.init_estimate(numPrimitives*sizeof(PrimRef));
        prims.resize(numPrimitives);
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);
        const PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) 
          : createPrimRefArray<Mesh,2>(scene,prims,virtualprogress);
        
        /* call BVH builder */
        BVH4BuilderMblur::build(bvh,CreateBVH4LeafMB<Primitive>(bvh,prims.data()),virtualprogress,prims.data(),pinfo,
                                sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
        
	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
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

    Builder* BVH4Triangle4vMBSceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderMblurSAH<TriangleMesh,Triangle4vMB>((BVH4*)bvh,scene,4,4,1.0f,4,inf); }
  }
}
