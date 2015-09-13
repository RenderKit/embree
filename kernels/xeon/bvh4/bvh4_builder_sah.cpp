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

      // FIXME: shrink bvh->alloc in destructor here an in other builders too

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
      
        /* tree rotations */
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

        double t0 = bvh->preBuild(mesh ? nullptr : TOSTRING(isa) "::BVH4BuilderSAH");

#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
          bvh->alloc.init_estimate(numSplitPrimitives*sizeof(PrimRef));
	    prims.resize(numSplitPrimitives);
            auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
            auto virtualprogress = BuildProgressMonitorFromClosure(progress);
	    PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) 
              : createPrimRefArray<Mesh,1>(scene,prims,virtualprogress);

            if (presplitFactor > 1.0f)
              pinfo = presplit<Mesh>(scene, pinfo, prims);

	    BVH4::NodeRef root;
            BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
	      (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),rotate,CreateBVH4Leaf<Primitive>(bvh,prims.data()),progress,
	       prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());

#if ROTATE_TREE
            for (int i=0; i<ROTATE_TREE; i++) 
              BVH4Rotate::rotate(bvh,bvh->root);
            bvh->clearBarrier(bvh->root);
#endif

            bvh->layoutLargeNodes(pinfo.size()*0.005f);

#if PROFILE
        }); 
#endif

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

    struct SpatialSplitHeuristic
    {
      Scene* scene;

      SpatialSplitHeuristic(Scene* scene)
        : scene(scene) {}

      float operator() (const PrimRef& prim)
      {
        const size_t geomID = prim.geomID();
        const size_t primID = prim.primID();
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa v0 = mesh->vertex(tri.v[0]);
        const Vec3fa v1 = mesh->vertex(tri.v[1]);
        const Vec3fa v2 = mesh->vertex(tri.v[2]);
        const float triAreaX = triangleArea(Vec2f(v0.y,v0.z),Vec2f(v1.y,v1.z),Vec2f(v2.y,v2.z));
        const float triAreaY = triangleArea(Vec2f(v0.x,v0.z),Vec2f(v1.x,v1.z),Vec2f(v2.x,v2.z));
        const float triAreaZ = triangleArea(Vec2f(v0.x,v0.y),Vec2f(v1.x,v1.y),Vec2f(v2.x,v2.y));
        const float triBoxArea = triAreaX+triAreaY+triAreaZ;
        const float boxArea = area(prim.bounds());
        //assert(boxArea>=2.0f*triBoxArea);
        //return max(0.0f,boxArea-0.9f*2.0f*triBoxArea);
        return boxArea;
        //return triBoxArea;
        //return boxArea-triBoxArea;
      }
    };

    template<typename Mesh, typename Primitive>
    struct BVH4BuilderSpatialSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH4BuilderSpatialSAH (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH4BuilderSpatialSAH (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          bvh->clear();
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

        double t0 = bvh->preBuild(mesh ? nullptr : TOSTRING(isa) "::BVH4BuilderSpatialSAH");

#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
	    bvh->alloc.init_estimate(numSplitPrimitives*sizeof(PrimRef)); 
	    //prims.resize(numSplitPrimitives);
	    //PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
            PrimRefList prims;
            auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
            auto virtualprogress = BuildProgressMonitorFromClosure(progress);
            PrimInfo pinfo = createPrimRefList<Mesh,1>(scene,prims,virtualprogress);
            
            SpatialSplitHeuristic heuristic(scene);

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
                  //const size_t n = 64;
                  const size_t n = min(ssize_t(127), max(ssize_t(1), ssize_t(nf)));
                  N += n;
                  prim.lower.a |= n << 24;
                }
              }
              return N;
            },std::plus<size_t>());

	    BVH4::NodeRef root;
            BVHBuilderBinnedSpatialSAH::build_reduce<BVH4::NodeRef>
	      (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),rotate,CreateBVH4ListLeaf<Primitive>(bvh),
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
	       prims,pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());

#if ROTATE_TREE
            for (int i=0; i<ROTATE_TREE; i++) 
              BVH4Rotate::rotate(bvh,bvh->root);
            bvh->clearBarrier(bvh->root);
#endif

             bvh->layoutLargeNodes(pinfo.size()*0.005f);
#if PROFILE
        }); 
#endif
        
        /* clear temporary data for static geometry */
        bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          bvh->shrink();
        }
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

    struct CreateBVH4NodeMB
    {
      __forceinline CreateBVH4NodeMB (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline BVH4::NodeMB* operator() (const isa::BVHBuilderBinnedSAH::BuildRecord& current, BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, Allocator* alloc) 
      {
        BVH4::NodeMB* node = (BVH4::NodeMB*) alloc->alloc0.malloc(sizeof(BVH4::NodeMB)); node->clear();
        for (size_t i=0; i<N; i++) {
          children[i].parent = (size_t*)&node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH4* bvh;
    };

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
      
	/* reduction function */
	auto reduce = [] (BVH4::NodeMB* node, const std::pair<BBox3fa,BBox3fa>* bounds, const size_t N) -> std::pair<BBox3fa,BBox3fa>
	{
	  assert(N <= BVH4::N);
	  BBox3fa bounds0 = empty;
	  BBox3fa bounds1 = empty;
	  for (size_t i=0; i<N; i++) {
	    const BBox3fa b0 = bounds[i].first;
	    const BBox3fa b1 = bounds[i].second;
	    node->set(i,b0,b1);
	    bounds0 = merge(bounds0,b0);
	    bounds1 = merge(bounds1,b1);
	  }
	  return std::pair<BBox3fa,BBox3fa>(bounds0,bounds1);
	};
	auto identity = std::make_pair(BBox3fa(empty),BBox3fa(empty));

        double t0 = bvh->preBuild(mesh ? nullptr : TOSTRING(isa) "::BVH4BuilderMblurSAH");

	//profile("BVH4BuilderMblurBinnedSAH",2,20,numPrimitives,[&] () {
	    
	    if (bvh->device->verbosity(1)) t0 = getSeconds();
	    
	    bvh->alloc.init_estimate(numPrimitives*sizeof(PrimRef));
	    prims.resize(numPrimitives);
            auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
            auto virtualprogress = BuildProgressMonitorFromClosure(progress);
	    const PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims,virtualprogress) 
              : createPrimRefArray<Mesh,2>(scene,prims,virtualprogress);
	    BVH4::NodeRef root;
            BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
	      (root,BVH4::CreateAlloc(bvh),identity,CreateBVH4NodeMB(bvh),reduce,CreateBVH4LeafMB<Primitive>(bvh,prims.data()),progress,
	       prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());
            
            //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: enable

        //});

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
