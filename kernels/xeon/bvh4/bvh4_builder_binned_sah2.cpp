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
#include "builders_new/presplit.h"
#include "builders_new/bvh_builder2.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"
#include "geometry/triangle4v_mb.h"

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
      
      __forceinline int operator() (const isa::BuildRecord2<BVH4::NodeRef>& current, BuildRecord2<BVH4::NodeRef>** children, const size_t N, Allocator* alloc) 
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i]->pinfo.geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return 0;
      }

      BVH4* bvh;
    };

    //BVH4::NodeRef lastLeaf;

    template<typename Primitive>
    struct CreateLeaf
    {
      __forceinline CreateLeaf (BVH4* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline int operator() (const BuildRecord2<BVH4::NodeRef>& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        //lastLeaf = node;
        *current.parent = node;
	return 1;
      }

      BVH4* bvh;
      PrimRef* prims;
    };
    
    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Mesh, typename Primitive>
    struct BVH4BuilderBinnedSAH2 : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH4BuilderBinnedSAH2 (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH4BuilderBinnedSAH2 (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(NULL), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
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
      
        /* verbose mode */
        if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderBinnedSAH2 " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
	//profile(2,20,numPrimitives,[&] (ProfileTimer& timer) {
	    
	    if (g_verbose >= 1 && mesh == NULL) t0 = getSeconds();
	    
	    bvh->alloc2.init(numSplitPrimitives*sizeof(PrimRef),numSplitPrimitives*sizeof(BVH4::Node));  // FIXME: better estimate
	    prims.resize(numSplitPrimitives);
	    PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);

            if (presplitFactor > 1.0f)
              pinfo = presplit<Mesh>(scene, pinfo, prims);

	    BVH4::NodeRef root = bvh_builder_binned_sah2_internal<BVH4::NodeRef>
	      (CreateAlloc(bvh),CreateBVH4Node(bvh),CreateLeaf<Primitive>(bvh,prims.data()),
	       prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());
            //bvh->set(lastLeaf,pinfo.geomBounds,pinfo.size());

	    if (g_verbose >= 1 && mesh == NULL) dt = getSeconds()-t0;

            //  timer("BVH4BuilderBinnedSAH2");
	    
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
    Builder* BVH4Triangle1SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle1>((BVH4*)bvh,scene,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle8>((BVH4*)bvh,scene,8,4,1.0f,8,inf,mode); }
#endif
    Builder* BVH4Triangle1vSceneBuilderBinnedSAH2 (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle1v>((BVH4*)bvh,scene,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderBinnedSAH2 (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4v>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderBinnedSAH2 (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4i>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }

    /* entry functions for the mesh builders */
    Builder* BVH4Triangle1MeshBuilderBinnedSAH2  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle1>((BVH4*)bvh,mesh,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4MeshBuilderBinnedSAH2  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4>((BVH4*)bvh,mesh,4,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderBinnedSAH2  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle8>((BVH4*)bvh,mesh,8,4,1.0f,8,inf,mode); }
#endif
    Builder* BVH4Triangle1vMeshBuilderBinnedSAH2 (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle1v>((BVH4*)bvh,mesh,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4vMeshBuilderBinnedSAH2 (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,2,2,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iMeshBuilderBinnedSAH2 (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,2,2,1.0f,4,inf,mode); }

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    struct CreateListBVH4Node // FIXME: merge with above class
    {
      __forceinline CreateListBVH4Node (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline int operator() (const isa::BuildRecord2<BVH4::NodeRef,PrimRefList>& current, BuildRecord2<BVH4::NodeRef,PrimRefList>** children, const size_t N, Allocator* alloc) 
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i]->pinfo.geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return 0;
      }

      BVH4* bvh;
    };

    template<typename Primitive>
    struct CreateListLeaf
    {
      __forceinline CreateListLeaf (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline int operator() (BuildRecord2<BVH4::NodeRef, PrimRefList>& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t N = Primitive::blocks(current.pinfo.size());
        Primitive* leaf = (Primitive*) alloc->alloc1.malloc(N*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)leaf,N);

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
	return 1;
      }

      BVH4* bvh;
      //const vector_t<std::pair<int,int>>& geomIDprimID;
    };

    template<typename Mesh, typename Primitive>
    struct BVH4ListBuilderBinnedSAH2 : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      //vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH4ListBuilderBinnedSAH2 (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH4ListBuilderBinnedSAH2 (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(NULL), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          //prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
      
        /* verbose mode */
        if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderBinnedSAH2 " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
	//profile(2,20,numPrimitives,[&] (ProfileTimer& timer) {
	    
	    if (g_verbose >= 1 && mesh == NULL) t0 = getSeconds();
	    
	    bvh->alloc2.init(numSplitPrimitives*sizeof(PrimRef),numSplitPrimitives*sizeof(BVH4::Node));  // FIXME: better estimate
	    //prims.resize(numSplitPrimitives);
	    //PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
            PrimRefList prims;
            PrimInfo pinfo = createPrimRefList<Mesh,1>(scene,prims);
            //PRINT(pinfo.size());

            /* calculate total surface area */
            float A = 0.0f;
            for (PrimRefList::block_iterator_unsafe iter = prims; iter; iter++) {
              A += area(*iter);
              iter++;
            }

            /* try to calculate a number of splits per primitive, such that
             * we do not generate more primitives than the size of the prims
             * array */
            float spatialSplitFactor = 1.5f;
            float f = spatialSplitFactor/0.9f;
            size_t N = 0;
            do {
              f *= 0.9f;
              N = 0;
              
              for (PrimRefList::block_iterator_unsafe iter = prims; iter; iter++) {
                const float nf = ceil(f*pinfo.size()*area(*iter)/A);
                const size_t n = min(ssize_t(255), max(ssize_t(1), ssize_t(nf)));
                N+=n;
              }
            } while (N>spatialSplitFactor*pinfo.size());

            //PRINT(spatialSplitFactor*pinfo.size());
            //PRINT(N);

            /*vector_t<std::pair<int,int>> geomIDprimID;
            geomIDprimID.resize(pinfo.size());
            PrimRefList::block_iterator_unsafe iter = prims;
            for (size_t i=0; i<pinfo.size(); i++) {
              geomIDprimID[i].first  = iter->lower.a;
              geomIDprimID[i].second = iter->upper.a;
              iter->lower.a = i;
              iter->upper.a = 16;
              iter++;
              }*/

            for (PrimRefList::block_iterator_unsafe iter = prims; iter; iter++) {
              assert((iter->lower.a & 0xFF000000) == 0);
              const float nf = ceil(f*pinfo.size()*area(*iter)/A);
              const size_t n = min(ssize_t(255), max(ssize_t(1), ssize_t(nf)));
              iter->lower.a |= n << 24;
            }

            //if (presplitFactor > 1.0f)
            //pinfo = presplit<Mesh>(scene, pinfo, prims);

	    BVH4::NodeRef root = bvh_builder_spatial_sah2_internal<BVH4::NodeRef>
	      (scene,CreateAlloc(bvh),CreateListBVH4Node(bvh),CreateListLeaf<Primitive>(bvh),
               [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o)
               {
                TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF); 
                TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
                const Vec3fa v0 = mesh->vertex(tri.v[0]);
                const Vec3fa v1 = mesh->vertex(tri.v[1]);
                const Vec3fa v2 = mesh->vertex(tri.v[2]);
                splitTriangle(prim,dim,pos,v0,v1,v2,left_o,right_o);
              },
	       prims,pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());

	    if (g_verbose >= 1 && mesh == NULL) dt = getSeconds()-t0;

            //  timer("BVH4BuilderBinnedSAH2");
	    
            //});

	/* clear temporary data for static geometry */
	//bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	//if (staticGeom) prims.resize(0,true);
	bvh->alloc2.cleanup();

	/* verbose mode */
	if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
	if (g_verbose >= 2 && mesh == NULL)
	  bvh->printStatistics();
      }
    };

    /* entry functions for the scene builder */
    //Builder* BVH4Triangle1SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle1>((BVH4*)bvh,scene,1,1,1.0f,2,inf,mode); }
    Builder* BVH4Triangle4SceneListBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4ListBuilderBinnedSAH2<TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,4,1.0f,4,inf,mode); }
//#if defined(__AVX__)
    //Builder* BVH4Triangle8SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle8>((BVH4*)bvh,scene,8,4,1.0f,8,inf,mode); }
//#endif
    //Builder* BVH4Triangle1vSceneBuilderBinnedSAH2 (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle1v>((BVH4*)bvh,scene,1,1,1.0f,2,inf,mode); }
    //Builder* BVH4Triangle4vSceneBuilderBinnedSAH2 (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4v>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }
    //Builder* BVH4Triangle4iSceneBuilderBinnedSAH2 (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH2<TriangleMesh,Triangle4i>((BVH4*)bvh,scene,2,2,1.0f,4,inf,mode); }

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    struct CreateBVH4NodeMB
    {
      __forceinline CreateBVH4NodeMB (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline BVH4::NodeMB* operator() (const isa::BuildRecord2<BVH4::NodeRef>& current, BuildRecord2<BVH4::NodeRef>** children, const size_t N, Allocator* alloc) 
      {
        BVH4::NodeMB* node = (BVH4::NodeMB*) alloc->alloc0.malloc(sizeof(BVH4::NodeMB)); node->clear();
        for (size_t i=0; i<N; i++) {
          //node->set(i,children[i]->pinfo.geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH4* bvh;
    };

    template<typename Primitive>
    struct CreateLeafMB
    {
      __forceinline CreateLeafMB (BVH4* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline std::pair<BBox3fa,BBox3fa> operator() (const BuildRecord2<BVH4::NodeRef>& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
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
    struct BVH4BuilderMblurBinnedSAH2 : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;

      BVH4BuilderMblurBinnedSAH2 (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)) {}

      BVH4BuilderMblurBinnedSAH2 (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(NULL), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,2>();
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
      
	/* reduction function */
	auto reduce = [] (BVH4::NodeMB* node, const std::pair<BBox3fa,BBox3fa>* bounds, const size_t N) 
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

        /* verbose mode */
        if (g_verbose >= 1)
	  std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderMblurBinnedSAH ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
	//profile("BVH4BuilderMblurBinnedSAH",2,20,numPrimitives,[&] () {
	    
	    if (g_verbose >= 1) t0 = getSeconds();
	    
	    bvh->alloc2.init(numPrimitives*sizeof(PrimRef),numPrimitives*sizeof(BVH4::NodeMB));  // FIXME: better estimate
	    prims.resize(numPrimitives);
	    const PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,2>(scene,prims);
	    BVH4::NodeRef root = bvh_builder_reduce_binned_sah2_internal<BVH4::NodeRef>
	      (CreateAlloc(bvh),identity,CreateBVH4NodeMB(bvh),reduce,CreateLeafMB<Primitive>(bvh,prims.data()),
	       prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH4::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());

	    if (g_verbose >= 1) dt = getSeconds()-t0;
	    
	    //});

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) prims.resize(0,true);
	bvh->alloc2.cleanup();

	/* verbose mode */
	if (g_verbose >= 1)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mprim/s)" << std::endl;
	if (g_verbose >= 2)
	  bvh->printStatistics();
      }
    };

    Builder* BVH4Triangle4vMBSceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderMblurBinnedSAH2<TriangleMesh,Triangle4vMB>((BVH4*)bvh,scene,4,4,1.0f,4,inf); }
  }
}
