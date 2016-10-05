// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "bvh.h"
#include "bvh_builder.h"

#include "../builders/primrefgen.h"
#include "../builders/presplit.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"
#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglei.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei_mb.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/quadi_mb.h"
#include "../geometry/object.h"

#include "../common/state.h"

#define PROFILE 0
#define PROFILE_RUNS 20

namespace embree
{
  namespace isa
  {
    MAYBE_UNUSED static const float travCost = 1.0f;
    MAYBE_UNUSED static const float defaultPresplitFactor = 1.2f;

    typedef FastAllocator::ThreadLocal2 Allocator;

    template<int N, typename Primitive>
    struct CreateLeaf
    {
      typedef BVHN<N> BVH;

      __forceinline CreateLeaf (BVH* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline size_t operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t n = current.prims.size();
        size_t items = Primitive::blocks(n);
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive),BVH::byteNodeAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return n;
      }

      BVH* bvh;
      PrimRef* prims;
    };


    template<int N, typename Primitive>
    struct CreateLeafQuantized
    {
      typedef BVHN<N> BVH;

      __forceinline CreateLeafQuantized (BVH* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline size_t operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t n = current.prims.size();
        size_t items = Primitive::blocks(n);
        size_t start = current.prims.begin();
        // todo alloc0/1 or alloc
        Primitive* accel = (Primitive*) alloc->alloc0.malloc(items*sizeof(Primitive),BVH::byteNodeAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }

        *current.parent = node;
	return n;
      }

      BVH* bvh;
      PrimRef* prims;
    };


    template<int N, typename Primitive>
    struct CreateLeafSpatial
    {
      typedef BVHN<N> BVH;

      __forceinline CreateLeafSpatial (BVH* bvh, PrimRef* prims0) : bvh(bvh), prims0(prims0) {}
      
      __forceinline size_t operator() (const BVHBuilderBinnedFastSpatialSAH::BuildRecord& current, Allocator* alloc)
      {
        PrimRef* const source = prims0;

        size_t n = current.prims.size();


        size_t items = Primitive::blocks(n);

        size_t start = current.prims.begin();

        // remove number of split encoding
        for (size_t i=0; i<n; i++) 
          source[start+i].lower.a &= 0x00FFFFFF;

        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive),BVH::byteNodeAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(source,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return n;
      }

      BVH* bvh;
      PrimRef* prims0;
    };
    
    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVHNBuilderSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? defaultPresplitFactor : 1.0f) {}


      BVHNBuilderSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY ) ? defaultPresplitFactor : 1.0f) {}

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

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

            /* create primref array */
            const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
            prims.resize(numSplitPrimitives);
            PrimInfo pinfo = mesh ? 
              createPrimRefArray<Mesh>  (mesh ,prims,bvh->scene->progressInterface) : 
              createPrimRefArray<Mesh,1>(scene,prims,bvh->scene->progressInterface);
        
            /* perform pre-splitting */
            if (presplitFactor > 1.0f) 
              pinfo = presplit<Mesh>(scene, pinfo, prims);
        
            /* call BVH builder */
            bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
            BVHNBuilder<N>::build(bvh,CreateLeaf<N,Primitive>(bvh,prims.data()),bvh->scene->progressInterface,prims.data(),pinfo,sahBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

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

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderSAHQuantized : public Builder
    {
      typedef BVHN<N> BVH;
      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVHNBuilderSAHQuantized (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? defaultPresplitFactor : 1.0f) {}

      BVHNBuilderSAHQuantized (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? defaultPresplitFactor : 1.0f) {}

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

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::QBVH" + toString(N) + "BuilderSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif
            /* create primref array */
            const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
            prims.resize(numSplitPrimitives);
            PrimInfo pinfo = mesh ? 
              createPrimRefArray<Mesh>  (mesh ,prims,bvh->scene->progressInterface) : 
              createPrimRefArray<Mesh,1>(scene,prims,bvh->scene->progressInterface);
        
            /* perform pre-splitting */
            if (presplitFactor > 1.0f) 
              pinfo = presplit<Mesh>(scene, pinfo, prims);
        
            /* call BVH builder */
            bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
            BVHNBuilderQuantized<N>::build(bvh,CreateLeafQuantized<N,Primitive>(bvh,prims.data()),bvh->scene->progressInterface,prims.data(),pinfo,sahBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

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

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Primitive>
    struct CreateListLeaf
    {
      typedef BVHN<N> BVH;

      __forceinline CreateListLeaf (BVH* bvh) : bvh(bvh) {}
      
      __forceinline size_t operator() (BVHBuilderBinnedSpatialSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t n = current.pinfo.size();
        size_t num = Primitive::blocks(n);
        Primitive* leaf = (Primitive*) alloc->alloc1.malloc(num*sizeof(Primitive),BVH::byteNodeAlignment);
        typename BVH::NodeRef node = bvh->encodeLeaf((char*)leaf,num);

        PrimRefList::block_iterator_unsafe iter1(current.prims);
        while (iter1) {
          iter1->lower.a &= 0x00FFFFFF;
          iter1++;
        }

        /* insert all triangles */
        PrimRefList::block_iterator_unsafe iter(current.prims);
        for (size_t i=0; i<num; i++) leaf[i].fill(iter,bvh->scene,false);
        assert(!iter);
        
        /* free all primitive blocks */
        while (PrimRefList::item* block = current.prims.take())
          delete block;

        *current.parent = node;
	return n;
      }

      BVH* bvh;
    };

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderSpatialSAH : public Builder
    {
      typedef BVHN<N> BVH;
      BVH* bvh;
      Scene* scene;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVHNBuilderSpatialSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize,
                             const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? defaultPresplitFactor : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          bvh->clear();
          return;
        }
        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderSpatialSAH");
        
        /* create primref list */
        PrimRefList prims;
        PrimInfo pinfo = createPrimRefList<Mesh,1>(scene,prims,bvh->scene->progressInterface);
        
        /* calculate total surface area */
        PrimRefList::iterator iter = prims;
        const size_t threadCount = TaskScheduler::threadCount();
        const float A = (float) parallel_reduce(size_t(0),threadCount,0.0, [&] (const range<size_t>& r) -> double // FIXME: this sum is not deterministic
                                                {
                                                  double A = 0.0f;
                                                  while (PrimRefList::item* block = iter.next()) {
                                                    for (size_t i=0; i<block->size(); i++) 
                                                      A += area(block->at(i).bounds());
                                                  }
                                                  return A;
                                                },std::plus<double>());
        
        /* calculate maximal number of spatial splits per primitive */
        float f = 10.0f;
        iter = prims;
        parallel_reduce(size_t(0),threadCount,size_t(0), [&] (const range<size_t>& r) -> size_t
                        {
                          size_t num = 0;
                          while (PrimRefList::item* block = iter.next()) {
                            for (size_t i=0; i<block->size(); i++) {
                              PrimRef& prim = block->at(i);
                              assert((prim.lower.a & 0xFF000000) == 0);
                              const float nf = ceil(f*pinfo.size()*area(prim.bounds())/A);
                              //const size_t n = 64;
                              //const size_t n = min(ssize_t(127), max(ssize_t(1), ssize_t(nf)));
                              const size_t n = 4+min(ssize_t(127-4), max(ssize_t(1), ssize_t(nf)));
                              num += n;
                              prim.lower.a |= n << 24;
                            }
                          }
                          return num;
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
        bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
        BVHNBuilderSpatial<N>::build(bvh,
                                     splitPrimitive,
                                     CreateListLeaf<N,Primitive>(bvh),
                                     bvh->scene->progressInterface,prims,pinfo,
                                     sahBlockSize,minLeafSize,maxLeafSize,travCost,intCost);
        
        /* clear temporary data for static geometry */
	if (scene->isStatic()) bvh->shrink();
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
      }
    };


    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Primitive>
    struct CreateLeafMB
    {
      typedef BVHN<N> BVH;
      __forceinline CreateLeafMB (BVH* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline std::pair<BBox3fa,BBox3fa> operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc)
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive),BVH::byteNodeAlignment);
        typename BVH::NodeRef node = bvh->encodeLeaf((char*)accel,items);
	BBox3fa bounds0 = empty;
	BBox3fa bounds1 = empty;
        for (size_t i=0; i<items; i++) {
          auto bounds = accel[i].fill_mblur(prims,start,current.prims.end(),bvh->scene,false);
	  bounds0.extend(bounds.first);
	  bounds1.extend(bounds.second);
        }
        *current.parent = node;
	return std::make_pair(bounds0,bounds1);
      }

      BVH* bvh;
      PrimRef* prims;
    };

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderMblurSAH : public Builder
    {
      typedef BVHN<N> BVH;
      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims; 
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;

      BVHNBuilderMblurSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)) {}

      BVHNBuilderMblurSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(mesh->parent->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,2>();

        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }      
        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderMblurSAH");
	    
        //bvh->alloc.init_estimate(numPrimitives*sizeof(PrimRef));
        prims.resize(numPrimitives);
        const PrimInfo pinfo = mesh ? 
          createPrimRefArray<Mesh>(mesh,prims,bvh->scene->progressInterface) : 
          createPrimRefArray<Mesh,2>(scene,prims,bvh->scene->progressInterface);

        /* call BVH builder */
        bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
        BVHNBuilderMblur<N>::build(bvh,CreateLeafMB<N,Primitive>(bvh,prims.data()),bvh->scene->progressInterface,prims.data(),pinfo,
                                   sahBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

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


    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderFastSpatialSAH : public Builder
    {
      typedef BVHN<N> BVH;
      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims0;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float splitFactor;

      BVHNBuilderFastSpatialSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims0(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          splitFactor(scene->device->tri_builder_replication_factor) {}

      BVHNBuilderFastSpatialSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims0(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          splitFactor(scene->device->tri_builder_replication_factor) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numOriginalPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numOriginalPrimitives == 0) {
          prims0.clear();
          bvh->clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderFastSpatialSAH");

        /* create primref array */
        const size_t numSplitPrimitives = max(numOriginalPrimitives,size_t(splitFactor*numOriginalPrimitives));
        prims0.resize(numSplitPrimitives);
        PrimInfo pinfo = mesh ? 
          createPrimRefArray<Mesh>  (mesh ,prims0,bvh->scene->progressInterface) : 
          createPrimRefArray<Mesh,1>(scene,prims0,bvh->scene->progressInterface);

        /* primref array could be smaller due to invalid geometry */
        const size_t numPrimitives = pinfo.size();

        /* calculate total surface area */
        const float A = (float) parallel_reduce(size_t(0),numPrimitives,0.0, [&] (const range<size_t>& r) -> double // FIXME: this sum is not deterministic
                                                {
                                                  double A = 0.0f;
                                                  for (size_t i=r.begin(); i<r.end(); i++)
                                                  {
                                                    PrimRef& prim = prims0[i];
                                                    A += area(prim.bounds());
                                                  }
                                                  return A;
                                                },std::plus<double>());

        const float f = 10.0f;
        const float invA = 1.0f / A;
        /* calculate maximal number of spatial splits per primitive */
        parallel_for( size_t(0), numPrimitives, [&](const range<size_t>& r)
                      {
                        for (size_t i=r.begin(); i<r.end(); i++)
                        {
                          PrimRef& prim = prims0[i];
                          assert((prim.lower.a & 0xFF000000) == 0);
                          const float nf = ceilf(f*pinfo.size()*area(prim.bounds()) * invA);
                          // FIXME: is there a better general heuristic ?
                          size_t n = 4+min(ssize_t(127-4), max(ssize_t(1), ssize_t(nf)));
                          prim.lower.a |= n << 24;              
                        }
                      });
        
        /* function that splits a primitive at some position and dimension */
        auto splitPrimitivePartition = [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) {
          TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF );  
          TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
          const Vec3fa& a = mesh->vertex(tri.v[0]);
          const Vec3fa& b = mesh->vertex(tri.v[1]);
          const Vec3fa& c = mesh->vertex(tri.v[2]);          
          const Vec3fa v[4] = { a,b,c,a };
          const vfloat4 v1(b[dim],c[dim],a[dim],b[dim]);
          const vfloat4 v0(a[dim],b[dim],c[dim],a[dim]);
          const vfloat4 inv_length = 1.0f / (v1-v0);
          splitTriQuadFast<3>(prim,dim,pos,v,inv_length,left_o,right_o);
        };

        auto splitPrimitiveBinning = [&] (SpatialBinInfo<FAST_SPATIAL_BUILDER_NUM_SPATIAL_SPLITS,PrimRef> &binner, 
                                    const PrimRef* const source, const size_t begin, const size_t end, 
                                    const SpatialBinMapping<FAST_SPATIAL_BUILDER_NUM_SPATIAL_SPLITS> &mapping)
          {
            for (size_t i=begin; i<end; i++)
            {
              const PrimRef &prim = source[i];
              const unsigned int splits = prim.geomID() >> 24;
              assert(splits != 1);
              const vint4 bin0 = mapping.bin(prim.bounds().lower);
              const vint4 bin1 = mapping.bin(prim.bounds().upper);

              for (size_t dim=0; dim<3; dim++) 
              {
                if (unlikely(mapping.invalid(dim))) { continue; }
                size_t bin;
                size_t l = bin0[dim];
                size_t r = bin1[dim];

                // same bin optimization
                if (likely(l == r)) 
                {
                  binner.add(dim,l,l,l,prim.bounds());
                  continue;
                }
                const size_t bin_start = bin0[dim];
                const size_t bin_end   = bin1[dim];
                BBox3fa rest = prim.bounds();
                TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF );  
                TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
                const Vec3fa v[4] = { mesh->vertex(tri.v[0]), 
                                      mesh->vertex(tri.v[1]), 
                                      mesh->vertex(tri.v[2]),
                                      mesh->vertex(tri.v[0]) };
                const Vec3fa inv_length[4] = { 
                  Vec3fa(1.0f) / (v[1]-v[0]),
                  Vec3fa(1.0f) / (v[2]-v[1]),
                  Vec3fa(1.0f) / (v[0]-v[2]),
                  Vec3fa(1.0f)
                };
                for (bin=bin_start; bin<bin_end; bin++) 
                {
                  const float pos = mapping.pos(bin+1,dim);
                  BBox3fa left,right;
                  splitTriQuadFast<3>(rest,dim,pos,v,inv_length,left,right);
                  if (unlikely(left.empty())) l++;                
                  binner.extend(dim,bin,left);
                  rest = right;
                }
                if (unlikely(rest.empty())) r--;
                binner.add(dim,l,r,bin,rest);
              }
            }              
          };



        /* call BVH builder */
        bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
        BVHNBuilderFastSpatial<N,FAST_SPATIAL_BUILDER_NUM_SPATIAL_SPLITS>::build(bvh,
                                                                                 splitPrimitivePartition,
                                                                                 splitPrimitiveBinning,
                                                                                 CreateLeafSpatial<N,Primitive>(bvh,prims0.data()),
                                                                                 bvh->scene->progressInterface,
                                                                                 prims0.data(),
                                                                                 numSplitPrimitives,
                                                                                 pinfo,
                                                                                 sahBlockSize,minLeafSize,maxLeafSize,
                                                                                 travCost,intCost);
        
	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          prims0.clear();
          bvh->shrink();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims0.clear();
      }
    };


    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderQuadFastSpatialSAH : public Builder
    {
      typedef BVHN<N> BVH;
      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims0;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float splitFactor;

      BVHNBuilderQuadFastSpatialSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims0(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          splitFactor(scene->device->tri_builder_replication_factor) {}

      BVHNBuilderQuadFastSpatialSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims0(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          splitFactor(scene->device->tri_builder_replication_factor) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numOriginalPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numOriginalPrimitives == 0) {
          prims0.clear();
          bvh->clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderFastSpatialSAH");

        /* create primref array */
        const size_t numSplitPrimitives = max(numOriginalPrimitives,size_t(splitFactor*numOriginalPrimitives));
        prims0.resize(numSplitPrimitives);
        PrimInfo pinfo = mesh ? 
          createPrimRefArray<Mesh>  (mesh ,prims0,bvh->scene->progressInterface) : 
          createPrimRefArray<Mesh,1>(scene,prims0,bvh->scene->progressInterface);

        /* primref array could be smaller due to invalid geometry */
        const size_t numPrimitives = pinfo.size();

        /* calculate total surface area */
        const float A = (float) parallel_reduce(size_t(0),numPrimitives,0.0, [&] (const range<size_t>& r) -> double // FIXME: this sum is not deterministic
                                                {
                                                  double A = 0.0f;
                                                  for (size_t i=r.begin(); i<r.end(); i++)
                                                  {
                                                    PrimRef& prim = prims0[i];
                                                    A += area(prim.bounds());
                                                  }
                                                  return A;
                                                },std::plus<double>());

        const float f = 10.0f;
        const float invA = 1.0f / A;
        /* calculate maximal number of spatial splits per primitive */
        parallel_for( size_t(0), numPrimitives, [&](const range<size_t>& r)
                      {
                        for (size_t i=r.begin(); i<r.end(); i++)
                        {
                          PrimRef& prim = prims0[i];
                          assert((prim.lower.a & 0xFF000000) == 0);
                          const float nf = ceilf(f*pinfo.size()*area(prim.bounds()) * invA);
                          // FIXME: is there a better general heuristic ?
                          size_t n = 4+min(ssize_t(127-4), max(ssize_t(1), ssize_t(nf)));
                          prim.lower.a |= n << 24;              
                        }
                      });
        
        /* function that splits a primitive at some position and dimension */
        auto splitPrimitivePartition = [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) {
          QuadMesh* mesh = (QuadMesh*) scene->get(prim.geomID() & 0x00FFFFFF );  
          QuadMesh::Quad tri = mesh->quad(prim.primID());
          const Vec3fa& a = mesh->vertex(tri.v[0]);
          const Vec3fa& b = mesh->vertex(tri.v[1]);
          const Vec3fa& c = mesh->vertex(tri.v[2]);          
          const Vec3fa& d = mesh->vertex(tri.v[3]);          
          const Vec3fa v[4] = { a,b,c,d };
          const vfloat4 v1(b[dim],c[dim],d[dim],a[dim]);
          const vfloat4 v0(a[dim],b[dim],c[dim],d[dim]);
          const vfloat4 inv_length = 1.0f / (v1-v0);
          splitTriQuadFast<4>(prim,dim,pos,v,inv_length,left_o,right_o);
        };

        auto splitPrimitiveBinning = [&] (SpatialBinInfo<FAST_SPATIAL_BUILDER_NUM_SPATIAL_SPLITS,PrimRef> &binner, 
                                    const PrimRef* const source, const size_t begin, const size_t end, 
                                    const SpatialBinMapping<FAST_SPATIAL_BUILDER_NUM_SPATIAL_SPLITS> &mapping)
          {
            for (size_t i=begin; i<end; i++)
            {
              const PrimRef &prim = source[i];
              const unsigned int splits = prim.geomID() >> 24;
              assert(splits != 1);
              const vint4 bin0 = mapping.bin(prim.bounds().lower);
              const vint4 bin1 = mapping.bin(prim.bounds().upper);

              for (size_t dim=0; dim<3; dim++) 
              {
                if (unlikely(mapping.invalid(dim))) { continue; }
                size_t bin;
                size_t l = bin0[dim];
                size_t r = bin1[dim];

                // same bin optimization
                if (likely(l == r)) 
                {
                  binner.add(dim,l,l,l,prim.bounds());
                  continue;
                }
                const size_t bin_start = bin0[dim];
                const size_t bin_end   = bin1[dim];
                BBox3fa rest = prim.bounds();
                QuadMesh* mesh = (QuadMesh*) scene->get(prim.geomID() & 0x00FFFFFF );  
                QuadMesh::Quad tri = mesh->quad(prim.primID());
                const Vec3fa v[4] = { mesh->vertex(tri.v[0]), 
                                      mesh->vertex(tri.v[1]), 
                                      mesh->vertex(tri.v[2]),
                                      mesh->vertex(tri.v[3]) };
                const Vec3fa inv_length[4] = { 
                  Vec3fa(1.0f) / (v[1]-v[0]),
                  Vec3fa(1.0f) / (v[2]-v[1]),
                  Vec3fa(1.0f) / (v[3]-v[2]),
                  Vec3fa(1.0f) / (v[0]-v[3])
                };
                for (bin=bin_start; bin<bin_end; bin++) 
                {
                  const float pos = mapping.pos(bin+1,dim);
                  BBox3fa left,right;
                  splitTriQuadFast<4>(rest,dim,pos,v,inv_length,left,right);
                  if (unlikely(left.empty())) l++;                
                  binner.extend(dim,bin,left);
                  rest = right;
                }
                if (unlikely(rest.empty())) r--;
                binner.add(dim,l,r,bin,rest);
              }
            }              
          };



        /* call BVH builder */
        bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
        BVHNBuilderFastSpatial<N,FAST_SPATIAL_BUILDER_NUM_SPATIAL_SPLITS>::build(bvh,
                                                                                 splitPrimitivePartition,
                                                                                 splitPrimitiveBinning,
                                                                                 CreateLeafSpatial<N,Primitive>(bvh,prims0.data()),
                                                                                 bvh->scene->progressInterface,
                                                                                 prims0.data(),
                                                                                 numSplitPrimitives,
                                                                                 pinfo,
                                                                                 sahBlockSize,minLeafSize,maxLeafSize,
                                                                                 travCost,intCost);
        
	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          prims0.clear();
          bvh->shrink();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims0.clear();
      }
    };


    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderSweepSAH : public Builder
    {
      typedef BVHN<N> BVH;
      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVHNBuilderSweepSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? defaultPresplitFactor : 1.0f) {}


      BVHNBuilderSweepSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY ) ? defaultPresplitFactor : 1.0f) {}

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

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderSAH");

        /* create primref array */
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
        prims.resize(numSplitPrimitives);
        PrimInfo pinfo = mesh ? 
          createPrimRefArray<Mesh>  (mesh ,prims,bvh->scene->progressInterface) : 
          createPrimRefArray<Mesh,1>(scene,prims,bvh->scene->progressInterface);
        
        /* perform pre-splitting */
        if (presplitFactor > 1.0f) 
          pinfo = presplit<Mesh>(scene, pinfo, prims);
        
        /* call BVH builder */
        bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
        BVHNBuilder<N>::build(bvh,CreateLeaf<N,Primitive>(bvh,prims.data()),bvh->scene->progressInterface,prims.data(),pinfo,sahBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

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

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/


#if defined(EMBREE_GEOMETRY_LINES)
    Builder* BVH4Line4iMeshBuilderSAH     (void* bvh, LineSegments* mesh, size_t mode) { return new BVHNBuilderSAH<4,LineSegments,Line4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Line4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,LineSegments,Line4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Line4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMblurSAH<4,LineSegments,Line4i>((BVH4*)bvh,scene ,4,1.0f,4,inf); }
    Builder* BVH4Line4iMBMeshBuilderSAH  (void* bvh, LineSegments* mesh, size_t mode) { return new BVHNBuilderMblurSAH<4,LineSegments,Line4i>((BVH4*)bvh,mesh ,4,1.0f,4,inf); }
#if defined(__AVX__)
    Builder* BVH8Line4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,LineSegments,Line4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Line4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMblurSAH<8,LineSegments,Line4i>((BVH8*)bvh,scene,4,1.0f,4,inf); }
#endif
#endif

#if defined(EMBREE_GEOMETRY_HAIR)
    Builder* BVH4Bezier1vSceneBuilderSAH   (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,BezierCurves,Bezier1v>((BVH4*)bvh,scene,1,1.0f,1,1,mode); }
    Builder* BVH4Bezier1iSceneBuilderSAH   (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,BezierCurves,Bezier1i>((BVH4*)bvh,scene,1,1.0f,1,1,mode); }
#endif

#if defined(EMBREE_GEOMETRY_TRIANGLES)
    Builder* BVH4Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    
    Builder* BVH4Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }

    Builder* BVH4Triangle4vMBMeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderMblurSAH<4,TriangleMesh,Triangle4vMB>((BVH4*)bvh,mesh ,4,1.0f,4,inf); }
    Builder* BVH4Triangle4vMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMblurSAH<4,TriangleMesh,Triangle4vMB>((BVH4*)bvh,scene,4,1.0f,4,inf); }
    Builder* BVH4Triangle4iMBMeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderMblurSAH<4,TriangleMesh,Triangle4iMB>((BVH4*)bvh,mesh ,4,1.0f,4,inf); }
    Builder* BVH4Triangle4iMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMblurSAH<4,TriangleMesh,Triangle4iMB>((BVH4*)bvh,scene,4,1.0f,4,inf); }

    Builder* BVH4Triangle4SceneBuilderSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSpatialSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSpatialSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSpatialSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }


    Builder* BVH4Triangle4SceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderFastSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderFastSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }


    Builder* BVH4QuantizedTriangle4iSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH8Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4v>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4i>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }

    Builder* BVH8Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4vMBMeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderMblurSAH<8,TriangleMesh,Triangle4vMB>((BVH8*)bvh,mesh ,4,1.0f,4,inf); }
    Builder* BVH8Triangle4vMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMblurSAH<8,TriangleMesh,Triangle4vMB>((BVH8*)bvh,scene,4,1.0f,4,inf); }
    Builder* BVH8Triangle4iMBMeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderMblurSAH<8,TriangleMesh,Triangle4iMB>((BVH8*)bvh,mesh ,4,1.0f,4,inf); }
    Builder* BVH8Triangle4iMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMblurSAH<8,TriangleMesh,Triangle4iMB>((BVH8*)bvh,scene,4,1.0f,4,inf); }
    Builder* BVH8Triangle4SceneBuilderSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSpatialSAH<8,TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8QuantizedTriangle4iSceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,TriangleMesh,Triangle4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

    /* new fast spatial split builder */
    Builder* BVH8Triangle4SceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<8,TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

    /* experimental full sweep builder */
    Builder* BVH8Triangle4SceneBuilderSweepSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSweepSAH<8,TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

#endif
#endif

#if defined(EMBREE_GEOMETRY_QUADS)
    Builder* BVH4Quad4vMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<4,QuadMesh,Quad4v>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4iMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,QuadMesh,Quad4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMblurSAH<4,QuadMesh,Quad4iMB>((BVH4*)bvh,scene ,4,1.0f,4,inf); }
    Builder* BVH4Quad4iMBMeshBuilderSAH  (void* bvh, QuadMesh* mesh, size_t mode) { return new BVHNBuilderMblurSAH<4,QuadMesh,Quad4iMB>((BVH4*)bvh,mesh ,4,1.0f,4,inf); }
    Builder* BVH4QuantizedQuad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,QuadMesh,Quad4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4QuantizedQuad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }

    /* new fast spatial split builder */
    Builder* BVH4Quad4vSceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderQuadFastSpatialSAH<4,QuadMesh,Quad4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }

#if defined(__AVX__)
    Builder* BVH8Quad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,QuadMesh,Quad4v>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Quad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,QuadMesh,Quad4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Quad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMblurSAH<8,QuadMesh,Quad4iMB>((BVH8*)bvh,scene,4,1.0f,4,inf); }
    Builder* BVH8QuantizedQuad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,QuadMesh,Quad4v>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8QuantizedQuad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,QuadMesh,Quad4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Quad4vMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<8,QuadMesh,Quad4v>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }

    /* new fast spatial split builder */
    Builder* BVH8Quad4vSceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderQuadFastSpatialSAH<8,QuadMesh,Quad4v>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

#endif
#endif

#if defined(EMBREE_GEOMETRY_USER)

    Builder* BVH4VirtualSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) {
      int minLeafSize = scene->device->object_accel_min_leaf_size;
      int maxLeafSize = scene->device->object_accel_max_leaf_size;
      return new BVHNBuilderSAH<4,AccelSet,Object>((BVH4*)bvh,scene,4,1.0f,minLeafSize,maxLeafSize,mode);
    }

    Builder* BVH4VirtualMBSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) {
      int minLeafSize = scene->device->object_accel_mb_min_leaf_size;
      int maxLeafSize = scene->device->object_accel_mb_max_leaf_size;
      return new BVHNBuilderMblurSAH<4,AccelSet,Object>((BVH4*)bvh,scene,4,1.0f,minLeafSize,maxLeafSize);
    }
#endif
  }
}
