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
#include "bvh4_statistics.h"

#include "builders_new/primrefgen.h"
#include "builders_new/bvh_builder.h"

#include "geometry/bezier1v.h"
#include "geometry/bezier1i.h"
#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"

#define PROFILE 

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

    template<typename Primitive>
    struct CreateLeaf
    {
      __forceinline CreateLeaf (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline int operator() (const BuildRecord<BVH4::NodeRef>& current, PrimRef* prims, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return 1;
      }

      BVH4* bvh;
    };
   
    template<typename Mesh, typename Primitive>
    struct BVH4BuilderBinnedSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      Mesh* mesh;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;

      BVH4BuilderBinnedSAH (BVH4* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)) {}

      BVH4BuilderBinnedSAH (BVH4* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(NULL), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH4::maxLeafBlocks)) {}

      void build(size_t, size_t) 
      {
        /* start measurement */
        double t0 = 0.0f;
        if (g_verbose >= 1) t0 = getSeconds();

        /* calculate scene size */
        const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        
        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
      
        /* verbose mode */
        if (g_verbose >= 1)
          std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderBinnedSAH ... " << std::flush;

#if defined(PROFILE)
      
        double dt_min = pos_inf;
        double dt_avg = 0.0f;
        double dt_max = neg_inf;
        for (size_t i=0; i<20; i++) 
        {
          double t0 = getSeconds();
#endif
          
          /* reserve data */
          bvh->alloc2.init(numPrimitives*sizeof(PrimRef),numPrimitives*sizeof(BVH4::Node)); 
          prims.resize(numPrimitives);
          
          /* build BVH */
          PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
          BVH4::NodeRef root = bvh_builder_binned_sah_internal<BVH4::NodeRef>(
            CreateAlloc(bvh),CreateBVH4Node(bvh),CreateLeaf<Primitive>(bvh),
            prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize);

          bvh->set(root,pinfo.geomBounds,pinfo.size());
          
#if defined(PROFILE)
          double dt = getSeconds()-t0;
          dt_min = min(dt_min,dt);
          dt_avg = dt_avg + dt;
          dt_max = max(dt_max,dt);
        }
        dt_avg /= double(20);
        
        std::cout << "[DONE]" << std::endl;
        std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
        std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
        std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
#endif
        
        /* stop measurement */
        double dt = 0.0f;
        if (g_verbose >= 1) dt = getSeconds()-t0;
        
        /* verbose mode */
        if (g_verbose >= 1) {
          std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
          std::cout << "  bvh4::alloc : "; bvh->alloc.print_statistics();
          std::cout << "  bvh4::alloc2: "; bvh->alloc2.print_statistics();
        }
        if (g_verbose >= 2)
          std::cout << BVH4Statistics(bvh).str();

        /* benchmark mode */
        if (g_benchmark) {
          BVH4Statistics stat(bvh);
          std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
        }
      }
    };
    
    /* entry functions for the scene builder */
    Builder* BVH4Bezier1vSceneBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<BezierCurves,Bezier1v>((BVH4*)bvh,scene,1,1,1.0f,1,1); }
    Builder* BVH4Bezier1iSceneBuilderBinnedSAH   (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<BezierCurves,Bezier1i>((BVH4*)bvh,scene,1,1,1.0f,1,1); }
    Builder* BVH4Triangle1SceneBuilderBinnedSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1>((BVH4*)bvh,scene,1,1,1.0f,2,inf); }
    Builder* BVH4Triangle4SceneBuilderBinnedSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,4,1.0f,4,inf); }
#if defined(__AVX__)
    Builder* BVH4Triangle8SceneBuilderBinnedSAH  (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle8>((BVH4*)bvh,scene,8,4,1.0f,8,inf); }
#endif
    Builder* BVH4Triangle1vSceneBuilderBinnedSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1v>((BVH4*)bvh,scene,1,1,1.0f,2,inf); }
    Builder* BVH4Triangle4vSceneBuilderBinnedSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4v>((BVH4*)bvh,scene,2,2,1.0f,4,inf); }
    Builder* BVH4Triangle4iSceneBuilderBinnedSAH (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4i>((BVH4*)bvh,scene,2,2,1.0f,4,inf); }

    /* entry functions for the mesh builders */
    Builder* BVH4Triangle1MeshBuilderBinnedSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1>((BVH4*)bvh,mesh,1,1,1.0f,2,inf); }
    Builder* BVH4Triangle4MeshBuilderBinnedSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4>((BVH4*)bvh,mesh,4,4,1.0f,4,inf); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderBinnedSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle8>((BVH4*)bvh,mesh,8,4,1.0f,8,inf); }
#endif
    Builder* BVH4Triangle1vMeshBuilderBinnedSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle1v>((BVH4*)bvh,mesh,1,1,1.0f,2,inf); }
    Builder* BVH4Triangle4vMeshBuilderBinnedSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,2,2,1.0f,4,inf); }
    Builder* BVH4Triangle4iMeshBuilderBinnedSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVH4BuilderBinnedSAH<TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,2,2,1.0f,4,inf); }
  }
}
