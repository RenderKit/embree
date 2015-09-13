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

#include "bvh4_builder_twolevel.h"
#include "bvh4_statistics.h"
#include "../builders/bvh_builder_sah.h"

#define PROFILE 0
#define MAX_OPEN_SIZE 10000

namespace embree
{
  namespace isa
  {
    BVH4BuilderTwoLevel::BVH4BuilderTwoLevel (BVH4* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel) 
      : bvh(bvh), objects(bvh->objects), scene(scene), createTriangleMeshAccel(createTriangleMeshAccel), refs(scene->device), prims(scene->device) {}
    
    BVH4BuilderTwoLevel::~BVH4BuilderTwoLevel ()
    {
      for (size_t i=0; i<builders.size(); i++) 
	delete builders[i];
    }

    void BVH4BuilderTwoLevel::build(size_t threadIndex, size_t threadCount) 
    {
      /* delete some objects */
      size_t N = scene->size();
      if (N < objects.size()) {
        parallel_for(N, objects.size(), [&] (const range<size_t>& r) {
            for (size_t i=r.begin(); i<r.end(); i++) {
              delete builders[i]; builders[i] = nullptr;
              delete objects[i]; objects[i] = nullptr;
            }
          });
      }

      /* reset memory allocator */
      bvh->alloc.reset();
      
      /* skip build for empty scene */
      const size_t numPrimitives = scene->getNumPrimitives<TriangleMesh,1>();
      if (numPrimitives == 0) {
        prims.resize(0);
        bvh->set(BVH4::emptyNode,empty,0);
        return;
      }

      double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4BuilderTwoLevel");

#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
          
      /* resize object array if scene got larger */
      if (objects.size()  < N) objects.resize(N);
      if (builders.size() < N) builders.resize(N);
      if (refs.size()     < N) refs.resize(N);
      nextRef = 0;
      
      /* create of acceleration structures */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) 
      {
        for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
        {
          TriangleMesh* mesh = scene->getTriangleMeshSafe(objectID);
          
          /* verify meshes got deleted properly */
          if (mesh == nullptr || mesh->numTimeSteps != 1) {
            assert(objectID < objects.size () && objects[objectID] == nullptr);
            assert(objectID < builders.size() && builders[objectID] == nullptr);
            continue;
          }
          
          /* delete BVH and builder for meshes that are scheduled for deletion */
          if (mesh->isErasing()) {
            delete builders[objectID]; builders[objectID] = nullptr;
            delete objects [objectID]; objects [objectID] = nullptr;
            continue;
          }
          
          /* create BVH and builder for new meshes */
          if (objects[objectID] == nullptr)
            createTriangleMeshAccel(mesh,(AccelData*&)objects[objectID],builders[objectID]);
        }
      });

      /* parallel build of acceleration structures */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) 
      {
        for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
        {
          /* ignore if no triangle mesh or not enabled */
          TriangleMesh* mesh = scene->getTriangleMeshSafe(objectID);
          if (mesh == nullptr || !mesh->isEnabled() || mesh->numTimeSteps != 1) 
            continue;
        
          BVH4*    object  = objects [objectID]; assert(object);
          Builder* builder = builders[objectID]; assert(builder);
          
          /* build object if it got modified */
#if !PROFILE 
          if (mesh->isModified()) 
#endif
            builder->build(0,0);
          
          /* create build primitive */
          if (!object->bounds.empty())
            refs[nextRef++] = BVH4BuilderTwoLevel::BuildRef(object->bounds,object->root);
        }
      });
      
      /* fast path for single geometry scenes */
      if (nextRef == 1) { 
        bvh->set(refs[0].node,refs[0].bounds(),numPrimitives);
        return;
      }

      /* open all large nodes */
      refs.resize(nextRef);
      open_sequential(numPrimitives); 
      
      /* fast path for small geometries */
      if (refs.size() == 1) { 
        bvh->set(refs[0].node,refs[0].bounds(),numPrimitives);
        return;
      }

      /* compute PrimRefs */
      prims.resize(refs.size());
      const PrimInfo pinfo = parallel_reduce(size_t(0), refs.size(), size_t(1024), PrimInfo(empty), [&] (const range<size_t>& r) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t i=r.begin(); i<r.end(); i++) {
          pinfo.add(refs[i].bounds());
          prims[i] = PrimRef(refs[i].bounds(),(size_t)refs[i].node);
        }
        return pinfo;
      }, [] (const PrimInfo& a, const PrimInfo& b) { return PrimInfo::merge(a,b); });

      /* skip if all objects where empty */
      if (pinfo.size() == 0)
        bvh->set(BVH4::emptyNode,empty,0);

      /* otherwise build toplevel hierarchy */
      else
      {
        BVH4::NodeRef root;
        BVHBuilderBinnedSAH::build<BVH4::NodeRef>
          (root,
           [&] { return bvh->alloc.threadLocal2(); },
           [&] (const isa::BVHBuilderBinnedSAH::BuildRecord& current, BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, FastAllocator::ThreadLocal2* alloc) -> int
           {
             BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
             for (size_t i=0; i<N; i++) {
               node->set(i,children[i].pinfo.geomBounds);
               children[i].parent = (size_t*)&node->child(i);
             }
             *current.parent = bvh->encodeNode(node);
             return 0;
           },
           [&] (const BVHBuilderBinnedSAH::BuildRecord& current, FastAllocator::ThreadLocal2* alloc) -> int
           {
             assert(current.prims.size() == 1);
             *current.parent = (BVH4::NodeRef) prims[current.prims.begin()].ID();
             return 1;
           },
           [&] (size_t dn) { bvh->scene->progressMonitor(0); },
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,1,1.0f,1.0f);
        
        bvh->set(root,pinfo.geomBounds,numPrimitives);
      }

#if PROFILE
      }); 
#endif

      bvh->alloc.cleanup();
      bvh->postBuild(t0);
    }

    void BVH4BuilderTwoLevel::clear()
    {
      for (size_t i=0; i<objects.size(); i++) 
        if (objects[i]) objects[i]->clear();

      for (size_t i=0; i<builders.size(); i++) 
	if (builders[i]) builders[i]->clear();

      refs.clear();
    }

    void BVH4BuilderTwoLevel::open_sequential(size_t numPrimitives)
    {
      if (refs.size() == 0)
	return;

      size_t N = min(numPrimitives/200,size_t(MAX_OPEN_SIZE));
      refs.reserve(N);
      
      std::make_heap(refs.begin(),refs.end());
      while (refs.size()+3 <= N)
      {
        std::pop_heap (refs.begin(),refs.end()); 
        BVH4::NodeRef ref = refs.back().node;
        if (ref.isLeaf()) break;
        refs.pop_back();    
        
        BVH4::Node* node = ref.node();
        for (size_t i=0; i<4; i++) {
          if (node->child(i) == BVH4::emptyNode) continue;
          refs.push_back(BuildRef(node->bounds(i),node->child(i)));
          std::push_heap (refs.begin(),refs.end()); 
        }
      }
    }
    
    Builder* BVH4BuilderTwoLevelSAH (void* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel) {
      return new BVH4BuilderTwoLevel((BVH4*)bvh,scene,createTriangleMeshAccel);
    }
  }
}
