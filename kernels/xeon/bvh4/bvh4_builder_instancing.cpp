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

#include "bvh4_builder_instancing.h"
#include "bvh4_statistics.h"
#include "../builders/bvh_builder_sah.h"
#include "../geometry/triangle4.h"

namespace embree
{
  namespace isa
  {
    BVH4BuilderInstancing::BVH4BuilderInstancing (BVH4* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel) 
      : bvh(bvh), objects(bvh->objects), scene(scene), createTriangleMeshAccel(createTriangleMeshAccel), refs(scene->device), prims(scene->device), 
        nextRef(0), numInstancedPrimitives(0) {}
    
    BVH4BuilderInstancing::~BVH4BuilderInstancing ()
    {
      for (size_t i=0; i<builders.size(); i++) 
	delete builders[i];
    }

    int hash(const AffineSpace3fa& space)
    {
      int xfmID = 0;
      for (size_t i=0; i<12; i++)
        xfmID ^= 0x12F576E1*i*((int*)&space)[i];
      return xfmID;
    }
    
    void BVH4BuilderInstancing::build(size_t threadIndex, size_t threadCount) 
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
      
      double t0 = bvh->preBuild(TOSTRING(isa) "::BVH4BuilderInstancing");
      
      /* resize object array if scene got larger */
      if (objects.size()  < N) objects.resize(N);
      if (builders.size() < N) builders.resize(N);
      if (refs.size()     < N) refs.resize(N);
      nextRef = 0;
      
      /* creation of acceleration structures */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) {
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
      
      numInstancedPrimitives = 0;
      
      /* parallel build of acceleration structures */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) {
          for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
          {
            /* ignore if no triangle mesh or not enabled */
            TriangleMesh* mesh = scene->getTriangleMeshSafe(objectID);
            if (mesh == nullptr || mesh->numTimeSteps != 1) 
              continue;
            
            BVH4*    object  = objects [objectID]; assert(object);
            Builder* builder = builders[objectID]; assert(builder);
            
            /* build object if it got modified */
            if (mesh->isModified() && mesh->isUsed()) 
              builder->build(0,0);
            
            /* create build primitive */
            if (!object->bounds.empty() && mesh->isEnabled()) {
              refs[nextRef++] = BVH4BuilderInstancing::BuildRef(one,object->bounds,object->root,-1,0);
              numInstancedPrimitives += mesh->size();
            }
          }
        });
      
      /* creates all instances */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) {
          for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
          {
            /* ignore if no triangle mesh or not enabled */
            Geometry* geom = scene->get(objectID);
            if (geom->getType() != (Geometry::TRIANGLE_MESH | Geometry::INSTANCE))
              continue;
            
            GeometryInstance* instance = (GeometryInstance*) geom;
            if (!instance->isEnabled() || instance->numTimeSteps != 1) 
              continue;
            
            BVH4*    object  = objects [instance->geom->id]; assert(object);
            
            /* create build primitive */
            if (!object->bounds.empty()) {
              refs[nextRef++] = BVH4BuilderInstancing::BuildRef(instance->local2world,object->bounds,object->root,objectID,hash(instance->local2world));
              numInstancedPrimitives += instance->geom->size();
            }
          }
        });
      refs.resize(nextRef);

      /* compute transform IDs */
      std::sort(refs.begin(),refs.end(), [] (const BuildRef& ref0, const BuildRef& ref1) { return ref0.xfmID < ref1.xfmID; });
      
      int lastXfmID = 0;
      AffineSpace3fa lastXfm = one;
      for (size_t i=0; i<refs.size(); i++) {
        if (refs[i].local2world != lastXfm) {
          lastXfmID++;
          lastXfm = refs[i].local2world;
        }
        refs[i].xfmID = lastXfmID;
      }
        
      
      /* fast path for single geometry scenes */
      /*if (nextRef == 1) { 
        bvh->set(refs[0].node,refs[0].bounds(),numPrimitives);
        return;
        }*/
      
      /* open all large nodes */
      
      open(numPrimitives); 
      
      /* fast path for small geometries */
      /*if (refs.size() == 1) { 
        bvh->set(refs[0].node,refs[0].bounds(),numPrimitives);
        return;
        }*/
      
      /* compute PrimRefs */
      prims.resize(refs.size());
      const PrimInfo pinfo = parallel_reduce(size_t(0), refs.size(), size_t(1024), PrimInfo(empty), [&] (const range<size_t>& r) -> PrimInfo {
          PrimInfo pinfo(empty);
          for (size_t i=r.begin(); i<r.end(); i++) 
          {
            const BBox3fa bounds = refs[i].worldBounds();
            pinfo.add(bounds);
            prims[i] = PrimRef(bounds,(size_t)&refs[i]);
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
            BuildRef* ref = (BuildRef*) prims[current.prims.begin()].ID();
            BVH4::TransformNode* node = (BVH4::TransformNode*) alloc->alloc0.malloc(sizeof(BVH4::TransformNode)); 
            new (node) BVH4::TransformNode(ref->local2world,ref->localBounds,ref->node,ref->instID,ref->xfmID); // FIXME: rcp should be precalculated somewhere
            *current.parent = BVH4::encodeNode(node);
            //*current.parent = ref->node;
            ((BVH4::NodeRef*)current.parent)->setBarrier();
            return 1;
          },
           [&] (size_t dn) { bvh->scene->progressMonitor(0); },
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,4,1,1,1.0f,1.0f);
        
        bvh->set(root,pinfo.geomBounds,numPrimitives);
      }
      
      bvh->root = collapse(bvh->root);
      
      bvh->alloc.cleanup();
      bvh->postBuild(t0);
    }
    
    void BVH4BuilderInstancing::clear()
    {
      for (size_t i=0; i<objects.size(); i++) 
        if (objects[i]) objects[i]->clear();
      
      for (size_t i=0; i<builders.size(); i++) 
	if (builders[i]) builders[i]->clear();
      
      refs.clear();
    }
    
    void set_primID(BVH4::NodeRef node, unsigned primID)
    {
      if (node.isLeaf())
      {
        size_t num;
        Triangle4* prims = (Triangle4*) node.leaf(num);
        for (size_t i=0; i<num; i++)
          for (size_t j=0; j<4; j++)
            if (prims[i].geomIDs[j] != -1) 
              prims[i].primIDs[j] = primID;
      }
      else 
      {
        BVH4::Node* n = node.node();
        for (size_t c=0; c<BVH4::N; c++)
          set_primID(n->child(c),primID);
      }
    }
    
    void BVH4BuilderInstancing::open(size_t numPrimitives)
    {
      if (refs.size() == 0)
	return;
      
      //size_t N = 0;
      //size_t N = numInstancedPrimitives/2000;
      size_t N = numInstancedPrimitives/100;
      //size_t N = numInstancedPrimitives;
      
      refs.reserve(N);
      
      std::make_heap(refs.begin(),refs.end());
      while (refs.size()+3 <= N)
      {
        std::pop_heap (refs.begin(),refs.end()); 
        BVH4::NodeRef ref = refs.back().node;
        const AffineSpace3fa local2world = refs.back().local2world;
        const int instID = refs.back().instID;
        const int xfmID = refs.back().xfmID;
        if (ref.isLeaf()) break;
        refs.pop_back();    
        
        BVH4::Node* node = ref.node();
        for (size_t i=0; i<BVH4::N; i++) {
          if (node->child(i) == BVH4::emptyNode) continue;
          refs.push_back(BuildRef(local2world,node->bounds(i),node->child(i),instID,xfmID));
          std::push_heap (refs.begin(),refs.end()); 
        }
      }
      
      //for (size_t i=0; i<refs.size(); i++)
      //  set_primID((BVH4::NodeRef) refs[i].node, i);
    }
    
    BVH4::NodeRef BVH4BuilderInstancing::collapse(BVH4::NodeRef& node)
    {
      if (node.isBarrier()) {
        node.clearBarrier();
        return node;
      }
      
      assert(node.isNode());
      BVH4::Node* n = node.node();
      BVH4::TransformNode* first = nullptr;
      for (size_t c=0; c<BVH4::N; c++) {
        if (n->child(c) == BVH4::emptyNode) continue;
        BVH4::NodeRef child = n->child(c) = collapse(n->child(c));
        if (child.isTransformNode()) first = child.transformNode();
      }
      
      bool allEqual = true;
      for (size_t c=0; c<BVH4::N; c++)  
      {
        BVH4::NodeRef child = n->child(c);
        if (child == BVH4::emptyNode) continue;
        
        if (!child.isTransformNode()) {
          allEqual = false;
          break;
        }
        
        if (child.transformNode()->world2local != first->world2local) {
          allEqual = false;
          break;
        }
        
        if (child.transformNode()->instID != first->instID) {
          allEqual = false;
          break;
        }
      }
      
      if (!allEqual) 
        return node;
      
      BBox3fa bounds = empty;
      for (size_t c=0; c<BVH4::N; c++) {
        if (n->child(c) == BVH4::emptyNode) continue;
        BVH4::TransformNode* child = n->child(c).transformNode();
        const BBox3fa cbounds = child->localBounds;
        n->set(c,cbounds,child->child);
        bounds.extend(cbounds);
      }
      first->localBounds = bounds;
      first->child = node;
      return BVH4::encodeNode(first);
    }
    
    Builder* BVH4BuilderInstancingSAH (void* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel) {
      return new BVH4BuilderInstancing((BVH4*)bvh,scene,createTriangleMeshAccel);
    }
  }
}
