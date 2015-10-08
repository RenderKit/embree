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
#include "../geometry/triangle.h"
#include "../geometry/trianglev_mb.h"

namespace embree
{
  namespace isa
  {
    //Builder* BVH4Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode = 0);
    Builder* BVH4Triangle4MeshBuilderSAH    (void* bvh, TriangleMesh* mesh, size_t mode = 0);
    Builder* BVH4Triangle4vMBMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode = 0);

    BVH4BuilderInstancing::BVH4BuilderInstancing (BVH4* bvh, Scene* scene) 
      : bvh(bvh), objects(bvh->objects), scene(scene), refs(scene->device), prims(scene->device), 
        nextRef(0)//, numInstancedPrimitives(0), worldBVH(new BVH4(Triangle4::type,scene)), worldBuilder(BVH4Triangle4SceneBuilderSAH(worldBVH,scene,0))  
    {
      //bvh->worldBVH = worldBVH; // BVH4 manages lifetime
    }
    
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

    int slot(int type, int numTimeSteps)
    {
      if (numTimeSteps == 1)
      {
        switch (type) {
        case Geometry::TRIANGLE_MESH: return 0;
        case Geometry::USER_GEOMETRY: break;
        case Geometry::BEZIER_CURVES: break;
        case Geometry::SUBDIV_MESH  : break;
        }
      } else {
        switch (type) {
        case Geometry::TRIANGLE_MESH: return 1;
        case Geometry::USER_GEOMETRY: break;
        case Geometry::BEZIER_CURVES: break;
        case Geometry::SUBDIV_MESH  : break;
        }
      }
      assert(false);
      return 0;
    }
    

    const BBox3fa xfmDeepBounds(const AffineSpace3fa& xfm, const BBox3fa& bounds, BVH4::NodeRef ref, size_t depth)
    {
      if (ref == BVH4::emptyNode) return empty;
      if (depth == 0 || !ref.isNode()) return xfmBounds(xfm,bounds);
      BVH4::Node* node = ref.node();

      BBox3fa box = empty;
      for (size_t i=0; i<BVH4::N; i++)
        box.extend(xfmDeepBounds(xfm,node->bounds(i),node->child(i),depth-1));
      return box;
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
      size_t numPrimitives = 0;
      //numPrimitives += scene->getNumPrimitives<TriangleMesh,1>();
      numPrimitives += scene->instanced1.numTriangles;
      numPrimitives += scene->instanced2.numTriangles;
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
            Geometry* geom = scene->get(objectID);
            
            /* verify if geometry got deleted properly */
            if (geom == nullptr) {
              assert(objectID < objects.size () && objects[objectID] == nullptr);
              assert(objectID < builders.size() && builders[objectID] == nullptr);
              continue;
            }
            
            /* create BVH and builder for new meshes */
            if (objects[objectID] == nullptr) 
            {
              if (geom->numTimeSteps == 1)
              {
                switch (geom->type) {
                case Geometry::TRIANGLE_MESH:
                  objects[objectID] = new BVH4(Triangle4::type,geom->parent);
                  builders[objectID] = BVH4Triangle4MeshBuilderSAH((BVH4*)objects[objectID],(TriangleMesh*)geom);
                  break;
                case Geometry::USER_GEOMETRY: break;
                case Geometry::BEZIER_CURVES: break;
                case Geometry::SUBDIV_MESH  : break;
                default                     : break; 
                }
              } else {
                 switch (geom->type) {
                 case Geometry::TRIANGLE_MESH:
                   objects[objectID] = new BVH4(Triangle4vMB::type,geom->parent);
                   builders[objectID] = BVH4Triangle4vMBMeshBuilderSAH((BVH4*)objects[objectID],(TriangleMesh*)geom);
                   break;
                 case Geometry::USER_GEOMETRY: break;
                 case Geometry::BEZIER_CURVES: break;
                 case Geometry::SUBDIV_MESH  : break;
                 default                     : break;
                 }
              }
            }
          }
        });
      
      /* parallel build of acceleration structures */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) {
          for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
          {
            Geometry* geom = scene->get(objectID);
            if (geom == nullptr) continue;
            Builder* builder = builders[objectID]; 
            if (builder == nullptr) continue;
            if (geom->isModified() && geom->isInstanced()) 
              builder->build(0,0);
          }
        });

      /* create world space instance */
      //worldBuilder->build();
      //if (!worldBVH->bounds.empty())
      //  refs[nextRef++] = BVH4BuilderInstancing::BuildRef(one,worldBVH->bounds,worldBVH->root,-1,-1,0);
      
      /* creates all instances */
      parallel_for(size_t(0), N, [&] (const range<size_t>& r) {
          for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
          {
            Geometry* geom = scene->get(objectID);
            if (geom == nullptr) continue;
            if (!(geom->getType() & Geometry::INSTANCE)) continue;
            GeometryInstance* instance = (GeometryInstance*) geom;
            if (!instance->isEnabled()) continue;
            BVH4* object = objects[instance->geom->id];
            if (object == nullptr) continue;
            if (object->bounds.empty()) continue;
            int s = slot(geom->getType() & ~Geometry::INSTANCE, geom->numTimeSteps);
            refs[nextRef++] = BVH4BuilderInstancing::BuildRef(instance->local2world,object->bounds,object->root,instance->mask,objectID,hash(instance->local2world),s);
          }
        });
      refs.resize(nextRef);

#if 0
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
#endif   
      
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
      
      else if (pinfo.size() == 1) {
        BuildRef* ref = (BuildRef*) prims[0].ID();
        //const BBox3fa bounds = xfmBounds(ref->local2world,ref->localBounds);
        const BBox3fa bounds = xfmDeepBounds(ref->local2world,ref->localBounds,ref->node,2);
        bvh->set(ref->node,bounds,numPrimitives);
      }

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
            new (node) BVH4::TransformNode(ref->local2world,ref->localBounds,ref->node,ref->mask,ref->instID,ref->xfmID,ref->type); // FIXME: rcp should be precalculated somewhere
            *current.parent = BVH4::encodeNode(node);
            //*current.parent = ref->node;
            ((BVH4::NodeRef*)current.parent)->setBarrier();
            return 1;
          },
           [&] (size_t dn) { bvh->scene->progressMonitor(0); },
           prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,4,1,1,1.0f,1.0f);
        
        bvh->set(root,pinfo.geomBounds,numPrimitives);
        bvh->root = collapse(bvh->root);
      }
            
      bvh->alloc.cleanup();
      bvh->postBuild(t0);
    }
    
    void BVH4BuilderInstancing::deleteGeometry(size_t geomID)
    {
      if (geomID >= objects.size()) return;
      delete builders[geomID]; builders[geomID] = nullptr;
      delete objects [geomID]; objects [geomID] = nullptr;
    }

    void BVH4BuilderInstancing::clear()
    {
      for (size_t i=0; i<objects.size(); i++) 
        if (objects[i]) objects[i]->clear();
      
      for (size_t i=0; i<builders.size(); i++) 
	if (builders[i]) builders[i]->clear();
      
      refs.clear();
    }
    
    void BVH4BuilderInstancing::open(size_t numInstancedPrimitives)
    {
      if (refs.size() == 0)
	return;
     
      if (scene->device->benchmark) { std::cout << "BENCHMARK_INSTANCES " << refs.size() << std::endl; }
      if (scene->device->benchmark) { std::cout << "BENCHMARK_INSTANCED_PRIMITIVES " << numInstancedPrimitives << std::endl; }
      
      /* calculate opening size */
      size_t N = 0;
      if      (scene->device->instancing_block_size ) N = numInstancedPrimitives/scene->device->instancing_block_size;
      else if (scene->device->instancing_open_factor) N = scene->device->instancing_open_factor*refs.size();
      N = max(N,scene->device->instancing_open_min);
      N = min(N,scene->device->instancing_open_max);
      refs.reserve(N);

      std::make_heap(refs.begin(),refs.end());
      while (refs.size()+BVH4::N-1 <= N)
      {
        std::pop_heap (refs.begin(),refs.end()); 
        BuildRef ref = refs.back();
        refs.pop_back();    
        
        if (ref.node.isNode()) 
        {
          BVH4::Node* node = ref.node.node();
          for (size_t i=0; i<BVH4::N; i++) {
            if (node->child(i) == BVH4::emptyNode) continue;
            refs.push_back(BuildRef(ref.local2world,node->bounds(i),node->child(i),ref.mask,ref.instID,ref.xfmID,ref.type));
            std::push_heap (refs.begin(),refs.end()); 
          }
        } 
        /*else if (ref.node.isNodeMB()) 
        {
          BVH4::NodeMB* node = ref.node.nodeMB();
          for (size_t i=0; i<BVH4::N; i++) {
            if (node->child(i) == BVH4::emptyNode) continue;
            refs.push_back(BuildRef(ref.local2world,node->bounds(i),node->child(i),ref.mask,ref.instID,ref.xfmID,ref.type));
            std::push_heap (refs.begin(),refs.end()); 
          }
        }*/
        else {
          refs.push_back(ref);
          break;
        }
      }

      if (scene->device->benchmark) { std::cout << "BENCHMARK_OPENED_INSTANCES " << refs.size() << std::endl; }
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
      return new BVH4BuilderInstancing((BVH4*)bvh,scene);
    }
  }
}
