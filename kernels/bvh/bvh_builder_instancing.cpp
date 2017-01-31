// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "bvh_builder_instancing.h"
#include "bvh_statistics.h"
#include "../builders/bvh_builder_sah.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/quadi_mb.h"

namespace embree
{
  namespace isa
  {
    //Builder* BVH4Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode = 0);
    Builder* BVH4Triangle4MeshBuilderSAH    (void* bvh, TriangleMesh* mesh, size_t mode = 0);
    //Builder* BVH4Triangle4vMBMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode = 0);

    Builder* BVH4Quad4vMeshBuilderSAH        (void* bvh, QuadMesh* mesh,     size_t mode = 0);
    //Builder* BVH4Quad4iMBMeshBuilderSAH     (void* bvh, QuadMesh* mesh,     size_t mode = 0);

    template<int N>
    BVHNBuilderInstancing<N>::BVHNBuilderInstancing (BVH* bvh, Scene* scene)
      : bvh(bvh), objects(bvh->objects), scene(scene), refs(scene->device), prims(scene->device), nextRef(0) {}
    
    template<int N>
    BVHNBuilderInstancing<N>::~BVHNBuilderInstancing ()
    {
      for (size_t i=0; i<builders.size(); i++) 
	delete builders[i];
    }

    int hash(const Vec3fa& vec)
    {
      int h = 0;
      for (size_t i=0; i<3; i++)
        h ^= 0x42F276E1*i*((int*)&vec)[i];
      return h;
    }

    int hash(const AffineSpace3fa& space)
    {
      int h = 0;
      h ^= 0xF2FF7631*hash(space.l.vx);
      h ^= 0xF2FF7731*hash(space.l.vy);
      h ^= 0xF2FF7831*hash(space.l.vz);
      h ^= 0xF2FF7931*hash(space.p);
      return h;
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
    
    template<int N>
    const BBox3fa xfmDeepBounds(const AffineSpace3fa& xfm, const BBox3fa& bounds, typename BVHN<N>::NodeRef ref, size_t depth)
    {
      if (ref == BVHN<N>::emptyNode) return empty;
      if (depth == 0 || !ref.isAlignedNode()) return xfmBounds(xfm,bounds);
      typename BVHN<N>::AlignedNode* node = ref.alignedNode();

      BBox3fa box = empty;
      for (size_t i=0; i<N; i++)
        box.extend(xfmDeepBounds<N>(xfm,node->bounds(i),node->child(i),depth-1));
      return box;
    }

    template<int N>
    void BVHNBuilderInstancing<N>::build(size_t threadIndex, size_t threadCount)
    {
      /* delete some objects */
      size_t num = scene->size();
      if (num < objects.size()) {
        parallel_for(num, objects.size(), [&] (const range<size_t>& r) {
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
      //numPrimitives += scene->getNumPrimitives<TriangleMesh,false>();
      numPrimitives += scene->instanced.numTriangles;
      numPrimitives += scene->instancedMB.numTriangles;
      if (numPrimitives == 0) {
        prims.resize(0);
        bvh->set(BVH::emptyNode,empty,0);
        return;
      }
      
      double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderInstancing");
      
      /* resize object array if scene got larger */
      if (objects.size()  < num) objects.resize(num);
      if (builders.size() < num) builders.resize(num);
      if (refs.size()     < num) refs.resize(num);
      nextRef.store(0);
      
      /* creation of acceleration structures */
      parallel_for(size_t(0), num, [&] (const range<size_t>& r) {
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
#if defined(EMBREE_GEOMETRY_TRIANGLES)
                case Geometry::TRIANGLE_MESH:
                  //objects[objectID] = new BVH4(Triangle4::type,geom->parent); // FIXME: enable
                  builders[objectID] = BVH4Triangle4MeshBuilderSAH((BVH4*)objects[objectID],(TriangleMesh*)geom);
                  break;
#endif
#if defined(EMBREE_GEOMETRY_QUADS)
                case Geometry::QUAD_MESH:
                  //objects[objectID] = new BVH4(Quad4v::type,geom->parent); // FIXME: enable
                  builders[objectID] = BVH4Quad4vMeshBuilderSAH((BVH4*)objects[objectID],(QuadMesh*)geom);
                  break;
#endif

                case Geometry::USER_GEOMETRY: break;
                case Geometry::BEZIER_CURVES: break;
                case Geometry::SUBDIV_MESH  : break;
                default                     : break; 
                }
              } else {
                switch (geom->type) {
/*#if defined(EMBREE_GEOMETRY_TRIANGLES)
                 case Geometry::TRIANGLE_MESH:
                   objects[objectID] = new BVH4(Triangle4vMB::type,geom->parent);
                   builders[objectID] = BVH4Triangle4vMBMeshBuilderSAH((BVH4*)objects[objectID],(TriangleMesh*)geom);
                   break;
#endif

#if defined(EMBREE_GEOMETRY_QUADS)
                 case Geometry::QUAD_MESH:
                   objects[objectID] = new BVH4(Quad4iMB::type,geom->parent);
                   builders[objectID] = BVH4Quad4iMBMeshBuilderSAH((BVH4*)objects[objectID],(QuadMesh*)geom);
                   break;
                   #endif*/
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
      parallel_for(size_t(0), num, [&] (const range<size_t>& r) {
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

      /* creates all instances */
      parallel_for(size_t(0), num, [&] (const range<size_t>& r) {
          for (size_t objectID=r.begin(); objectID<r.end(); objectID++)
          {
            Geometry* geom = scene->get(objectID);
            if (geom == nullptr) continue;
            if (!(geom->getType() & Geometry::INSTANCE)) continue;
            GeometryInstance* instance = (GeometryInstance*) geom;
            if (!instance->isEnabled()) continue;
            BVH* object = objects[instance->geom->id];
            if (object == nullptr) continue;
            if (object->getBounds().empty()) continue;
            int s = slot(geom->getType() & ~Geometry::INSTANCE, geom->numTimeSteps);
            refs[nextRef++] = BVHNBuilderInstancing::BuildRef(instance->local2world,object->getBounds(),object->root,instance->mask,unsigned(objectID),hash(instance->local2world),s);
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
        bvh->set(BVH::emptyNode,empty,0);
      
      else if (pinfo.size() == 1) {
        BuildRef* ref = (BuildRef*) prims[0].ID();
        //const BBox3fa bounds = xfmBounds(ref->local2world,ref->localBounds);
        const BBox3fa bounds = xfmDeepBounds<N>(ref->local2world,ref->localBounds,ref->node,2);
        bvh->set(ref->node,LBBox3fa(bounds),numPrimitives);
      }
      
      /* otherwise build toplevel hierarchy */
      else
      {
        NodeRef root;
        BVHBuilderBinnedSAH::build<NodeRef>
          (root,
           [&] { return bvh->alloc.threadLocal2(); },
           [&] (const isa::BVHBuilderBinnedSAH::BuildRecord& current, BVHBuilderBinnedSAH::BuildRecord* children, const size_t num, FastAllocator::ThreadLocal2* alloc) -> int
          {
            AlignedNode* node = (AlignedNode*) alloc->alloc0->malloc(sizeof(AlignedNode)); node->clear();
            for (size_t i=0; i<num; i++) {
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
            TransformNode* node = (TransformNode*) alloc->alloc0->malloc(sizeof(TransformNode));
            new (node) TransformNode(ref->local2world,ref->localBounds,ref->node,ref->mask,ref->instID,ref->xfmID,ref->type); // FIXME: rcp should be precalculated somewhere
            *current.parent = BVH::encodeNode(node);
            //*current.parent = ref->node;
            ((NodeRef*)current.parent)->setBarrier();
            return 1;
          },
           [&] (size_t dn) { bvh->scene->progressMonitor(0); },
           prims.data(),pinfo,N,BVH::maxBuildDepthLeaf,4,1,1,1.0f,1.0f,DEFAULT_SINGLE_THREAD_THRESHOLD);
        
        bvh->set(root,LBBox3fa(pinfo.geomBounds),numPrimitives);
        numCollapsedTransformNodes = refs.size();
        //bvh->root = collapse(bvh->root);
        //if (scene->device->verbosity(1))
        //  std::cout << "collapsing from " << refs.size() << " to " << numCollapsedTransformNodes << " minimally possible " << nextRef << std::endl;
      }
            
      bvh->alloc.cleanup();
      bvh->postBuild(t0);
    }
    
    template<int N>
    void BVHNBuilderInstancing<N>::deleteGeometry(size_t geomID)
    {
      if (geomID >= objects.size()) return;
      delete builders[geomID]; builders[geomID] = nullptr;
      delete objects [geomID]; objects [geomID] = nullptr;
    }

    template<int N>
    void BVHNBuilderInstancing<N>::clear()
    {
      for (size_t i=0; i<objects.size(); i++) 
        if (objects[i]) objects[i]->clear();
      
      for (size_t i=0; i<builders.size(); i++) 
	if (builders[i]) builders[i]->clear();
      
      refs.clear();
    }
    
    Builder* BVH4BuilderInstancingTriangleMeshSAH (void* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel) {
      return new BVHNBuilderInstancing<4>((BVH4*)bvh,scene);
    }

    Builder* BVH4BuilderInstancingQuadMeshSAH (void* bvh, Scene* scene, const createQuadMeshAccelTy createQuadMeshAccel) {
      return new BVHNBuilderInstancing<4>((BVH4*)bvh,scene);
    }

  }
}
