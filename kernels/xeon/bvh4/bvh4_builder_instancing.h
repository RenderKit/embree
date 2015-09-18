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

#pragma once

#include "bvh4.h"
#include "../../common/scene_triangle_mesh.h"

namespace embree
{
  namespace isa
  {
    class BVH4BuilderInstancing : public Builder
    {
      ALIGNED_CLASS;
    public:

      struct BuildRef
    {
    public:
      __forceinline BuildRef () {}
      
      __forceinline BuildRef (const AffineSpace3fa& local2world, const BBox3fa& localBounds_in, BVH4::NodeRef node, int instID, int xfmID) 
        : local2world(local2world), localBounds(localBounds_in), node(node), instID(instID), xfmID(xfmID)
      {
        if (node.isLeaf())
          localBounds.lower.w = 0.0f;
        else {
          const BBox3fa worldBounds = xfmBounds(local2world,localBounds);
          localBounds.lower.w = area(worldBounds);
        }
      }
      
      __forceinline BBox3fa worldBounds() const {
        return xfmBounds(local2world,localBounds);
      }
      
      friend bool operator< (const BuildRef& a, const BuildRef& b) {
        return a.localBounds.lower.w < b.localBounds.lower.w;
      }
      
    public:
      AffineSpace3fa local2world;
      BBox3fa localBounds;
      BVH4::NodeRef node;
      int instID;
      int xfmID;
    };
      
      /*! Constructor. */
      BVH4BuilderInstancing (BVH4* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel);
      
      /*! Destructor */
      ~BVH4BuilderInstancing ();
      
      /*! builder entry point */
      void build(size_t threadIndex, size_t threadCount);

      void clear();

      void open(size_t numPrimitives);

      BVH4::NodeRef collapse(BVH4::NodeRef& node);
      
    public:
      BVH4* bvh;
      std::vector<BVH4*>& objects;
      std::vector<Builder*> builders;
      BVH4* worldBVH;
      Ref<Builder> worldBuilder;

    public:
      Scene* scene;
      createTriangleMeshAccelTy createTriangleMeshAccel;
      
      mvector<BuildRef> refs;
      mvector<PrimRef> prims;
      AtomicCounter nextRef;
      AtomicCounter numInstancedPrimitives; 
    };
  }
}
