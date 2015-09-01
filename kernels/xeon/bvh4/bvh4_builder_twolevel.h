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
    class BVH4BuilderTwoLevel : public Builder
    {
      ALIGNED_CLASS;
    public:

      struct BuildRef
    {
    public:
      __forceinline BuildRef () {}
      
      __forceinline BuildRef (const BBox3fa& bounds, BVH4::NodeRef node) 
        : lower(bounds.lower), upper(bounds.upper), node(node)
      {
        if (node.isLeaf())
          lower.w = 0.0f;
        else
          lower.w = area(this->bounds());
      }
      
      __forceinline BBox3fa bounds () const {
        return BBox3fa(lower,upper);
      }
      
      friend bool operator< (const BuildRef& a, const BuildRef& b) {
        return a.lower.w < b.lower.w;
      }
      
    public:
      Vec3fa lower;
      Vec3fa upper;
      BVH4::NodeRef node;
    };
      
      /*! Constructor. */
      BVH4BuilderTwoLevel (BVH4* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel);
      
      /*! Destructor */
      ~BVH4BuilderTwoLevel ();
      
      /*! builder entry point */
      void build(size_t threadIndex, size_t threadCount);

      void clear();

      void open_sequential(size_t numPrimitives);
      
    public:
      BVH4* bvh;
      std::vector<BVH4*>& objects;
      std::vector<Builder*> builders;
      
    public:
      Scene* scene;
      createTriangleMeshAccelTy createTriangleMeshAccel;
      
      mvector<BuildRef> refs;
      mvector<PrimRef> prims;
      AlignedAtomicCounter32 nextRef;
    };
  }
}
