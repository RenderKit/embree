// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "bvh.h"
#include "../common/scene_triangle_mesh.h"
#include "../common/primref.h"

namespace embree
{
  namespace isa
  {
    template<int N, typename Mesh>
    class BVHNBuilderInstancing : public Builder
    {
      ALIGNED_CLASS;

      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::AlignedNode AlignedNode;
      typedef typename BVH::AlignedNodeMB AlignedNodeMB;
      typedef typename BVH::TransformNode TransformNode;

    public:

      typedef void (*createMeshAccelTy)(Mesh* mesh, AccelData*& accel, Builder*& builder);

      struct BuildRef
      {
      public:
        __forceinline BuildRef () {}

        __forceinline BuildRef (const AffineSpace3fa& local2world, const BBox3fa& localBounds_in, NodeRef node, unsigned mask, int instID, int xfmID, int oldtype, int depth = 0, unsigned int numPrims = 1)
          : local2world(local2world), localBounds(localBounds_in), node(node), mask(mask), instID(instID), xfmID(xfmID), oldtype(oldtype), depth(depth), numPrims(numPrims)
        {
          if (node.isAlignedNode()) {
          //if (node.isAlignedNode() || node.isAlignedNodeMB()) {
            const BBox3fa worldBounds = xfmBounds(local2world,localBounds);
            localBounds.lower.w = area(worldBounds);
          } else {
            localBounds.lower.w = 0.0f;
          }
        }

        __forceinline void clearArea() {
          localBounds.lower.w = 0.0f;
        }

        __forceinline BBox3fa worldBounds() const {
          return xfmBounds(local2world,localBounds);
        }

        __forceinline BBox3fa bounds() const {
          return worldBounds();
        }

        __forceinline unsigned geomID() const { 
          return instID;
        }

        __forceinline unsigned int primID() const { return numPrims; }

        __forceinline unsigned int size() const { return 1; } 

        __forceinline unsigned int numPrimitives() const { return numPrims; } 


        __forceinline void binBoundsAndCenter(BBox3fa& bounds_o, Vec3fa& center_o) const 
        {
          bounds_o = bounds();
          center_o = embree::center2(bounds_o);
          bounds_o.upper.a = numPrims;
        }

        friend bool operator< (const BuildRef& a, const BuildRef& b) {
          return a.localBounds.lower.w < b.localBounds.lower.w;
        }

      public:
        AffineSpace3fa local2world;
        BBox3fa localBounds;
        NodeRef node;
        unsigned mask;
        int instID;
        int xfmID;
        int oldtype;
        int depth;
        unsigned int numPrims;
      };
      
      /*! Constructor. */
      BVHNBuilderInstancing (BVH* bvh, Scene* scene, const createMeshAccelTy createMeshAccel);
      
      /*! Destructor */
      ~BVHNBuilderInstancing ();
      
      /*! builder entry point */
      void build();
      void deleteGeometry(size_t geomID);
      void clear();

      void open(size_t numPrimitives);

      size_t numCollapsedTransformNodes;
      NodeRef collapse(NodeRef& node);

      __forceinline size_t openBuildRef(BuildRef &bref, BuildRef *const refs) {
        if (bref.node.isLeaf())
        {
          refs[0] = bref;
          return 1;
        }
        assert(bref.node.isAlignedNode());
        NodeRef ref = bref.node;
        unsigned int numPrims = max((unsigned int)bref.numPrimitives() / N,(unsigned int)1);
        AlignedNode* node = ref.alignedNode();
        size_t n = 0;
        for (size_t i=0; i<N; i++) {
          if (node->child(i) == BVH::emptyNode) continue;
          refs[i] = BuildRef(bref.local2world,node->bounds(i),node->child(i),bref.mask,bref.instID,bref.xfmID,bref.oldtype,bref.depth+1,numPrims);
          n++;
        }
        assert(n > 1);
        return n;        
      }
      
    public:
      BVH* bvh;
      std::vector<BVH*>& objects;
      std::vector<Builder*> builders;
      const createMeshAccelTy createMeshAccel;

    public:
      Scene* scene;
      
      mvector<BuildRef> refs;
      mvector<PrimRef> prims;
      std::atomic<size_t> nextRef;
    };
  }
}
