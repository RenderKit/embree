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

#include "../bvh/bvh.h"

namespace embree
{
  namespace isa
  {
    template<int N>
    class BVHNRefitter
    {
    public:

      /*! Type shortcuts */
      typedef BVHN<N> BVH;
      typedef typename BVH::Node Node;
      typedef typename BVH::NodeRef NodeRef;

      struct LeafBoundsInterface {
        virtual const BBox3fa leafBounds(NodeRef& ref) const = 0;
      };

    public:
    
      /*! Constructor. */
      BVHNRefitter (BVH* bvh, const LeafBoundsInterface& leafBounds);

      /*! refits the BVH */
      void refit();

    private:
      size_t annotate_tree_sizes(NodeRef& ref);
      void calculate_refit_roots ();

      /* static subtrees based on BVH depth */
      void gather_subtree_refs(NodeRef& ref, 
                               size_t &subtrees,
                               const size_t depth = 0);

      BBox3fa refit_toplevel(NodeRef& ref,
                             size_t &subtrees,
                             const size_t depth = 0);

      
      /* dynamic subtrees */
      __forceinline BBox3fa node_bounds(NodeRef& ref)
      {
        if (ref.isNode())
          return ref.node()->bounds();
        else
          return leafBounds.leafBounds(ref);
      }

      BBox3fa recurse_bottom(NodeRef& ref);
      BBox3fa recurse_top(NodeRef& ref);
      
    public:
      BVH* bvh;                              //!< BVH to refit
      const LeafBoundsInterface& leafBounds; //!< calculates bounds of leaves
      std::vector<NodeRef*> roots;           //!< List of equal sized subtrees for bvh refit

      static const size_t MAX_SUB_TREE_EXTRACTION_DEPTH = (N==4) ? 5    : (N==8) ? 4    : 3;
      static const size_t MAX_NUM_SUB_TREES             = (N==4) ? 1024 : (N==8) ? 4096 : N*N*N; // N ^ MAX_SUB_TREE_EXTRACTION_DEPTH
      size_t numSubTrees;
      NodeRef subTrees[MAX_NUM_SUB_TREES];
    };

    template<int N, typename Mesh, typename Primitive>
    class BVHNRefitT : public Builder, public BVHNRefitter<N>::LeafBoundsInterface
    {
      ALIGNED_CLASS;
    public:
      
      /*! Type shortcuts */
      typedef BVHN<N> BVH;
      typedef typename BVH::Node Node;
      typedef typename BVH::NodeRef NodeRef;
      
    public:
      BVHNRefitT (BVH* bvh, Builder* builder, Mesh* mesh, size_t mode);
      ~BVHNRefitT();

      virtual void build(size_t threadIndex, size_t threadCount);
      
      virtual void clear();

      virtual const BBox3fa leafBounds (NodeRef& ref) const
      {
        size_t num; char* prim = ref.leaf(num);
        if (unlikely(ref == BVH::emptyNode)) return empty;
        BBox3fa bounds = empty;
        for (size_t i=0; i<num; i++)
            bounds.extend(((Primitive*)prim)[i].update(mesh));
        return bounds;
      }
      
    private:
      Mesh* mesh;
      Builder* builder;
      BVHNRefitter<N>* refitter;
      BVH* bvh;
    };
  }
}
