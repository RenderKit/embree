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

namespace embree
{
  namespace isa
  {
    class BVH4Refitter
    {
    public:

      /*! Type shortcuts */
      typedef BVH4::Node    Node;
      typedef BVH4::NodeRef NodeRef;

      struct LeafBoundsInterface {
        virtual const BBox3fa leafBounds(NodeRef& ref) const = 0;
      };

    public:
    
      /*! Constructor. */
      BVH4Refitter (BVH4* bvh, const LeafBoundsInterface& leafBounds);

      /*! refits the BVH */
      void refit();

    private:
      size_t annotate_tree_sizes(NodeRef& ref);
      void calculate_refit_roots ();

      /* static subtrees based on BVH4 depth */
      void gather_subtree_refs(NodeRef& ref, 
                               size_t &subtrees,
                               const size_t depth = 0);

      BBox3fa refit_toplevel(NodeRef& ref,
                             size_t &subtrees,
                             const size_t depth = 0);

      
      /* dynamic subtrees */
      BBox3fa node_bounds(NodeRef& ref);
      BBox3fa recurse_bottom(NodeRef& ref);
      BBox3fa recurse_top(NodeRef& ref);
      
    public:
      BVH4* bvh;                             //!< BVH to refit
      const LeafBoundsInterface& leafBounds; //!< calculates bounds of leaves
      std::vector<NodeRef*> roots;           //!< List of equal sized subtrees for bvh refit

      static const size_t MAX_NUM_SUB_TREES = 1024;
      static const size_t MAX_SUB_TREE_EXTRACTION_DEPTH = 5;
      size_t numSubTrees;
      BVH4::NodeRef subTrees[MAX_NUM_SUB_TREES];      
    };

    template<typename Primitive>
      class BVH4RefitT : public Builder, public BVH4Refitter::LeafBoundsInterface 
    {
      ALIGNED_CLASS;
    public:
      
      /*! Type shortcuts */
      typedef BVH4::Node    Node;
      typedef BVH4::NodeRef NodeRef;
      
    public:
      BVH4RefitT (BVH4* bvh, Builder* builder, TriangleMesh* mesh, size_t mode);
      ~BVH4RefitT();

      virtual void build(size_t threadIndex, size_t threadCount);
      
      virtual void clear();

      virtual const BBox3fa leafBounds (NodeRef& ref) const
      {
        size_t N; char* prim = ref.leaf(N);
        if (unlikely(ref == BVH4::emptyNode)) return empty;
        BBox3fa bounds = empty;
        for (size_t i=0; i<N; i++)
            bounds.extend(((Primitive*)prim)[i].update(mesh));
        return bounds;
      }
      
    private:
      TriangleMesh* mesh;
      Builder* builder;
      BVH4Refitter* refitter;
      BVH4* bvh;
    };
  }
}
