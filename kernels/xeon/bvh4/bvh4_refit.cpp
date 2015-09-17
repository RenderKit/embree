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

#include "bvh4_refit.h"
#include "bvh4_statistics.h"

#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle4v.h"
#include "../geometry/triangle4i.h"

#include <algorithm>

#define STATIC_SUBTREE_EXTRACTION 1

#define TIMER(x) 

namespace embree
{
  namespace isa
  {
    static const size_t block_size = 1024;
    
    __forceinline bool compare(const BVH4::NodeRef* a, const BVH4::NodeRef* b)
    {
      int sa = *(size_t*)&a->node()->lower_x;
      int sb = *(size_t*)&b->node()->lower_x;
      return sa < sb;
    }

    BVH4Refitter::BVH4Refitter (BVH4* bvh, const LeafBoundsInterface& leafBounds)
      : bvh(bvh), leafBounds(leafBounds), numSubTrees(0)
    {
#if STATIC_SUBTREE_EXTRACTION == 1

#else
      if (bvh->numPrimitives > block_size) {
        annotate_tree_sizes(bvh->root);
        calculate_refit_roots();
      }
#endif
    }

    void BVH4Refitter::refit()
    {
#if STATIC_SUBTREE_EXTRACTION == 1
      if (bvh->numPrimitives <= block_size) 
      {
        bvh->bounds = recurse_bottom(bvh->root);
      }
      else
      {
        TIMER(const double t0 = getSeconds());

        numSubTrees = 0;
        gather_subtree_refs(bvh->root,numSubTrees,0);

        TIMER(const double t1 = getSeconds());
        TIMER(PRINT(t1-t0));

        TIMER(const double t2 = getSeconds());

        if (numSubTrees)
          parallel_for(size_t(0), numSubTrees, [&] (const range<size_t>& r) {
              for (size_t i=r.begin(); i<r.end(); i++) {
                NodeRef& ref = subTrees[i];
                recurse_bottom(ref);
              }
            });

        TIMER(const double t3 = getSeconds());
        TIMER(PRINT(t3-t2));
        TIMER(const double t4 = getSeconds());

        numSubTrees = 0;        
        bvh->bounds = refit_toplevel(bvh->root,numSubTrees,0);

        TIMER(const double t5 = getSeconds());
        TIMER(PRINT(t5-t4));

        //PRINT(numSubTrees);
        //PRINT(bvh->bounds);
        //if (numSubTrees) exit(0);

      }
#else
      /* single threaded fallback */
      size_t numRoots = roots.size();
      if (numRoots <= 1) {
        bvh->bounds = recurse_bottom(bvh->root);
      }

      /* parallel refit */
      else 
      {

        parallel_for(size_t(0), roots.size(), [&] (const range<size_t>& r) {
            for (size_t i=r.begin(); i<r.end(); i++) {
              NodeRef& ref = *roots[i];
              recurse_bottom(ref);
              ref.setBarrier();
            }
          });
        bvh->bounds = recurse_top(bvh->root);
      }
#endif
    
  }

    size_t BVH4Refitter::annotate_tree_sizes(BVH4::NodeRef& ref)
    {
      if (ref.isNode())
      {
        Node* node = ref.node();
        size_t n = 0;
        for (size_t i=0; i<BVH4::N; i++) {
          BVH4::NodeRef& child = node->child(i);
          if (child == BVH4::emptyNode) continue;
          n += annotate_tree_sizes(child); 
        }
        *((size_t*)&node->lower_x) = n;
        return n;
      }
      else
      {
        size_t num; 
        ref.leaf(num);
        return num;
      }
    }

    void BVH4Refitter::gather_subtree_refs(BVH4::NodeRef& ref, 
                                           size_t &subtrees,
                                           const size_t depth)
    {
      if (ref.isNode())
      {
        if (depth >= MAX_SUB_TREE_EXTRACTION_DEPTH) 
        {
          assert(subtrees < MAX_NUM_SUB_TREES);
          subTrees[subtrees++] = ref;
          return;
        }

        Node* node = ref.node();
        for (size_t i=0; i<BVH4::N; i++) {
          BVH4::NodeRef& child = node->child(i);
          if (child == BVH4::emptyNode) continue;
          gather_subtree_refs(child,subtrees,depth+1); 
        }
      }
    }

    BBox3fa BVH4Refitter::refit_toplevel(NodeRef& ref,
                                         size_t &subtrees,
                                         const size_t depth)
    {
      if (ref.isNode())
      {
        Node* node = ref.node();
        BBox3fa bounds[4];

        if (depth >= MAX_SUB_TREE_EXTRACTION_DEPTH) 
        {
          assert(subtrees < MAX_NUM_SUB_TREES);
          assert(subTrees[subtrees++] == ref);
        }

        for (size_t i=0; i<BVH4::N; i++) 
        {
          BVH4::NodeRef& child = node->child(i);
          bounds[i] = BBox3fa(empty);

          if (unlikely(child == BVH4::emptyNode)) continue;

          if (depth >= MAX_SUB_TREE_EXTRACTION_DEPTH) 
            bounds[i] = node->bounds();
          else
            bounds[i] = refit_toplevel(child,subtrees,depth+1); 
        }
        
        BBox<Vec3f4> dest;

        transpose((float4&)bounds[0].lower,
                  (float4&)bounds[1].lower,
                  (float4&)bounds[2].lower,
                  (float4&)bounds[3].lower,
                  dest.lower.x,
                  dest.lower.y,
                  dest.lower.z);
        
        transpose((float4&)bounds[0].upper,
                  (float4&)bounds[1].upper,
                  (float4&)bounds[2].upper,
                  (float4&)bounds[3].upper,
                  dest.upper.x,
                  dest.upper.y,
                  dest.upper.z);
      
        /* set new bounds */
        node->lower_x = dest.lower.x;
        node->lower_y = dest.lower.y;
        node->lower_z = dest.lower.z;
        node->upper_x = dest.upper.x;
        node->upper_y = dest.upper.y;
        node->upper_z = dest.upper.z;
        
        const Vec3fa lower = min(min(bounds[0].lower,bounds[1].lower),
                                 min(bounds[2].lower,bounds[3].lower));
        const Vec3fa upper = max(max(bounds[0].upper,bounds[1].upper),
                                 max(bounds[2].upper,bounds[3].upper));
        return BBox3fa(lower,upper);
      }
      else
        return leafBounds.leafBounds(ref);
    }

    
    void BVH4Refitter::calculate_refit_roots ()
    {
      if (!bvh->root.isNode()) return;
      
      roots.push_back(&bvh->root);
      std::make_heap (roots.begin(), roots.end(), compare);
      
      while (true)
      {
        std::pop_heap(roots.begin(), roots.end(), compare);
        BVH4::NodeRef* node = roots.back();
        roots.pop_back();
        if (*(size_t*)&node->node()->lower_x < block_size) 
          break;
        
        for (size_t i=0; i<BVH4::N; i++) {
          BVH4::NodeRef* child = &node->node()->child(i);
          if (child->isNode()) {
            roots.push_back(child);
            std::push_heap(roots.begin(), roots.end(), compare);
          }
        }
      }
    }

    __forceinline BBox3fa BVH4Refitter::node_bounds(NodeRef& ref)
    {
      if (ref.isNode())
        return ref.node()->bounds();
      else
        return leafBounds.leafBounds(ref);
    }
    
    BBox3fa BVH4Refitter::recurse_bottom(NodeRef& ref)
    {
      /* this is a leaf node */
      if (unlikely(ref.isLeaf()))
        return leafBounds.leafBounds(ref);
      
      /* recurse if this is an internal node */
      Node* node = ref.node();

      /* enable exclusive prefetch for >= AVX platforms */
      
#if defined(__AVX__)      
      ref.prefetchW();
#endif      
      const BBox3fa bounds0 = recurse_bottom(node->child(0));
      const BBox3fa bounds1 = recurse_bottom(node->child(1));
      const BBox3fa bounds2 = recurse_bottom(node->child(2));      
      const BBox3fa bounds3 = recurse_bottom(node->child(3));
      
      /* AOS to SOA transform */
      BBox<Vec3f4> bounds;
      transpose((float4&)bounds0.lower,(float4&)bounds1.lower,(float4&)bounds2.lower,(float4&)bounds3.lower,bounds.lower.x,bounds.lower.y,bounds.lower.z);
      transpose((float4&)bounds0.upper,(float4&)bounds1.upper,(float4&)bounds2.upper,(float4&)bounds3.upper,bounds.upper.x,bounds.upper.y,bounds.upper.z);
      
      /* set new bounds */
      node->lower_x = bounds.lower.x;
      node->lower_y = bounds.lower.y;
      node->lower_z = bounds.lower.z;
      node->upper_x = bounds.upper.x;
      node->upper_y = bounds.upper.y;
      node->upper_z = bounds.upper.z;
      
      /* return merged bounds */
#if 1
      const Vec3fa lower = min(min(bounds0.lower,bounds1.lower),min(bounds2.lower,bounds3.lower));
      const Vec3fa upper = max(max(bounds0.upper,bounds1.upper),max(bounds2.upper,bounds3.upper));

      return BBox3fa(lower,upper);
#else
      const float lower_x = reduce_min(bounds.lower.x);
      const float lower_y = reduce_min(bounds.lower.y);
      const float lower_z = reduce_min(bounds.lower.z);
      const float upper_x = reduce_max(bounds.upper.x);
      const float upper_y = reduce_max(bounds.upper.y);
      const float upper_z = reduce_max(bounds.upper.z);

      return BBox3fa(Vec3fa(lower_x,lower_y,lower_z),
                     Vec3fa(upper_x,upper_y,upper_z));
#endif
    }
    
    BBox3fa BVH4Refitter::recurse_top(NodeRef& ref)
    {
      /* stop here if we encounter a barrier */
      if (unlikely(ref.isBarrier())) {
        ref.clearBarrier();
        return node_bounds(ref);
      }
      
      /* this is a leaf node */
      if (unlikely(ref.isLeaf()))
        return leafBounds.leafBounds(ref);
      
      /* recurse if this is an internal node */
      Node* node = ref.node();
      const BBox3fa bounds0 = recurse_top(node->child(0));
      const BBox3fa bounds1 = recurse_top(node->child(1));
      const BBox3fa bounds2 = recurse_top(node->child(2));
      const BBox3fa bounds3 = recurse_top(node->child(3));
      
      /* AOS to SOA transform */
      BBox<Vec3f4> bounds;
      transpose((float4&)bounds0.lower,(float4&)bounds1.lower,(float4&)bounds2.lower,(float4&)bounds3.lower,bounds.lower.x,bounds.lower.y,bounds.lower.z);
      transpose((float4&)bounds0.upper,(float4&)bounds1.upper,(float4&)bounds2.upper,(float4&)bounds3.upper,bounds.upper.x,bounds.upper.y,bounds.upper.z);
      
      /* set new bounds */
      node->lower_x = bounds.lower.x;
      node->lower_y = bounds.lower.y;
      node->lower_z = bounds.lower.z;
      node->upper_x = bounds.upper.x;
      node->upper_y = bounds.upper.y;
      node->upper_z = bounds.upper.z;

#if 1
      const Vec3fa lower = min(min(bounds0.lower,bounds1.lower),min(bounds2.lower,bounds3.lower));
      const Vec3fa upper = max(max(bounds0.upper,bounds1.upper),max(bounds2.upper,bounds3.upper));

      return BBox3fa(lower,upper);
#else      
      /* return merged bounds */
      const float lower_x = reduce_min(bounds.lower.x);
      const float lower_y = reduce_min(bounds.lower.y);
      const float lower_z = reduce_min(bounds.lower.z);
      const float upper_x = reduce_max(bounds.upper.x);
      const float upper_y = reduce_max(bounds.upper.y);
      const float upper_z = reduce_max(bounds.upper.z);
      return BBox3fa(Vec3fa(lower_x,lower_y,lower_z),
                    Vec3fa(upper_x,upper_y,upper_z));
#endif
    }
    
    template<typename Primitive>
    BVH4RefitT<Primitive>::BVH4RefitT (BVH4* bvh, Builder* builder, TriangleMesh* mesh, size_t mode)
      : builder(builder), refitter(nullptr), mesh(mesh), bvh(bvh) {}

    template<typename Primitive>
    BVH4RefitT<Primitive>::~BVH4RefitT () {
      delete builder;
      delete refitter;
    }

    template<typename Primitive>
    void BVH4RefitT<Primitive>::clear()
    {
      if (builder) 
        builder->clear();
    }
    
    template<typename Primitive>
    void BVH4RefitT<Primitive>::build(size_t threadIndex, size_t threadCount) 
    {
      /* build initial BVH */
      if (builder) {
        builder->build(threadIndex,threadCount);
        delete builder; builder = nullptr;
        refitter = new BVH4Refitter(bvh,*(LeafBoundsInterface*)this);
      }
      
      /* refit BVH */
      double t0 = 0.0;
      if (bvh->device->verbosity(2)) {
        std::cout << "refitting BVH4 <" << bvh->primTy.name << "> ... " << std::flush;
        t0 = getSeconds();
      }
      
      refitter->refit();

      if (bvh->device->verbosity(2)) 
      {
        double t1 = getSeconds();
        std::cout << "[DONE]" << std::endl;
        std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(mesh->size())/(t1-t0) << " Mprim/s" << std::endl;
        std::cout << BVH4Statistics(bvh).str();
      }
    }

#if !defined (__AVX512F__)
    
    Builder* BVH4Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode);
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode);
#endif
    Builder* BVH4Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode);
    Builder* BVH4Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode);

    Builder* BVH4Triangle4MeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVH4RefitT<Triangle4>((BVH4*)accel,BVH4Triangle4MeshBuilderSAH(accel,mesh,mode),mesh,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVH4RefitT<Triangle8>((BVH4*)accel,BVH4Triangle8MeshBuilderSAH(accel,mesh,mode),mesh,mode); }
#endif
    Builder* BVH4Triangle4vMeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVH4RefitT<Triangle4v>((BVH4*)accel,BVH4Triangle4vMeshBuilderSAH(accel,mesh,mode),mesh,mode); }
    Builder* BVH4Triangle4iMeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVH4RefitT<Triangle4i>((BVH4*)accel,BVH4Triangle4iMeshBuilderSAH(accel,mesh,mode),mesh,mode); }

#endif
  }
}

