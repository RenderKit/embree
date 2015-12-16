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

#include "bvh_refit.h"
#include "bvh_statistics.h"

#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglei.h"

#include <algorithm>

#define STATIC_SUBTREE_EXTRACTION 1

namespace embree
{
  namespace isa
  {
    static const size_t block_size = 1024;
    
    template<int N>
    __forceinline bool compare(const typename BVHN<N>::NodeRef* a, const typename BVHN<N>::NodeRef* b)
    {
      int sa = *(size_t*)&a->node()->lower_x;
      int sb = *(size_t*)&b->node()->lower_x;
      return sa < sb;
    }

    template<int N>
    __forceinline BBox<Vec3<vfloat<N>>> transpose(const BBox3fa* bounds);

    template<>
    __forceinline BBox3vf4 transpose<4>(const BBox3fa* bounds)
    {
      BBox3vf4 dest;

      transpose((vfloat4&)bounds[0].lower,
                (vfloat4&)bounds[1].lower,
                (vfloat4&)bounds[2].lower,
                (vfloat4&)bounds[3].lower,
                dest.lower.x,
                dest.lower.y,
                dest.lower.z);

      transpose((vfloat4&)bounds[0].upper,
                (vfloat4&)bounds[1].upper,
                (vfloat4&)bounds[2].upper,
                (vfloat4&)bounds[3].upper,
                dest.upper.x,
                dest.upper.y,
                dest.upper.z);

      return dest;
    }

#if defined(__AVX__)
    template<>
    __forceinline BBox3vf8 transpose<8>(const BBox3fa* bounds)
    {
      BBox3vf8 dest;

      transpose((vfloat4&)bounds[0].lower,
                (vfloat4&)bounds[1].lower,
                (vfloat4&)bounds[2].lower,
                (vfloat4&)bounds[3].lower,
                (vfloat4&)bounds[4].lower,
                (vfloat4&)bounds[5].lower,
                (vfloat4&)bounds[6].lower,
                (vfloat4&)bounds[7].lower,
                dest.lower.x,
                dest.lower.y,
                dest.lower.z);

      transpose((vfloat4&)bounds[0].upper,
                (vfloat4&)bounds[1].upper,
                (vfloat4&)bounds[2].upper,
                (vfloat4&)bounds[3].upper,
                (vfloat4&)bounds[4].upper,
                (vfloat4&)bounds[5].upper,
                (vfloat4&)bounds[6].upper,
                (vfloat4&)bounds[7].upper,
                dest.upper.x,
                dest.upper.y,
                dest.upper.z);

      return dest;
    }
#endif

    template<int N>
    __forceinline BBox3fa merge(const BBox3fa* bounds);

    template<>
    __forceinline BBox3fa merge<4>(const BBox3fa* bounds)
    {
      const Vec3fa lower = min(min(bounds[0].lower,bounds[1].lower),
                               min(bounds[2].lower,bounds[3].lower));
      const Vec3fa upper = max(max(bounds[0].upper,bounds[1].upper),
                               max(bounds[2].upper,bounds[3].upper));
      return BBox3fa(lower,upper);
    }

#if defined(__AVX__)
    template<>
    __forceinline BBox3fa merge<8>(const BBox3fa* bounds)
    {
      const Vec3fa lower = min(min(min(bounds[0].lower,bounds[1].lower),min(bounds[2].lower,bounds[3].lower)),
                               min(min(bounds[4].lower,bounds[5].lower),min(bounds[6].lower,bounds[7].lower)));
      const Vec3fa upper = max(max(max(bounds[0].upper,bounds[1].upper),max(bounds[2].upper,bounds[3].upper)),
                               max(max(bounds[4].upper,bounds[5].upper),max(bounds[6].upper,bounds[7].upper)));
      return BBox3fa(lower,upper);
    }
#endif

    template<int N>
    BVHNRefitter<N>::BVHNRefitter (BVH* bvh, const LeafBoundsInterface& leafBounds)
      : bvh(bvh), leafBounds(leafBounds), numSubTrees(0)
    {
#if STATIC_SUBTREE_EXTRACTION

#else
      if (bvh->numPrimitives > block_size) {
        annotate_tree_sizes(bvh->root);
        calculate_refit_roots();
      }
#endif
    }

    template<int N>
    void BVHNRefitter<N>::refit()
    {
#if STATIC_SUBTREE_EXTRACTION
      if (bvh->numPrimitives <= block_size) {
        bvh->bounds = recurse_bottom(bvh->root);
      }
      else
      {
        numSubTrees = 0;
        gather_subtree_refs(bvh->root,numSubTrees,0);

        if (numSubTrees)
          parallel_for(size_t(0), numSubTrees, [&] (const range<size_t>& r) {
              for (size_t i=r.begin(); i<r.end(); i++) {
                NodeRef& ref = subTrees[i];
                recurse_bottom(ref);
              }
            });

        numSubTrees = 0;        
        bvh->bounds = refit_toplevel(bvh->root,numSubTrees,0);
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

    template<int N>
    size_t BVHNRefitter<N>::annotate_tree_sizes(NodeRef& ref)
    {
      if (ref.isNode())
      {
        Node* node = ref.node();
        size_t n = 0;
        for (size_t i=0; i<N; i++) {
          NodeRef& child = node->child(i);
          if (child == BVH::emptyNode) continue;
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

    template<int N>
    void BVHNRefitter<N>::gather_subtree_refs(NodeRef& ref,
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
        for (size_t i=0; i<N; i++) {
          NodeRef& child = node->child(i);
          if (child == BVH::emptyNode) continue;
          gather_subtree_refs(child,subtrees,depth+1); 
        }
      }
    }

    template<int N>
    BBox3fa BVHNRefitter<N>::refit_toplevel(NodeRef& ref,
                                            size_t &subtrees,
                                            const size_t depth)
    {
      if (ref.isNode())
      {
        Node* node = ref.node();
        BBox3fa bounds[N];

        if (depth >= MAX_SUB_TREE_EXTRACTION_DEPTH) 
        {
          assert(subtrees < MAX_NUM_SUB_TREES);
          assert(subTrees[subtrees++] == ref);
        }

        for (size_t i=0; i<N; i++)
        {
          NodeRef& child = node->child(i);
          bounds[i] = BBox3fa(empty);

          if (unlikely(child == BVH::emptyNode)) continue;

          if (depth >= MAX_SUB_TREE_EXTRACTION_DEPTH) 
            bounds[i] = node->bounds();
          else
            bounds[i] = refit_toplevel(child,subtrees,depth+1); 
        }
        
        BBox<Vec3<vfloat<N>>> boundsT = transpose<N>(bounds);
      
        /* set new bounds */
        node->lower_x = boundsT.lower.x;
        node->lower_y = boundsT.lower.y;
        node->lower_z = boundsT.lower.z;
        node->upper_x = boundsT.upper.x;
        node->upper_y = boundsT.upper.y;
        node->upper_z = boundsT.upper.z;
        
        return merge<N>(bounds);
      }
      else
        return leafBounds.leafBounds(ref);
    }

    template<int N>
    void BVHNRefitter<N>::calculate_refit_roots ()
    {
      if (!bvh->root.isNode()) return;
      
      roots.push_back(&bvh->root);
      std::make_heap (roots.begin(), roots.end(), compare<N>);
      
      while (true)
      {
        std::pop_heap(roots.begin(), roots.end(), compare<N>);
        NodeRef* node = roots.back();
        roots.pop_back();
        if (*(size_t*)&node->node()->lower_x < block_size) 
          break;
        
        for (size_t i=0; i<N; i++) {
          NodeRef* child = &node->node()->child(i);
          if (child->isNode()) {
            roots.push_back(child);
            std::push_heap(roots.begin(), roots.end(), compare<N>);
          }
        }
      }
    }
    
    template<int N>
    BBox3fa BVHNRefitter<N>::recurse_bottom(NodeRef& ref)
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
      BBox3fa bounds[N];

      #pragma unroll
      for (size_t i=0; i<N; i++)
        bounds[i] = recurse_bottom(node->child(i));
      
      /* AOS to SOA transform */
      BBox<Vec3<vfloat<N>>> boundsT = transpose<N>(bounds);
      
      /* set new bounds */
      node->lower_x = boundsT.lower.x;
      node->lower_y = boundsT.lower.y;
      node->lower_z = boundsT.lower.z;
      node->upper_x = boundsT.upper.x;
      node->upper_y = boundsT.upper.y;
      node->upper_z = boundsT.upper.z;
      
      /* return merged bounds */
      return merge<N>(bounds);
    }
    
    template<int N>
    BBox3fa BVHNRefitter<N>::recurse_top(NodeRef& ref)
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
      BBox3fa bounds[N];

      #pragma unroll
      for (size_t i=0; i<N; i++)
        bounds[i] = recurse_top(node->child(i));
      
      /* AOS to SOA transform */
      BBox<Vec3<vfloat<N>>> boundsT = transpose<N>(bounds);
      
      /* set new bounds */
      node->lower_x = boundsT.lower.x;
      node->lower_y = boundsT.lower.y;
      node->lower_z = boundsT.lower.z;
      node->upper_x = boundsT.upper.x;
      node->upper_y = boundsT.upper.y;
      node->upper_z = boundsT.upper.z;

      /* return merged bounds */
      return merge<N>(bounds);
    }
    
    template<int N, typename Mesh, typename Primitive>
    BVHNRefitT<N,Mesh,Primitive>::BVHNRefitT (BVH* bvh, Builder* builder, Mesh* mesh, size_t mode)
      : builder(builder), refitter(nullptr), mesh(mesh), bvh(bvh) {}

    template<int N, typename Mesh, typename Primitive>
    BVHNRefitT<N,Mesh,Primitive>::~BVHNRefitT () {
      delete builder;
      delete refitter;
    }

    template<int N, typename Mesh, typename Primitive>
    void BVHNRefitT<N,Mesh,Primitive>::clear()
    {
      if (builder) 
        builder->clear();
    }
    
    template<int N, typename Mesh, typename Primitive>
    void BVHNRefitT<N,Mesh,Primitive>::build(size_t threadIndex, size_t threadCount)
    {
      /* build initial BVH */
      if (builder) {
        builder->build(threadIndex,threadCount);
        delete builder; builder = nullptr;
        refitter = new BVHNRefitter<N>(bvh,*(typename BVHNRefitter<N>::LeafBoundsInterface*)this);
      }
      
      /* refit BVH */
      double t0 = 0.0;
      if (bvh->device->verbosity(2)) {
        std::cout << "refitting BVH" << N << " <" << bvh->primTy.name << "> ... " << std::flush;
        t0 = getSeconds();
      }
      
      refitter->refit();

      if (bvh->device->verbosity(2)) 
      {
        double t1 = getSeconds();
        std::cout << "[DONE]" << std::endl;
        std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(mesh->size())/(t1-t0) << " Mprim/s" << std::endl;
        std::cout << BVHNStatistics<N>(bvh).str();
      }
    }

    template class BVHNRefitter<4>;
#if defined(__AVX__)
    template class BVHNRefitter<8>;
#endif
    
    Builder* BVH4Line4iMeshBuilderSAH (void* bvh, LineSegments* mesh, size_t mode);
    Builder* BVH4Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode);
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode);
#endif
    Builder* BVH4Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode);
    Builder* BVH4Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode);

    Builder* BVH4Line4iMeshRefitSAH (void* accel, LineSegments* mesh, size_t mode) { return new BVHNRefitT<4,LineSegments,Line4i>((BVH4*)accel,BVH4Line4iMeshBuilderSAH(accel,mesh,mode),mesh,mode); }
    Builder* BVH4Triangle4MeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVHNRefitT<4,TriangleMesh,Triangle4>((BVH4*)accel,BVH4Triangle4MeshBuilderSAH(accel,mesh,mode),mesh,mode); }

#if defined(__AVX__)
    Builder* BVH4Triangle8MeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVHNRefitT<4,TriangleMesh,Triangle8>((BVH4*)accel,BVH4Triangle8MeshBuilderSAH(accel,mesh,mode),mesh,mode); }
#endif
    Builder* BVH4Triangle4vMeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVHNRefitT<4,TriangleMesh,Triangle4v>((BVH4*)accel,BVH4Triangle4vMeshBuilderSAH(accel,mesh,mode),mesh,mode); }
    Builder* BVH4Triangle4iMeshRefitSAH (void* accel, TriangleMesh* mesh, size_t mode) { return new BVHNRefitT<4,TriangleMesh,Triangle4i>((BVH4*)accel,BVH4Triangle4iMeshBuilderSAH(accel,mesh,mode),mesh,mode); }

  }
}

