// ======================================================================== //
// Copyright 2009-2020 Intel Corporation                                    //
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

#include "bvh_node_base.h"

namespace embree
{
  /*! BVHN AlignedNode */
  template<int N>
    struct AlignedNode_t : public BaseNode_t<N>
  {
    using BaseNode_t<N>::children;
    
    struct Create
    {
      __forceinline NodeRefPtr<N> operator() (const FastAllocator::CachedAllocator& alloc, size_t numChildren = 0) const
      {
        AlignedNode_t<N>* node = (AlignedNode_t<N>*) alloc.malloc0(sizeof(AlignedNode_t<N>),NodeRefPtr<N>::byteNodeAlignment); node->clear();
        return NodeRefPtr<N>::encodeNode(node);
      }
    };
    
    struct Set
    {
      __forceinline void operator() (NodeRefPtr<N> node, size_t i, NodeRefPtr<N> child, const BBox3fa& bounds) const {
        node.alignedNode()->setRef(i,child);
        node.alignedNode()->setBounds(i,bounds);
      }
    };
    
    struct Create2
    {
      template<typename BuildRecord>
      __forceinline NodeRefPtr<N> operator() (BuildRecord* children, const size_t num, const FastAllocator::CachedAllocator& alloc) const
      {
        AlignedNode_t<N>* node = (AlignedNode_t<N>*) alloc.malloc0(sizeof(AlignedNode_t<N>), NodeRefPtr<N>::byteNodeAlignment); node->clear();
        for (size_t i=0; i<num; i++) node->setBounds(i,children[i].bounds());
        return NodeRefPtr<N>::encodeNode(node);
      }
    };
    
    struct Set2
    {
      template<typename BuildRecord>
      __forceinline NodeRefPtr<N> operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRefPtr<N> ref, NodeRefPtr<N>* children, const size_t num) const
      {
        AlignedNode_t<N>* node = ref.alignedNode();
        for (size_t i=0; i<num; i++) node->setRef(i,children[i]);
        return ref;
      }
    };
    
    struct Set3
    {
      Set3 (FastAllocator* allocator, PrimRef* prims)
      : allocator(allocator), prims(prims) {}
      
      template<typename BuildRecord>
      __forceinline NodeRefPtr<N> operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRefPtr<N> ref, NodeRefPtr<N>* children, const size_t num) const
      {
        AlignedNode_t<N>* node = ref.alignedNode();
        for (size_t i=0; i<num; i++) node->setRef(i,children[i]);
        
        if (unlikely(precord.alloc_barrier))
        {
          PrimRef* begin = &prims[precord.prims.begin()];
          PrimRef* end   = &prims[precord.prims.end()]; // FIXME: extended end for spatial split builder!!!!!
          size_t bytes = (size_t)end - (size_t)begin;
          allocator->addBlock(begin,bytes);
        }
        
        return ref;
      }
      
      FastAllocator* const allocator;
      PrimRef* const prims;
    };
    
    /*! Clears the node. */
    __forceinline void clear() {
      lower_x = lower_y = lower_z = pos_inf;
      upper_x = upper_y = upper_z = neg_inf;
      BaseNode_t<N>::clear();
    }
    
    /*! Sets bounding box and ID of child. */
    __forceinline void setRef(size_t i, const NodeRefPtr<N>& ref) {
      assert(i < N);
      children[i] = ref;
    }
    
    /*! Sets bounding box of child. */
    __forceinline void setBounds(size_t i, const BBox3fa& bounds)
    {
      assert(i < N);
      lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
      upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
    }
    
    /*! Sets bounding box and ID of child. */
    __forceinline void set(size_t i, const NodeRefPtr<N>& ref, const BBox3fa& bounds) {
      setBounds(i,bounds);
      children[i] = ref;
    }
    
    /*! Returns bounds of node. */
    __forceinline BBox3fa bounds() const {
      const Vec3fa lower(reduce_min(lower_x),reduce_min(lower_y),reduce_min(lower_z));
      const Vec3fa upper(reduce_max(upper_x),reduce_max(upper_y),reduce_max(upper_z));
      return BBox3fa(lower,upper);
    }
    
    /*! Returns bounds of specified child. */
    __forceinline BBox3fa bounds(size_t i) const
    {
      assert(i < N);
      const Vec3fa lower(lower_x[i],lower_y[i],lower_z[i]);
      const Vec3fa upper(upper_x[i],upper_y[i],upper_z[i]);
      return BBox3fa(lower,upper);
    }
    
    /*! Returns extent of bounds of specified child. */
    __forceinline Vec3fa extend(size_t i) const {
      return bounds(i).size();
    }
    
    /*! Returns bounds of all children (implemented later as specializations) */
    __forceinline void bounds(BBox<vfloat4>& bounds0, BBox<vfloat4>& bounds1, BBox<vfloat4>& bounds2, BBox<vfloat4>& bounds3) const {} // N = 4
    
    /*! swap two children of the node */
    __forceinline void swap(size_t i, size_t j)
    {
      assert(i<N && j<N);
      std::swap(children[i],children[j]);
      std::swap(lower_x[i],lower_x[j]);
      std::swap(lower_y[i],lower_y[j]);
      std::swap(lower_z[i],lower_z[j]);
      std::swap(upper_x[i],upper_x[j]);
      std::swap(upper_y[i],upper_y[j]);
      std::swap(upper_z[i],upper_z[j]);
    }

    /*! swap the children of two nodes */
    __forceinline static void swap(AlignedNode_t* a, size_t i, AlignedNode_t* b, size_t j)
    {
      assert(i<N && j<N);
      std::swap(a->children[i],b->children[j]);
      std::swap(a->lower_x[i],b->lower_x[j]);
      std::swap(a->lower_y[i],b->lower_y[j]);
      std::swap(a->lower_z[i],b->lower_z[j]);
      std::swap(a->upper_x[i],b->upper_x[j]);
      std::swap(a->upper_y[i],b->upper_y[j]);
      std::swap(a->upper_z[i],b->upper_z[j]);
    }

    /*! compacts a node (moves empty children to the end) */
    __forceinline static void compact(AlignedNode_t* a)
    {
      /* find right most filled node */
      ssize_t j=N;
      for (j=j-1; j>=0; j--)
        if (a->child(j) != NodeRefPtr<N>::emptyNode)
          break;

      /* replace empty nodes with filled nodes */
      for (ssize_t i=0; i<j; i++) {
        if (a->child(i) == NodeRefPtr<N>::emptyNode) {
          a->swap(i,j);
          for (j=j-1; j>i; j--)
            if (a->child(j) != NodeRefPtr<N>::emptyNode)
              break;
        }
      }
    }
    
    /*! Returns reference to specified child */
    __forceinline       NodeRefPtr<N>& child(size_t i)       { assert(i<N); return children[i]; }
    __forceinline const NodeRefPtr<N>& child(size_t i) const { assert(i<N); return children[i]; }
    
    /*! output operator */
    friend std::ostream& operator<<(std::ostream& o, const AlignedNode_t<N>& n)
    {
      o << "AlignedNode { " << std::endl;
      o << "  lower_x " << n.lower_x << std::endl;
      o << "  upper_x " << n.upper_x << std::endl;
      o << "  lower_y " << n.lower_y << std::endl;
      o << "  upper_y " << n.upper_y << std::endl;
      o << "  lower_z " << n.lower_z << std::endl;
      o << "  upper_z " << n.upper_z << std::endl;
      o << "  children = ";
      for (size_t i=0; i<N; i++) o << n.children[i] << " ";
      o << std::endl;
      o << "}" << std::endl;
      return o;
    }
    
  public:
    vfloat<N> lower_x;           //!< X dimension of lower bounds of all N children.
    vfloat<N> upper_x;           //!< X dimension of upper bounds of all N children.
    vfloat<N> lower_y;           //!< Y dimension of lower bounds of all N children.
    vfloat<N> upper_y;           //!< Y dimension of upper bounds of all N children.
    vfloat<N> lower_z;           //!< Z dimension of lower bounds of all N children.
    vfloat<N> upper_z;           //!< Z dimension of upper bounds of all N children.
  };
}
