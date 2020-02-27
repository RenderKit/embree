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

#include "bvh_node_aabb_mb.h"

namespace embree
{
  /*! Aligned 4D Motion Blur Node */
  template<typename NodeRef, int N>
    struct AlignedNodeMB4D_t : public AlignedNodeMB_t<NodeRef, N>
  {
    using BaseNode_t<NodeRef,N>::children;
    using AlignedNodeMB_t<NodeRef,N>::set;
    using AlignedNodeMB_t<NodeRef,N>::bounds;
    using AlignedNodeMB_t<NodeRef,N>::lower_x;
    using AlignedNodeMB_t<NodeRef,N>::lower_y;
    using AlignedNodeMB_t<NodeRef,N>::lower_z;
    using AlignedNodeMB_t<NodeRef,N>::upper_x;
    using AlignedNodeMB_t<NodeRef,N>::upper_y;
    using AlignedNodeMB_t<NodeRef,N>::upper_z;
    using AlignedNodeMB_t<NodeRef,N>::lower_dx;
    using AlignedNodeMB_t<NodeRef,N>::lower_dy;
    using AlignedNodeMB_t<NodeRef,N>::lower_dz;
    using AlignedNodeMB_t<NodeRef,N>::upper_dx;
    using AlignedNodeMB_t<NodeRef,N>::upper_dy;
    using AlignedNodeMB_t<NodeRef,N>::upper_dz;

    typedef BVHNodeRecord<NodeRef>     NodeRecord;
    typedef BVHNodeRecordMB<NodeRef>   NodeRecordMB;
    typedef BVHNodeRecordMB4D<NodeRef> NodeRecordMB4D;
    
    struct Create
    {
      __forceinline NodeRef operator() (const FastAllocator::CachedAllocator& alloc, bool hasTimeSplits = true) const
      {
        if (hasTimeSplits)
        {
          AlignedNodeMB4D_t* node = (AlignedNodeMB4D_t*) alloc.malloc0(sizeof(AlignedNodeMB4D_t),NodeRef::byteNodeAlignment); node->clear();
          return NodeRef::encodeNode(node);
        }
        else
        {
          AlignedNodeMB_t<NodeRef,N>* node = (AlignedNodeMB_t<NodeRef,N>*) alloc.malloc0(sizeof(AlignedNodeMB_t<NodeRef,N>),NodeRef::byteNodeAlignment); node->clear();
          return NodeRef::encodeNode(node);
        }
      }
    };
    
    struct Create2
    {
      template<typename BuildRecord>
      __forceinline NodeRef operator() (BuildRecord* children, const size_t num, const FastAllocator::CachedAllocator& alloc, bool hasTimeSplits = true) const
      {
        if (hasTimeSplits)
        {
          AlignedNodeMB4D_t* node = (AlignedNodeMB4D_t*) alloc.malloc0(sizeof(AlignedNodeMB4D_t),NodeRef::byteNodeAlignment); node->clear();
          return NodeRef::encodeNode(node);
        }
        else
        {
          AlignedNodeMB_t<NodeRef,N>* node = (AlignedNodeMB_t<NodeRef,N>*) alloc.malloc0(sizeof(AlignedNodeMB_t<NodeRef,N>),NodeRef::byteNodeAlignment); node->clear();
          return NodeRef::encodeNode(node);
        }
      }
    };

    struct Set
    {
      __forceinline void operator() (NodeRef ref, size_t i, const NodeRecordMB4D& child) const
      {
        if (likely(ref.isAlignedNodeMB())) {
          ref.alignedNodeMB()->set(i, child);
        } else {
          ref.alignedNodeMB4D()->set(i, child);
        }
      }
    };

    struct Set2
    {
      template<typename BuildRecord>
      __forceinline void operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRef ref, NodeRecordMB4D* children, const size_t num) const
      {
        if (likely(ref.isAlignedNodeMB())) {
          for (size_t i=0; i<num; i++)
            ref.alignedNodeMB()->set(i, children[i]);
        } else {
          for (size_t i=0; i<num; i++)
            ref.alignedNodeMB4D()->set(i, children[i]);
        }
      }
    };

    /*! Clears the node. */
    __forceinline void clear()  {
      lower_t = vfloat<N>(pos_inf);
      upper_t = vfloat<N>(neg_inf);
      AlignedNodeMB_t<NodeRef,N>::clear();
    }
    
    /*! Sets bounding box of child. */
    __forceinline void setBounds(size_t i, const BBox3fa& bounds0_i, const BBox3fa& bounds1_i)
    {
      /*! for empty bounds we have to avoid inf-inf=nan */
      BBox3fa bounds0(min(bounds0_i.lower,Vec3fa(+FLT_MAX)),max(bounds0_i.upper,Vec3fa(-FLT_MAX)));
      BBox3fa bounds1(min(bounds1_i.lower,Vec3fa(+FLT_MAX)),max(bounds1_i.upper,Vec3fa(-FLT_MAX)));
      bounds0 = bounds0.enlarge_by(4.0f*float(ulp));
      bounds1 = bounds1.enlarge_by(4.0f*float(ulp));
      Vec3fa dlower = bounds1.lower-bounds0.lower;
      Vec3fa dupper = bounds1.upper-bounds0.upper;
      
      lower_x[i] = bounds0.lower.x; lower_y[i] = bounds0.lower.y; lower_z[i] = bounds0.lower.z;
      upper_x[i] = bounds0.upper.x; upper_y[i] = bounds0.upper.y; upper_z[i] = bounds0.upper.z;
      
      lower_dx[i] = dlower.x; lower_dy[i] = dlower.y; lower_dz[i] = dlower.z;
      upper_dx[i] = dupper.x; upper_dy[i] = dupper.y; upper_dz[i] = dupper.z;
    }
    
    /*! Sets bounding box of child. */
    __forceinline void setBounds(size_t i, const LBBox3fa& bounds) {
      setBounds(i, bounds.bounds0, bounds.bounds1);
    }
    
    /*! Sets bounding box of child. */
    __forceinline void setBounds(size_t i, const LBBox3fa& bounds, const BBox1f& tbounds)
    {
      setBounds(i, bounds.global(tbounds));
      lower_t[i] = tbounds.lower;
      upper_t[i] = tbounds.upper == 1.0f ? 1.0f+float(ulp) : tbounds.upper;
    }
    
    /*! Sets bounding box and ID of child. */
    __forceinline void set(size_t i, NodeRef childID, const LBBox3fa& bounds, const BBox1f& tbounds) 
    {
      AlignedNodeMB_t<NodeRef,N>::setRef(i,childID);
      setBounds(i, bounds, tbounds);
    }
    
    /*! Sets bounding box and ID of child. */
    __forceinline void set(size_t i, const NodeRecordMB4D& child) {
      set(i, child.ref, child.lbounds, child.dt);
    }
    
    /*! Returns reference to specified child */
    __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
    __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }
    
    /*! Returns the expected surface area when randomly sampling the time. */
    __forceinline float expectedHalfArea(size_t i) const {
      return AlignedNodeMB_t<NodeRef,N>::lbounds(i).expectedHalfArea(timeRange(i));
    }
    
    /*! returns time range for specified child */
    __forceinline BBox1f timeRange(size_t i) const {
      return BBox1f(lower_t[i],upper_t[i]);
    }
    
    /*! stream output operator */
    friend embree_ostream operator<<(embree_ostream cout, const AlignedNodeMB4D_t& n) 
    {
      cout << "AlignedNodeMB4D {" << embree_endl;
      for (size_t i=0; i<N; i++) 
      {
        const BBox3fa b0 = n.bounds0(i);
        const BBox3fa b1 = n.bounds1(i);
        cout << "  child" << i << " { " << embree_endl;
        cout << "    bounds0 = " << lerp(b0,b1,n.lower_t[i]) << ", " << embree_endl;
        cout << "    bounds1 = " << lerp(b0,b1,n.upper_t[i]) << ", " << embree_endl;
        cout << "    time_bounds = " << n.lower_t[i] << ", " << n.upper_t[i] << embree_endl;
        cout << "  }";
      }
      cout << "}";
      return cout;
    }
    
  public:
    vfloat<N> lower_t;        //!< time dimension of lower bounds of all N children
    vfloat<N> upper_t;        //!< time dimension of upper bounds of all N children
  };
}
