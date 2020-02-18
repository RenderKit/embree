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
  /*! Node with unaligned bounds */
  template<int N>
    struct UnalignedNode_t : public BaseNode_t<N>
  {
    using BaseNode_t<N>::children;
    
    struct Create
    {
      __forceinline NodeRefPtr<N> operator() (const FastAllocator::CachedAllocator& alloc) const
      {
        UnalignedNode_t<N>* node = (UnalignedNode_t<N>*) alloc.malloc0(sizeof(UnalignedNode_t<N>),NodeRefPtr<N>::byteNodeAlignment); node->clear();
        return NodeRefPtr<N>::encodeNode(node);
      }
    };
    
    struct Set
    {
      __forceinline void operator() (NodeRefPtr<N> node, size_t i, NodeRefPtr<N> child, const OBBox3fa& bounds) const {
        node.unalignedNode()->setRef(i,child);
        node.unalignedNode()->setBounds(i,bounds);
      }
    };
    
    /*! Clears the node. */
    __forceinline void clear()
    {
      naabb.l.vx = Vec3fa(nan);
      naabb.l.vy = Vec3fa(nan);
      naabb.l.vz = Vec3fa(nan);
      naabb.p    = Vec3fa(nan);
      BaseNode_t<N>::clear();
    }
    
    /*! Sets bounding box. */
    __forceinline void setBounds(size_t i, const OBBox3fa& b)
    {
      assert(i < N);
      
      AffineSpace3fa space = b.space;
      space.p -= b.bounds.lower;
      space = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19f),b.bounds.upper-b.bounds.lower))*space;
      
      naabb.l.vx.x[i] = space.l.vx.x;
      naabb.l.vx.y[i] = space.l.vx.y;
      naabb.l.vx.z[i] = space.l.vx.z;
      
      naabb.l.vy.x[i] = space.l.vy.x;
      naabb.l.vy.y[i] = space.l.vy.y;
      naabb.l.vy.z[i] = space.l.vy.z;
      
      naabb.l.vz.x[i] = space.l.vz.x;
      naabb.l.vz.y[i] = space.l.vz.y;
      naabb.l.vz.z[i] = space.l.vz.z;
      
      naabb.p.x[i] = space.p.x;
      naabb.p.y[i] = space.p.y;
      naabb.p.z[i] = space.p.z;
    }
    
    /*! Sets ID of child. */
    __forceinline void setRef(size_t i, const NodeRefPtr<N>& ref) {
      assert(i < N);
      children[i] = ref;
    }
    
    /*! Returns the extent of the bounds of the ith child */
    __forceinline Vec3fa extent(size_t i) const {
      assert(i<N);
      const Vec3fa vx(naabb.l.vx.x[i],naabb.l.vx.y[i],naabb.l.vx.z[i]);
      const Vec3fa vy(naabb.l.vy.x[i],naabb.l.vy.y[i],naabb.l.vy.z[i]);
      const Vec3fa vz(naabb.l.vz.x[i],naabb.l.vz.y[i],naabb.l.vz.z[i]);
      return rsqrt(vx*vx + vy*vy + vz*vz);
    }
    
    /*! Returns reference to specified child */
    __forceinline       NodeRefPtr<N>& child(size_t i)       { assert(i<N); return children[i]; }
    __forceinline const NodeRefPtr<N>& child(size_t i) const { assert(i<N); return children[i]; }
    
    /*! output operator */
    friend std::ostream& operator<<(std::ostream& o, const UnalignedNode_t<N>& n)
    {
      o << "UnAlignedNode { " << n.naabb << " } " << std::endl;
      return o;
    }
    
  public:
    AffineSpace3vf<N> naabb;   //!< non-axis aligned bounding boxes (bounds are [0,1] in specified space)
  };
}
