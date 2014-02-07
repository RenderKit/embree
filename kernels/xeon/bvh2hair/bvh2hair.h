// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "embree2/rtcore.h"
#include "common/alloc.h"
#include "common/accel.h"
#include "common/scene.h"
#include "geometry/primitive.h"

namespace embree
{
  /*! BVH2 with rotated bounds. */
  class BVH2Hair : public Bounded
  {
  public:
    
    /*! forward declaration of node type */
    struct Node;

    /*! branching width of the tree */
    static const size_t N = 1;

    /*! Number of address bits the Node and primitives are aligned
        to. Maximally 2^alignment-2 many primitive blocks per leaf are
        supported. */
    static const size_t alignment = 4;

    /*! Masks the bits that store the number of items per leaf. */
    static const size_t align_mask = (1 << alignment)-1;  
    static const size_t items_mask = (1 << (alignment-1))-1;  

    /*! Empty node */
    static const size_t emptyNode = 1;

    /*! Invalid node, used as marker in traversal */
    static const size_t invalidNode = (((size_t)-1) & (~items_mask)) | 1;
      
    /*! Maximal depth of the BVH. */
    static const size_t maxDepth = 32;
    
    /*! Maximal number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = items_mask-1;

    /*! Cost of one traversal step. */
    static const int travCost = 1;

    /*! Pointer that points to a node or a list of primitives */
    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (size_t ptr) : ptr(ptr) { }

      /*! checks if this is a leaf */
      __forceinline int isLeaf() const { return (ptr & (size_t)align_mask) != 0; }
      
      /*! checks if this is a node */
      __forceinline int isNode() const { return (ptr & (size_t)align_mask) == 0; }
      
      /*! returns node pointer */
      __forceinline       Node* node()       { assert(isNode()); return (      Node*)ptr; }
      __forceinline const Node* node() const { assert(isNode()); return (const Node*)ptr; }
      
      /*! returns leaf pointer */
      __forceinline char* leaf(size_t& num) const {
        assert(isLeaf());
        num = (ptr & (size_t)items_mask)-1;
        return (char*)(ptr & ~(size_t)align_mask);
      }

    private:
      size_t ptr;
    };

    /*! Non-axis aligned bounds */
    struct NAABBox3fa
    {
    public:
      __forceinline NAABBox3fa (const BBox3fa& bounds) 
        : xfm(one), bounds(bounds) {}
      
      __forceinline NAABBox3fa (const AffineSpace3f& xfm, const BBox3fa& bounds) 
        : xfm(xfm), bounds(bounds) {}
      
    public:
      AffineSpace3f xfm;   //!< orthonormal transformation
      BBox3fa bounds;      //!< bounds in transformed space // FIXME: one could merge this into above transformation, however, this causes problems with curve radius
    };

    /*! BVH2Hair Node */
    struct Node
    {
      /*! Clears the node. */
      __forceinline void clear() {
        naabb[0] = naabb[1] = one;
        children[0] = children[1] = emptyNode;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const NAABBox3f& b, const NodeRef& childID) {
        assert(i < 2);
        naabb[i] = rcp(b);
        children[i] = childID;
      }

      /*! Returns bounds of specified child. */
      __forceinline const NAABBox3f& bounds(size_t i) const { assert(i < 2); return naabb[2]; }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<2); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<2); return children[i]; }

    public:
      NAABBox3f naabb   [2];   //!< left and right non-axis aligned bounding box
      NodeRef   children[2];   //!< Pointer to the 2 children (can be a node or leaf)
    };

    /*! Hair Leaf */
    struct Bezier1
    {
    public:

      /*! Default constructor. */
      __forceinline Bezier1 () {}

      /*! Construction from vertices and IDs. */
      __forceinline Bezier1 (const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const Vec3fa& p3, const float t0, const float t1,
                             const unsigned int geomID, const unsigned int primID)
        : p0(p0), p1(p1), p2(p2), p3(p3), t0(t0), dt(t1-t0), geomID(geomID), primID(primID) {}

      /*! calculate the bounds of the curve */
      __forceinline BBox3fa bounds() const {
        const BBox3fa b = merge(BBox3fa(p[0]),BBox3fa(p[1]),BBox3fa(p[2]),BBox3fa(p[3]));
        return enlarge(b,Vec3fa(b.upper.w));
      }

    public:
      Vec3fa p0;            //!< 1st control point (x,y,z,r)
      Vec3fa p1;            //!< 2nd control point (x,y,z,r)
      Vec3fa p2;            //!< 3rd control point (x,y,z,r)
      Vec3fa p3;            //!< 4th control point (x,y,z,r)
      float t0,t1;          //!< t range of this sub-curve
      unsigned int geomID;  //!< geometry ID
      unsigned int primID;  //!< primitive ID
    };

  public:

    /*! BVH2Hair default constructor. */
    BVH2Hair ();

    /*! BVH2Hair destruction */
    ~BVH2Hair ();

    /*! BVH2Hair instantiations */
    static Accel* BVH2HairBezier1(Scene* scene);

    /*! initializes the acceleration structure */
    void init (size_t numPrimitives = 0);

    /*! allocator for nodes */
    LinearAllocatorPerThread alloc;

    /*! allocates a new node */
    __forceinline Node* allocNode(size_t thread) {
      Node* node = (Node*) alloc.malloc(thread,sizeof(Node),1 << 4); node->clear(); return node;
    }

    /*! allocates a block of primitives */
    __forceinline char* allocPrimitiveBlocks(size_t thread, size_t num) {
      return (char*) alloc.malloc(thread,num*primTy.bytes,1 << 4);
    }

    /*! Encodes a node */
    __forceinline NodeRef encodeNode(Node* node) { 
      return NodeRef((size_t) node);
    }
    
    /*! Encodes a leaf */
    __forceinline NodeRef encodeLeaf(char* data, size_t num) {
      assert(!((size_t)data & align_mask)); 
      return NodeRef((size_t)data | (1+min(num,(size_t)maxLeafBlocks)));
    }

  public:
    NodeRef root;  //!< Root node
  };
}
