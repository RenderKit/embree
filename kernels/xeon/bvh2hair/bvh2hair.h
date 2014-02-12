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
    struct AlignedNode;
    struct UnalignedNode;

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
    static const size_t invalidNode = (((size_t)-1) & (~items_mask)) | 2;
      
    /*! Maximal depth of the BVH. */
    static const size_t maxDepth = 32;
    static const size_t maxBuildDepth = 32;
    
    /*! Maximal number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = items_mask-2;

    /*! Cost of one traversal step. */
    static const int travCostAligned = 1;
    static const int travCostUnaligned = 2;
    static const int intCost = 5;

    /*! Pointer that points to a node or a list of primitives */
    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (size_t ptr) : ptr(ptr) { }

      /*! Cast to size_t */
      __forceinline operator size_t() const { return ptr; }

      /*! checks if this is a leaf */
      __forceinline int isLeaf() const { return (ptr & (size_t)align_mask) > 1; }
      
      /*! checks if this is a node with aligned bounding boxes */
      __forceinline int isAlignedNode() const { return (ptr & (size_t)align_mask) == 0; }

      /*! checks if this is a node with aligned bounding boxes */
      __forceinline int isUnalignedNode() const { return (ptr & (size_t)align_mask) == 1; }
      
      /*! returns aligned node pointer */
      __forceinline       AlignedNode* alignedNode()       { assert(isAlignedNode()); return (      AlignedNode*)ptr; }
      __forceinline const AlignedNode* alignedNode() const { assert(isAlignedNode()); return (const AlignedNode*)ptr; }

      /*! returns unaligned node pointer */
      __forceinline       UnalignedNode* unalignedNode()       { assert(isUnalignedNode()); return (      UnalignedNode*)((size_t)ptr & ~(size_t)align_mask); }
      __forceinline const UnalignedNode* unalignedNode() const { assert(isUnalignedNode()); return (const UnalignedNode*)((size_t)ptr & ~(size_t)align_mask); }
      
      /*! returns leaf pointer */
      __forceinline char* leaf(size_t& num) const {
        assert(isLeaf());
        num = (ptr & (size_t)items_mask)-2;
        return (char*)(ptr & ~(size_t)align_mask);
      }

    private:
      size_t ptr;
    };

    /*! Non-axis aligned bounds */
    struct NAABBox3fa
    {
    public:
      
      __forceinline NAABBox3fa () {}

      __forceinline NAABBox3fa (EmptyTy) 
        : space(one), bounds(empty) {}
      
      __forceinline NAABBox3fa (const BBox3fa& bounds) 
        : space(one), bounds(bounds) {}
      
      __forceinline NAABBox3fa (const AffineSpace3fa& space, const BBox3fa& bounds) 
        : space(space), bounds(bounds) {}

      friend std::ostream& operator<<(std::ostream& cout, const NAABBox3fa& p) {
        return std::cout << "{ space = " << p.space << ", bounds = " << p.bounds << "}";
      }
      
    public:
      AffineSpace3fa space; //!< orthonormal transformation
      BBox3fa bounds;       //!< bounds in transformed space // FIXME: one could merge this into above transformation, however, this causes problems with curve radius
    };

    /*! Node with aligned bounds */
    struct AlignedNode
    {
      /*! Clears the node. */
      __forceinline void clear() {
        aabb[0] = aabb[1] = empty;
        children[0] = children[1] = emptyNode;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const BBox3fa& b, const NodeRef& childID) {
        assert(i < 2);
        aabb[i] = b;
        children[i] = childID;
      }

      /*! Returns bounds of specified child. */
      __forceinline const BBox3fa& bounds(size_t i) const { assert(i < 2); return aabb[i]; }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<2); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<2); return children[i]; }

    public:
      BBox3fa aabb    [2];   //!< left and right non-axis aligned bounding box
      NodeRef children[2];   //!< Pointer to the 2 children (can be a node or leaf)
    };

    /*! Node with unaligned bounds */
    struct UnalignedNode
    {
      /*! Clears the node. */
      __forceinline void clear() {
        naabb[0] = naabb[1] = empty;
        children[0] = children[1] = emptyNode;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const NAABBox3fa& b, const NodeRef& childID) {
        assert(i < 2);
        naabb[i] = b;
        children[i] = childID;
      }

      /*! Returns bounds of specified child. */
      __forceinline const NAABBox3fa& bounds(size_t i) const { assert(i < 2); return naabb[i]; }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<2); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<2); return children[i]; }

    public:
      NAABBox3fa naabb   [2];   //!< left and right non-axis aligned bounding box
      NodeRef    children[2];   //!< Pointer to the 2 children (can be a node or leaf)
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
      __forceinline const BBox3fa bounds() const {
        const BBox3fa b = merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2),BBox3fa(p3));
        return enlarge(b,Vec3fa(b.upper.w));
      }

      /*! calculate bounds in specified coordinate space */
      __forceinline const BBox3fa bounds(const AffineSpace3fa& space) const 
      {
        const BBox3fa b0 = xfmPoint(space,p0);
        const BBox3fa b1 = xfmPoint(space,p1);
        const BBox3fa b2 = xfmPoint(space,p2);
        const BBox3fa b3 = xfmPoint(space,p3);
        const BBox3fa b = merge(b0,b1,b2,b3);
        const float   r = max(p0.w,p1.w,p2.w,p3.w);
        return enlarge(b,Vec3fa(r));
      }
      
    public:
      Vec3fa p0;            //!< 1st control point (x,y,z,r)
      Vec3fa p1;            //!< 2nd control point (x,y,z,r)
      Vec3fa p2;            //!< 3rd control point (x,y,z,r)
      Vec3fa p3;            //!< 4th control point (x,y,z,r)
      float t0,dt;          //!< t range of this sub-curve
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

    /*! allocates a new aligned node */
    __forceinline AlignedNode* allocAlignedNode(size_t thread) {
      AlignedNode* node = (AlignedNode*) alloc.malloc(thread,sizeof(AlignedNode),1 << 4); node->clear(); return node;
    }

    /*! allocates a new unaligned node */
    __forceinline UnalignedNode* allocUnalignedNode(size_t thread) {
      UnalignedNode* node = (UnalignedNode*) alloc.malloc(thread,sizeof(UnalignedNode),1 << 4); node->clear(); return node;
    }

    /*! allocates a block of primitives */
    __forceinline char* allocPrimitiveBlocks(size_t thread, size_t num) {
      return (char*) alloc.malloc(thread,num*sizeof(Bezier1),1 << 4);
    }

    /*! Encodes an alingned node */
    __forceinline NodeRef encodeNode(AlignedNode* node) { 
      return NodeRef((size_t) node);
    }

    /*! Encodes an unaligned node */
    __forceinline NodeRef encodeNode(UnalignedNode* node) { 
      return NodeRef(((size_t) node) | 1);
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
