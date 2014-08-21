// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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
#include "geometry/bezier1.h"
#include "geometry/bezier1i.h"

namespace embree
{
  /*! BVH4 with unaligned bounds. */
  class BVH4Hair : public Bounded
  {
    ALIGNED_CLASS;
  public:

    /*! forward declaration of node type */
    struct Node;
    struct AlignedNode;
    struct UnalignedNode;
    struct AlignedNodeMB;
    struct UnalignedNodeMB;
    
    /*! branching width of the tree */
    static const size_t N = 4;

    /*! Number of address bits the Node and primitives are aligned
        to. Maximally 2^alignment-2 many primitive blocks per leaf are
        supported. */
    static const size_t alignment = 4;

    /*! Masks the bits that store the number of items per leaf. */
    static const size_t align_mask = (1 << alignment)-1;  
    static const size_t items_mask = (1 << (alignment-1))-1;  

    /*! Empty node */
    static const size_t emptyNode = 4;

    /*! Invalid node, used as marker in traversal */
    static const size_t invalidNode = (((size_t)-1) & (~items_mask)) | emptyNode;
      
    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 32;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+16;
    static const size_t maxDepth = maxBuildDepthLeaf+maxBuildDepthLeaf+maxBuildDepth;   
 
    /*! Maximal number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = items_mask-emptyNode;

    /*! Cost of one traversal step. */
    static const int travCostAligned = 2;
    static const int travCostUnaligned = 3; // FIXME: find best cost
    static const int intCost = 6;

    /*! Pointer that points to a node or a list of primitives */
    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (size_t ptr) : ptr(ptr) { }

      /*! Cast to size_t */
      __forceinline operator size_t() const { return ptr; }

      /*! Prefetches the node this reference points to */
      __forceinline void prefetch() const 
      {
	prefetchL1(((char*)ptr)+0*64);
	prefetchL1(((char*)ptr)+1*64);
	prefetchL1(((char*)ptr)+2*64);
	prefetchL1(((char*)ptr)+3*64);
      }

      __forceinline void prefetch_L2() const 
      {
	prefetchL2(((char*)ptr)+0*64);
	prefetchL2(((char*)ptr)+1*64);
	prefetchL2(((char*)ptr)+2*64);
	prefetchL2(((char*)ptr)+3*64);
      }

      /*! checks if this is a leaf */
      __forceinline int isLeaf() const { return (ptr & (size_t)align_mask) >= emptyNode; }

      /*! checks if this is a node */
      __forceinline int isNode() const { return (ptr & (size_t)align_mask) < emptyNode; }
      
      /*! checks if this is a node with aligned bounding boxes */
      __forceinline int isAlignedNode() const { return (ptr & (size_t)align_mask) == 0; }

      /*! checks if this is a node with unaligned bounding boxes */
      __forceinline int isUnalignedNode() const { return (ptr & (size_t)align_mask) == 1; }

      /*! checks if this is a node with aligned bounding boxes */
      __forceinline int isAlignedNodeMB() const { return (ptr & (size_t)align_mask) == 2; }

      /*! checks if this is a motion blur node with unaligned bounding boxes */
      __forceinline int isUnalignedNodeMB() const { return (ptr & (size_t)align_mask) == 3; }
      
      /*! returns node pointer */
      __forceinline       Node* node()       { assert(isNode()); return (      Node*)((size_t)ptr & ~(size_t)align_mask); }
      __forceinline const Node* node() const { assert(isNode()); return (const Node*)((size_t)ptr & ~(size_t)align_mask); }

      /*! returns aligned node pointer */
      __forceinline       AlignedNode* alignedNode()       { assert(isAlignedNode()); return (      AlignedNode*)ptr; }
      __forceinline const AlignedNode* alignedNode() const { assert(isAlignedNode()); return (const AlignedNode*)ptr; }

      /*! returns unaligned node pointer */
      __forceinline       UnalignedNode* unalignedNode()       { assert(isUnalignedNode()); return (      UnalignedNode*)((size_t)ptr & ~(size_t)align_mask); }
      __forceinline const UnalignedNode* unalignedNode() const { assert(isUnalignedNode()); return (const UnalignedNode*)((size_t)ptr & ~(size_t)align_mask); }

      /*! returns aligned motion blur node pointer */
      __forceinline       AlignedNodeMB* alignedNodeMB()       { assert(isAlignedNodeMB()); return (      AlignedNodeMB*)((size_t)ptr & ~(size_t)align_mask); }
      __forceinline const AlignedNodeMB* alignedNodeMB() const { assert(isAlignedNodeMB()); return (const AlignedNodeMB*)((size_t)ptr & ~(size_t)align_mask); }

      /*! returns unaligned motion blur node pointer */
      __forceinline       UnalignedNodeMB* unalignedNodeMB()       { assert(isUnalignedNodeMB()); return (      UnalignedNodeMB*)((size_t)ptr & ~(size_t)align_mask); }
      __forceinline const UnalignedNodeMB* unalignedNodeMB() const { assert(isUnalignedNodeMB()); return (const UnalignedNodeMB*)((size_t)ptr & ~(size_t)align_mask); }
      
      /*! returns leaf pointer */
      __forceinline char* leaf(size_t& num) const {
        assert(isLeaf());
        num = (ptr & (size_t)items_mask)-emptyNode;
        return (char*)(ptr & ~(size_t)align_mask);
      }

    private:
      size_t ptr;
    };

    /*! Base Node structure */
    struct Node
    {
      /*! Clears the node. */
      __forceinline void clear() {
        for (size_t i=0; i<N; i++) children[i] = emptyNode;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
        assert(i < N);
        children[i] = childID;
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

    public:
      NodeRef children[N];   //!< Pointer to the children (can be a node or leaf)
    };

    /*! Node with aligned bounds */
    struct AlignedNode : public Node
    {
      enum { stride = sizeof(ssef) };

      /*! Clears the node. */
      __forceinline void clear() {
        lower_x = lower_y = lower_z = pos_inf; 
        upper_x = upper_y = upper_z = neg_inf;
        Node::clear();
      }

      /*! Sets bounding box. */
      __forceinline void set(const size_t i, const BBox3fa& bounds) 
      {
        assert(i < N);
        lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
        upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
      }

      /*! Sets ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
        Node::set(i,childID);
      }

      /*! Returns bounds of specified child. */
      __forceinline const BBox3fa bounds(const size_t i) const { 
        assert(i < N);
        const Vec3fa lower(lower_x[i],lower_y[i],lower_z[i]);
        const Vec3fa upper(upper_x[i],upper_y[i],upper_z[i]);
        return BBox3fa(lower,upper);
      }

      /*! returns 4 bounding boxes */
      __forceinline const BBoxSSE3f getBounds(const size_t nearX, const size_t nearY, const size_t nearZ) const 
      {
        const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
        const ssef nearx = load4f((const char*)&lower_x+nearX);
        const ssef neary = load4f((const char*)&lower_y+nearY);
        const ssef nearz = load4f((const char*)&lower_z+nearZ);
        const ssef farx  = load4f((const char*)&lower_x+farX );
        const ssef fary  = load4f((const char*)&lower_y+farY );
        const ssef farz  = load4f((const char*)&lower_z+farZ );
        return BBoxSSE3f(sse3f(nearx,neary,nearz),sse3f(farx,fary,farz));
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend(const size_t i) const {
        assert(i < N);
        return bounds(i).size();
      }

    public:
      ssef lower_x;           //!< X dimension of lower bounds of all 4 children.
      ssef upper_x;           //!< X dimension of upper bounds of all 4 children.
      ssef lower_y;           //!< Y dimension of lower bounds of all 4 children.
      ssef upper_y;           //!< Y dimension of upper bounds of all 4 children.
      ssef lower_z;           //!< Z dimension of lower bounds of all 4 children.
      ssef upper_z;           //!< Z dimension of upper bounds of all 4 children.
    };

    /*! Node with unaligned bounds */
    struct UnalignedNode : public Node
    {
      /*! Clears the node. */
      __forceinline void clear() 
      {
	AffineSpace3fa empty = AffineSpace3fa::scale(Vec3fa(1E+19));
	naabb.l.vx = empty.l.vx;
	naabb.l.vy = empty.l.vy;
	naabb.l.vz = empty.l.vz;
	naabb.p    = empty.p;
        Node::clear();
      }

      /*! Sets bounding box. */
      __forceinline void set(size_t i, const NAABBox3fa& b) 
      {
        assert(i < N);

        AffineSpace3fa space = b.space;
        space.p -= b.bounds.lower;
        space = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19),b.bounds.upper-b.bounds.lower))*space;
        
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
      __forceinline void set(size_t i, const NodeRef& childID) {
        Node::set(i,childID);
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend(size_t i) const {
        assert(i<N);
        const Vec3fa vx(naabb.l.vx.x[i],naabb.l.vx.y[i],naabb.l.vx.z[i]);
        const Vec3fa vy(naabb.l.vy.x[i],naabb.l.vy.y[i],naabb.l.vy.z[i]);
        const Vec3fa vz(naabb.l.vz.x[i],naabb.l.vz.y[i],naabb.l.vz.z[i]);
        const Vec3fa p (naabb.p   .x[i],naabb.p   .y[i],naabb.p   .z[i]);
        return rsqrt(vx*vx + vy*vy + vz*vz);
      }

    public:
      AffineSpaceSSE3f naabb;   //!< non-axis aligned bounding boxes (bounds are [0,1] in specified space)
    };

    /*! Motion blur node with aligned bounds */
    struct AlignedNodeMB : public Node
    {
      enum { stride = sizeof(ssef) };

      /*! Clears the node. */
      __forceinline void clear() {
        lower0_x = lower0_y = lower0_z = pos_inf; 
        upper0_x = upper0_y = upper0_z = neg_inf;
        lower1_x = lower1_y = lower1_z = pos_inf; 
        upper1_x = upper1_y = upper1_z = neg_inf;
        Node::clear();
      }

      /*! Sets bounding box. */
      __forceinline void set(const size_t i, const BBox3fa& b0, const BBox3fa& b1) 
      {
        assert(i < N);
        lower0_x[i] = b0.lower.x; lower0_y[i] = b0.lower.y; lower0_z[i] = b0.lower.z;
        upper0_x[i] = b0.upper.x; upper0_y[i] = b0.upper.y; upper0_z[i] = b0.upper.z;
        lower1_x[i] = b1.lower.x; lower1_y[i] = b1.lower.y; lower1_z[i] = b1.lower.z;
        upper1_x[i] = b1.upper.x; upper1_y[i] = b1.upper.y; upper1_z[i] = b1.upper.z;
      }

      /*! Sets ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
        Node::set(i,childID);
      }

      /*! Returns bounds of specified child. */
      __forceinline const BBox3fa bounds0(const size_t i) const { 
        assert(i < N);
        const Vec3fa lower(lower0_x[i],lower0_y[i],lower0_z[i]);
        const Vec3fa upper(upper0_x[i],upper0_y[i],upper0_z[i]);
        return BBox3fa(lower,upper);
      }

      /*! returns 4 bounding boxes */
      __forceinline const BBoxSSE3f getBounds(const float t, const size_t nearX, const size_t nearY, const size_t nearZ) const 
      {
        const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
        const ssef near0x = load4f((const char*)&lower0_x+nearX);
        const ssef near0y = load4f((const char*)&lower0_y+nearY);
        const ssef near0z = load4f((const char*)&lower0_z+nearZ);
        const ssef far0x  = load4f((const char*)&lower0_x+farX );
        const ssef far0y  = load4f((const char*)&lower0_y+farY );
        const ssef far0z  = load4f((const char*)&lower0_z+farZ );

        const ssef near1x = load4f((const char*)&lower1_x+nearX);
        const ssef near1y = load4f((const char*)&lower1_y+nearY);
        const ssef near1z = load4f((const char*)&lower1_z+nearZ);
        const ssef far1x  = load4f((const char*)&lower1_x+farX );
        const ssef far1y  = load4f((const char*)&lower1_y+farY );
        const ssef far1z  = load4f((const char*)&lower1_z+farZ );

        const ssef nearx = (1.0f-t)*near0x+t*near1x;
        const ssef neary = (1.0f-t)*near0y+t*near1y;
        const ssef nearz = (1.0f-t)*near0z+t*near1z;
        const ssef farx  = (1.0f-t)*far0x+t*far1x;
        const ssef fary  = (1.0f-t)*far0y+t*far1y;
        const ssef farz  = (1.0f-t)*far0z+t*far1z;
        return BBoxSSE3f(sse3f(nearx,neary,nearz),sse3f(farx,fary,farz));
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend0(const size_t i) const {
        assert(i < N);
        return bounds0(i).size();
      }

    public:
      ssef lower0_x;           //!< X dimension of lower bounds of all 4 children.
      ssef upper0_x;           //!< X dimension of upper bounds of all 4 children.
      ssef lower0_y;           //!< Y dimension of lower bounds of all 4 children.
      ssef upper0_y;           //!< Y dimension of upper bounds of all 4 children.
      ssef lower0_z;           //!< Z dimension of lower bounds of all 4 children.
      ssef upper0_z;           //!< Z dimension of upper bounds of all 4 children.

      ssef lower1_x;           //!< X dimension of lower bounds of all 4 children.
      ssef upper1_x;           //!< X dimension of upper bounds of all 4 children.
      ssef lower1_y;           //!< Y dimension of lower bounds of all 4 children.
      ssef upper1_y;           //!< Y dimension of upper bounds of all 4 children.
      ssef lower1_z;           //!< Z dimension of lower bounds of all 4 children.
      ssef upper1_z;           //!< Z dimension of upper bounds of all 4 children.
    };

    /*! Motion blur node with unaligned bounds */
    struct UnalignedNodeMB : public Node
    {
      /*! Clears the node. */
      __forceinline void clear() 
      {
        space0 = space1 = one;
        t0s0.lower = t0s0.upper = Vec3fa(1E10);
        t1s0_t0s1.lower = t1s0_t0s1.upper = Vec3fa(zero);
        t1s1.lower = t1s1.upper = Vec3fa(1E10);
        Node::clear();
      }

      /*! Sets spaces. */
      __forceinline void set(size_t i, const AffineSpace3fa& s0, const AffineSpace3fa& s1) 
      {
        assert(i < N);

        space0.l.vx.x[i] = s0.l.vx.x; space0.l.vx.y[i] = s0.l.vx.y; space0.l.vx.z[i] = s0.l.vx.z; 
        space0.l.vy.x[i] = s0.l.vy.x; space0.l.vy.y[i] = s0.l.vy.y; space0.l.vy.z[i] = s0.l.vy.z;
        space0.l.vz.x[i] = s0.l.vz.x; space0.l.vz.y[i] = s0.l.vz.y; space0.l.vz.z[i] = s0.l.vz.z; 
        space0.p   .x[i] = s0.p   .x; space0.p   .y[i] = s0.p   .y; space0.p   .z[i] = s0.p   .z; 

        space1.l.vx.x[i] = s1.l.vx.x; space1.l.vx.y[i] = s1.l.vx.y; space1.l.vx.z[i] = s1.l.vx.z;
        space1.l.vy.x[i] = s1.l.vy.x; space1.l.vy.y[i] = s1.l.vy.y; space1.l.vy.z[i] = s1.l.vy.z;
        space1.l.vz.x[i] = s1.l.vz.x; space1.l.vz.y[i] = s1.l.vz.y; space1.l.vz.z[i] = s1.l.vz.z;
        space1.p   .x[i] = s1.p   .x; space1.p   .y[i] = s1.p   .y; space1.p   .z[i] = s1.p   .z; 
      }

      /*! Sets bounding boxes. */
      __forceinline void set(size_t i, const BBox3fa& a, const BBox3fa& b, const BBox3fa& c)
      {
        assert(i < N);

        t0s0.lower.x[i] = a.lower.x; t0s0.lower.y[i] = a.lower.y; t0s0.lower.z[i] = a.lower.z;
        t0s0.upper.x[i] = a.upper.x; t0s0.upper.y[i] = a.upper.y; t0s0.upper.z[i] = a.upper.z;

        t1s0_t0s1.lower.x[i] = b.lower.x; t1s0_t0s1.lower.y[i] = b.lower.y; t1s0_t0s1.lower.z[i] = b.lower.z;
        t1s0_t0s1.upper.x[i] = b.upper.x; t1s0_t0s1.upper.y[i] = b.upper.y; t1s0_t0s1.upper.z[i] = b.upper.z;

        t1s1.lower.x[i] = c.lower.x; t1s1.lower.y[i] = c.lower.y; t1s1.lower.z[i] = c.lower.z;
        t1s1.upper.x[i] = c.upper.x; t1s1.upper.y[i] = c.upper.y; t1s1.upper.z[i] = c.upper.z;
      }

      /*! Sets ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
        Node::set(i,childID);
      }

      /*! Returns bounds of specified child. */
      __forceinline const BBox3fa bounds0(const size_t i) const { 
        assert(i < N);
        const Vec3fa lower(t0s0.lower.x[i],t0s0.lower.y[i],t0s0.lower.z[i]);
        const Vec3fa upper(t0s0.upper.x[i],t0s0.upper.y[i],t0s0.upper.z[i]);
        return BBox3fa(lower,upper);
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend0(size_t i) const {
        assert(i < N);
        return bounds0(i).size();
      }

    public:
      AffineSpaceSSE3f space0;   
      AffineSpaceSSE3f space1;   
      BBoxSSE3f t0s0;
      BBoxSSE3f t1s0_t0s1;
      BBoxSSE3f t1s1;
    };

  public:

    /*! BVH4Hair default constructor. */
    BVH4Hair (const PrimitiveType& primTy, Scene* scene);

    /*! BVH4Hair destruction */
    ~BVH4Hair ();

    /*! BVH4Hair instantiations */
    static Accel* BVH4HairBezier1(Scene* scene, bool highQuality);
    static Accel* BVH4HairBezier1i(Scene* scene, bool highQuality);
    static Accel* BVH4HairBezier1iMB(Scene* scene, bool highQuality);

    /*! initializes the acceleration structure */
    void init (size_t numPrimitivesMin = 0, size_t numPrimitivesMax = 0);

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

    /*! allocates a new aligned node */
    __forceinline AlignedNodeMB* allocAlignedNodeMB(size_t thread) {
      AlignedNodeMB* node = (AlignedNodeMB*) alloc.malloc(thread,sizeof(AlignedNodeMB),1 << 4); node->clear(); return node;
    }

    /*! allocates a new unaligned node */
    __forceinline UnalignedNodeMB* allocUnalignedNodeMB(size_t thread) {
      UnalignedNodeMB* node = (UnalignedNodeMB*) alloc.malloc(thread,sizeof(UnalignedNodeMB),1 << 4); node->clear(); return node;
    }

    /*! allocates a block of primitives */
    __forceinline char* allocPrimitiveBlocks(size_t thread, size_t num) {
      return (char*) alloc.malloc(thread,num*primTy.bytes,1 << 4);
    }

    /*! Encodes an aligned node */
    __forceinline NodeRef encodeNode(AlignedNode* node) { 
      return NodeRef((size_t) node);
    }

    /*! Encodes an unaligned node */
    __forceinline NodeRef encodeNode(UnalignedNode* node) { 
      return NodeRef(((size_t) node) | 1);
    }

    /*! Encodes an aligned node */
    __forceinline NodeRef encodeNode(AlignedNodeMB* node) { 
      return NodeRef(((size_t) node) | 2);
    }

    /*! Encodes an unaligned node */
    __forceinline NodeRef encodeNode(UnalignedNodeMB* node) { 
      return NodeRef(((size_t) node) | 3);
    }
    
    /*! Encodes a leaf */
    __forceinline NodeRef encodeLeaf(char* data, size_t num) {
      assert(!((size_t)data & align_mask)); 
      return NodeRef((size_t)data | (emptyNode+min(num,(size_t)maxLeafBlocks)));
    }

  public:
    const PrimitiveType& primTy;       //!< primitive type stored in the BVH
    Scene* scene;
    NodeRef root;  //!< Root node
    size_t numPrimitives;
    size_t numVertices;
  };
}
