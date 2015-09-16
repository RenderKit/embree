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

#include "../../common/default.h"
#include "../../common/alloc.h"
#include "../../common/accel.h"
#include "../../common/device.h"
#include "../../common/scene.h"
#include "../geometry/primitive.h"
#include "../../common/ray.h"

namespace embree
{
  /*! Multi BVH with 4 children. Each node stores the bounding box of
   * it's 4 children as well as 4 child pointers. */
  class BVH4 : public AccelData
  {
    ALIGNED_CLASS;
  public:
    
    /*! forward declaration of node type */
    struct BaseNode;
    struct Node;
    struct NodeMB;
    struct UnalignedNode;
    struct UnalignedNodeMB;

    /*! branching width of the tree */
    static const size_t N = 4;
    
    /*! Number of address bits the Node and primitives are aligned
        to. Maximally 2^alignment-1 many primitive blocks per leaf are
        supported. */
    static const size_t alignment = 4;

    /*! highest address bit is used as barrier for some algorithms */
    static const size_t barrier_mask = (1LL << (8*sizeof(size_t)-1));

    /*! Masks the bits that store the number of items per leaf. */
    static const size_t align_mask = (1 << alignment)-1;  
    static const size_t items_mask = (1 << alignment)-1;  

    /*! different supported node types */
    static const size_t tyNode = 0;
    static const size_t tyNodeMB = 1;
    static const size_t tyUnalignedNode = 2;
    static const size_t tyUnalignedNodeMB = 3;
    static const size_t tyLeaf = 8;

    /*! Empty node */
    static const size_t emptyNode = tyLeaf;

    /*! Invalid node, used as marker in traversal */
    static const size_t invalidNode = (((size_t)-1) & (~items_mask)) | tyLeaf;
      
    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 32;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+16;
    static const size_t maxDepth = maxBuildDepthLeaf+maxBuildDepthLeaf+maxBuildDepth;
    
    /*! Maximal number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = items_mask-tyLeaf;

    /*! Cost of one traversal step. */
    static const int travCost = 1;
    static const int travCostAligned = 1;
    static const int travCostUnaligned = 3; // FIXME: find best cost
    static const int intCost = 1; // set to 1 for statistics // FIXME: is this used? was 6;

    /*! flags used to enable specific node types in intersectors */
    enum NodeFlags {  // FIXME: use these flags also in intersector implementations, currently hardcoded constants are used
      FLAG_ALIGNED_NODE = 0x0001,
      FLAG_ALIGNED_NODE_MB = 0x0010,
      FLAG_UNALIGNED_NODE = 0x0100,
      FLAG_UNALIGNED_NODE_MB = 0x1000,
      FLAG_NODE_MB = FLAG_ALIGNED_NODE_MB | FLAG_UNALIGNED_NODE_MB
    };

    /*! Builder interface to create allocator */
    struct CreateAlloc
    {      
    public:
      __forceinline CreateAlloc (BVH4* bvh) : bvh(bvh) {}
      __forceinline FastAllocator::ThreadLocal2* operator() () const { return bvh->alloc.threadLocal2();  }
    
    private:
      BVH4* bvh;
    };

    /*! Builder interface to create Node */
    struct CreateNode
    {
      __forceinline CreateNode (BVH4* bvh) : bvh(bvh) {}
      
      template<typename BuildRecord>
      __forceinline Node* operator() (const BuildRecord& current, BuildRecord* children, const size_t N, FastAllocator::ThreadLocal2* alloc) 
      {
        Node* node = (Node*) alloc->alloc0.malloc(sizeof(Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i].bounds());
          children[i].parent = (size_t*)&node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH4* bvh;
    };

    struct NoRotate
    {
      __forceinline size_t operator() (BVH4::Node* node, const size_t* counts, const size_t N) {
        return 0;
      }
    };

    /*! Pointer that points to a node or a list of primitives */
    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (size_t ptr) : ptr(ptr) {}

      /*! Cast to size_t */
      __forceinline operator size_t() const { return ptr; }

      /*! Prefetches the node this reference points to */
      __forceinline void prefetch(int types=0) const {
	prefetchL1(((char*)ptr)+0*64);
	prefetchL1(((char*)ptr)+1*64);
	if (types > 0x1) {
	  prefetchL1(((char*)ptr)+2*64);
	  prefetchL1(((char*)ptr)+3*64);
	}
      }

      __forceinline void prefetchL2(int types=0) const {
        embree::prefetchL2(((char*)ptr)+0*64);
	embree::prefetchL2(((char*)ptr)+1*64);
	if (types > 0x1) {
	  embree::prefetchL2(((char*)ptr)+2*64);
	  embree::prefetchL2(((char*)ptr)+3*64);
	}
      }

      __forceinline void prefetchW(int types=0) const {
        embree::prefetchEX(((char*)ptr)+0*64);
	embree::prefetchEX(((char*)ptr)+1*64);
	if (types > 0x1) {
	  embree::prefetchEX(((char*)ptr)+2*64);
	  embree::prefetchEX(((char*)ptr)+3*64);
	}
      }

      /*! Sets the barrier bit. */
      __forceinline void setBarrier() { assert(!isBarrier()); ptr |= barrier_mask; }
      
      /*! Clears the barrier bit. */
      __forceinline void clearBarrier() { ptr &= ~barrier_mask; }

      /*! Checks if this is an barrier. A barrier tells the top level tree rotations how deep to enter the tree. */
      __forceinline bool isBarrier() const { return (ptr & barrier_mask) != 0; }

      /*! checks if this is a leaf */
      __forceinline size_t isLeaf() const { return ptr & tyLeaf; }

      /*! checks if this is a leaf */
      __forceinline int isLeaf(int types) const { 
	if      (types == 0x0001) return !isNode();
	/*else if (types == 0x0010) return !isNodeMB();
	else if (types == 0x0100) return !isUnalignedNode();
	else if (types == 0x1000) return !isUnalignedNodeMB();*/
	else return isLeaf();
      }
      
      /*! checks if this is a node */
      __forceinline int isNode() const { return (ptr & (size_t)align_mask) == tyNode; }
      __forceinline int isNode(int types) const { return (types == 0x1) || ((types & 0x1) && isNode()); }

      /*! checks if this is a motion blur node */
      __forceinline int isNodeMB() const { return (ptr & (size_t)align_mask) == tyNodeMB; }
      __forceinline int isNodeMB(int types) const { return (types == 0x10) || ((types & 0x10) && isNodeMB()); }

      /*! checks if this is a node with unaligned bounding boxes */
      __forceinline int isUnalignedNode() const { return (ptr & (size_t)align_mask) == tyUnalignedNode; }
      __forceinline int isUnalignedNode(int types) const { return (types == 0x100) || ((types & 0x100) && isUnalignedNode()); }

      /*! checks if this is a motion blur node with unaligned bounding boxes */
      __forceinline int isUnalignedNodeMB() const { return (ptr & (size_t)align_mask) == tyUnalignedNodeMB; }
      __forceinline int isUnalignedNodeMB(int types) const { return (types == 0x1000) || ((types & 0x1000) && isUnalignedNodeMB()); }

      /*! returns base node pointer */
      __forceinline BaseNode* baseNode(int types) 
      { 
	assert(!isLeaf()); 
	if (types == 0x1) { 
          assert((ptr & (size_t)align_mask) == 0); 
          return (BaseNode*)ptr; 
        }
	else  
          return (BaseNode*)(ptr & ~(size_t)align_mask); 
      }
      __forceinline const BaseNode* baseNode(int types) const 
      { 
	assert(!isLeaf()); 
	if (types == 0x1) { 
          assert((ptr & (size_t)align_mask) == 0); 
          return (const BaseNode*)ptr; 
        }
	else
          return (const BaseNode*)(ptr & ~(size_t)align_mask); 
      }

      /*! returns node pointer */
      __forceinline       Node* node()       { assert(isNode()); return (      Node*)ptr; }
      __forceinline const Node* node() const { assert(isNode()); return (const Node*)ptr; }

      /*! returns motion blur node pointer */
      __forceinline       NodeMB* nodeMB()       { assert(isNodeMB()); return (      NodeMB*)(ptr & ~(size_t)align_mask); }
      __forceinline const NodeMB* nodeMB() const { assert(isNodeMB()); return (const NodeMB*)(ptr & ~(size_t)align_mask); }

      /*! returns unaligned node pointer */
      __forceinline       UnalignedNode* unalignedNode()       { assert(isUnalignedNode()); return (      UnalignedNode*)(ptr & ~(size_t)align_mask); }
      __forceinline const UnalignedNode* unalignedNode() const { assert(isUnalignedNode()); return (const UnalignedNode*)(ptr & ~(size_t)align_mask); }

      /*! returns unaligned motion blur node pointer */
      __forceinline       UnalignedNodeMB* unalignedNodeMB()       { assert(isUnalignedNodeMB()); return (      UnalignedNodeMB*)(ptr & ~(size_t)align_mask); }
      __forceinline const UnalignedNodeMB* unalignedNodeMB() const { assert(isUnalignedNodeMB()); return (const UnalignedNodeMB*)(ptr & ~(size_t)align_mask); }
            
      /*! returns leaf pointer */
      __forceinline char* leaf(size_t& num) const {
        assert(isLeaf());
        num = (ptr & (size_t)items_mask)-tyLeaf;
        return (char*)(ptr & ~(size_t)align_mask);
      }

      /*! clear all bit flags */
      __forceinline void clearFlags() {
        ptr &= ~(size_t)align_mask;
      }

    private:
      size_t ptr;
    };
    
    /*! BVH4 Base Node */
    struct BaseNode
    {
      /*! Clears the node. */
      __forceinline void clear() {
	for (size_t i=0; i<N; i++) children[i] = emptyNode;
      }

        /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

      /*! verifies the node */
      __forceinline bool verify() const  // FIXME: implement tree verify
      {
	for (size_t i=0; i<BVH4::N; i++) {
	  if (child(i) == BVH4::emptyNode) {
	    for (; i<BVH4::N; i++) {
	      if (child(i) != BVH4::emptyNode)
		return false;
	    }
	    break;
	  }
	}
	return true;
      }

      NodeRef children[N];    //!< Pointer to the 4 children (can be a node or leaf)
    };
    
    /*! BVH4 Node */
    struct Node : public BaseNode
    {
      /*! Clears the node. */
      __forceinline void clear() {
        lower_x = lower_y = lower_z = pos_inf; 
        upper_x = upper_y = upper_z = neg_inf;
	BaseNode::clear();
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
	assert(i < N);
        children[i] = childID;
      }

      /*! Sets bounding box of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds) 
      {
        assert(i < N);
        lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
        upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds, const NodeRef& childID) {
        set(i,bounds);
        children[i] = childID;
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

      /*! Returns bounds of all children */
      __forceinline void bounds(BBox<float4>& bounds0, BBox<float4>& bounds1, BBox<float4>& bounds2, BBox<float4>& bounds3) const {
        transpose(lower_x,lower_y,lower_z,float4(zero),bounds0.lower,bounds1.lower,bounds2.lower,bounds3.lower);
        transpose(upper_x,upper_y,upper_z,float4(zero),bounds0.upper,bounds1.upper,bounds2.upper,bounds3.upper);
      }

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

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }
      
    public:
      float4 lower_x;           //!< X dimension of lower bounds of all 4 children.
      float4 upper_x;           //!< X dimension of upper bounds of all 4 children.
      float4 lower_y;           //!< Y dimension of lower bounds of all 4 children.
      float4 upper_y;           //!< Y dimension of upper bounds of all 4 children.
      float4 lower_z;           //!< Z dimension of lower bounds of all 4 children.
      float4 upper_z;           //!< Z dimension of upper bounds of all 4 children.
    };

    /*! Motion Blur Node */
    struct NodeMB : public BaseNode
    {
      /*! Clears the node. */
      __forceinline void clear()  {
        lower_x = lower_y = lower_z = float4(nan);
        upper_x = upper_y = upper_z = float4(nan);
        lower_dx = lower_dy = lower_dz = float4(nan); // initialize with NAN and update during refit
        upper_dx = upper_dy = upper_dz = float4(nan);
	BaseNode::clear();
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds, NodeRef childID) {
        lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
        upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
        children[i] = childID;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, NodeRef childID) {
	children[i] = childID;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds) {
        lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
        upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds0, const BBox3fa& bounds1) 
      {
        lower_x[i] = bounds0.lower.x; lower_y[i] = bounds0.lower.y; lower_z[i] = bounds0.lower.z;
        upper_x[i] = bounds0.upper.x; upper_y[i] = bounds0.upper.y; upper_z[i] = bounds0.upper.z;

        /*! for empty bounds we have to avoid inf-inf=nan */
        if (unlikely(bounds0.empty())) { 
          lower_dx[i] = lower_dy[i] = lower_dz[i] = zero;
          upper_dx[i] = upper_dy[i] = upper_dz[i] = zero;
        } 
        /*! standard case */
        else {
          const Vec3fa dlower = bounds1.lower-bounds0.lower;
          const Vec3fa dupper = bounds1.upper-bounds0.upper;
          lower_dx[i] = dlower.x; lower_dy[i] = dlower.y; lower_dz[i] = dlower.z;
          upper_dx[i] = dupper.x; upper_dy[i] = dupper.y; upper_dz[i] = dupper.z;
        }
      }

      /*! tests if the node has valid bounds */
      __forceinline bool hasBounds() const {
        return lower_dx.i[0] != cast_f2i(float(nan));
      }

      /*! Return bounding box for time 0 */
      __forceinline BBox3fa bounds0(size_t i) const {
        return BBox3fa(Vec3fa(lower_x[i],lower_y[i],lower_z[i]),
                      Vec3fa(upper_x[i],upper_y[i],upper_z[i]));
      }

      /*! Return bounding box for time 1 */
      __forceinline BBox3fa bounds1(size_t i) const {
        return BBox3fa(Vec3fa(lower_x[i]+lower_dx[i],lower_y[i]+lower_dy[i],lower_z[i]+lower_dz[i]),
                      Vec3fa(upper_x[i]+upper_dx[i],upper_y[i]+upper_dy[i],upper_z[i]+upper_dz[i]));
      }

      /*! Returns extent of bounds of specified child. */
      __forceinline Vec3fa extend0(size_t i) const {
	return bounds0(i).size();
      }

      /*! Returns bounds of node. */
      __forceinline BBox3fa bounds() const {
        return BBox3fa(Vec3fa(reduce_min(min(lower_x,lower_x+lower_dx)),
                             reduce_min(min(lower_y,lower_y+lower_dy)),
                             reduce_min(min(lower_z,lower_z+lower_dz))),
                      Vec3fa(reduce_max(max(upper_x,upper_x+upper_dx)),
                             reduce_max(max(upper_y,upper_y+upper_dy)),
                             reduce_max(max(upper_z,upper_z+upper_dz))));
      }

      /*! swap two children of the node */
      __forceinline void swap(size_t i, size_t j)
      {
	assert(i<N && j<N);
	std::swap(children[i],children[j]);

	std::swap(lower_x[i],lower_x[j]);
	std::swap(upper_x[i],upper_x[j]);
	std::swap(lower_y[i],lower_y[j]);
	std::swap(upper_y[i],upper_y[j]);
	std::swap(lower_z[i],lower_z[j]);
	std::swap(upper_z[i],upper_z[j]);
	
	std::swap(lower_dx[i],lower_dx[j]);
	std::swap(upper_dx[i],upper_dx[j]);
	std::swap(lower_dy[i],lower_dy[j]);
	std::swap(upper_dy[i],upper_dy[j]);
	std::swap(lower_dz[i],lower_dz[j]);
	std::swap(upper_dz[i],upper_dz[j]);
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

    public:
      float4 lower_x;        //!< X dimension of lower bounds of all 4 children.
      float4 upper_x;        //!< X dimension of upper bounds of all 4 children.
      float4 lower_y;        //!< Y dimension of lower bounds of all 4 children.
      float4 upper_y;        //!< Y dimension of upper bounds of all 4 children.
      float4 lower_z;        //!< Z dimension of lower bounds of all 4 children.
      float4 upper_z;        //!< Z dimension of upper bounds of all 4 children.

      float4 lower_dx;        //!< X dimension of lower bounds of all 4 children.
      float4 upper_dx;        //!< X dimension of upper bounds of all 4 children.
      float4 lower_dy;        //!< Y dimension of lower bounds of all 4 children.
      float4 upper_dy;        //!< Y dimension of upper bounds of all 4 children.
      float4 lower_dz;        //!< Z dimension of lower bounds of all 4 children.
      float4 upper_dz;        //!< Z dimension of upper bounds of all 4 children.
    };

    /*! Node with unaligned bounds */
    struct UnalignedNode : public BaseNode
    {
      /*! Clears the node. */
      __forceinline void clear() 
      {
	naabb.l.vx = Vec3fa(nan);
	naabb.l.vy = Vec3fa(nan);
	naabb.l.vz = Vec3fa(nan);
	naabb.p    = Vec3fa(nan);
	BaseNode::clear();
      }

      /*! Sets bounding box. */
      __forceinline void set(size_t i, const OBBox3fa& b) 
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
      __forceinline void set(size_t i, const NodeRef& childID) {
        //Node::set(i,childID);
	assert(i < N);
	children[i] = childID;
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

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

    public:
      AffineSpaceSSE3f naabb;   //!< non-axis aligned bounding boxes (bounds are [0,1] in specified space)
    };

    struct UnalignedNodeMB : public BaseNode
    {
      /*! Clears the node. */
      __forceinline void clear() 
      {
        space0 = one;
        //b0.lower = b0.upper = Vec3fa(nan);
        b1.lower = b1.upper = Vec3fa(nan);
        BaseNode::clear();
      }

      /*! Sets space and bounding boxes. */
      __forceinline void set(size_t i, const AffineSpace3fa& s0, const BBox3fa& a, const BBox3fa& c)
      {
        assert(i < N);

	AffineSpace3fa space = s0;
        space.p -= a.lower;
	Vec3fa scale = 1.0f/max(Vec3fa(1E-19f),a.upper-a.lower);
        space = AffineSpace3fa::scale(scale)*space;
	BBox3fa a1((a.lower-a.lower)*scale,(a.upper-a.lower)*scale);
	BBox3fa c1((c.lower-a.lower)*scale,(c.upper-a.lower)*scale);

        space0.l.vx.x[i] = space.l.vx.x; space0.l.vx.y[i] = space.l.vx.y; space0.l.vx.z[i] = space.l.vx.z; 
        space0.l.vy.x[i] = space.l.vy.x; space0.l.vy.y[i] = space.l.vy.y; space0.l.vy.z[i] = space.l.vy.z;
        space0.l.vz.x[i] = space.l.vz.x; space0.l.vz.y[i] = space.l.vz.y; space0.l.vz.z[i] = space.l.vz.z; 
        space0.p   .x[i] = space.p   .x; space0.p   .y[i] = space.p   .y; space0.p   .z[i] = space.p   .z; 

        /*b0.lower.x[i] = a1.lower.x; b0.lower.y[i] = a1.lower.y; b0.lower.z[i] = a1.lower.z;
	  b0.upper.x[i] = a1.upper.x; b0.upper.y[i] = a1.upper.y; b0.upper.z[i] = a1.upper.z;*/

        b1.lower.x[i] = c1.lower.x; b1.lower.y[i] = c1.lower.y; b1.lower.z[i] = c1.lower.z;
        b1.upper.x[i] = c1.upper.x; b1.upper.y[i] = c1.upper.y; b1.upper.z[i] = c1.upper.z;
      }

      /*! Sets ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
        //Node::set(i,childID);
	assert(i < N);
	children[i] = childID;
      }

      /*! Returns bounds of specified child. */
      __forceinline const BBox3fa bounds0(const size_t i) const { 
        assert(i < N);
        /*const Vec3fa lower(b0.lower.x[i],b0.lower.y[i],b0.lower.z[i]);
        const Vec3fa upper(b0.upper.x[i],b0.upper.y[i],b0.upper.z[i]);
        return BBox3fa(lower,upper);*/
	return empty; // FIXME: not yet implemented
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend0(size_t i) const {
        assert(i < N);
        //return bounds0(i).size();
	return zero; // FIXME: no yet implemented
      }

    public:
      AffineSpaceSSE3f space0;   
      //BBoxSSE3f b0;
      BBoxSSE3f b1;
    };

    /*! swap the children of two nodes */
    __forceinline static void swap(Node* a, size_t i, Node* b, size_t j)
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
    __forceinline static void compact(Node* a)
    {
      /* find right most filled node */
      ssize_t j=N;
      for (j=j-1; j>=0; j--)
        if (a->child(j) != emptyNode)
          break;

      /* replace empty nodes with filled nodes */
      for (ssize_t i=0; i<j; i++) {
        if (a->child(i) == emptyNode) {
          a->swap(i,j);
          for (j=j-1; j>i; j--)
            if (a->child(j) != emptyNode)
              break;
        }
      }
    }

    /*! compacts a node (moves empty children to the end) */
    __forceinline static void compact(NodeMB* a)
    {
      /* find right most filled node */
      ssize_t j=N;
      for (j=j-1; j>=0; j--)
        if (a->child(j) != emptyNode)
          break;

      /* replace empty nodes with filled nodes */
      for (ssize_t i=0; i<j; i++) {
        if (a->child(i) == emptyNode) {
          a->swap(i,j);
          for (j=j-1; j>i; j--)
            if (a->child(j) != emptyNode)
              break;
        }
      }
    }

  public:

    /*! BVH4 default constructor. */
    BVH4 (const PrimitiveType& primTy, Scene* scene, bool listMode);

    /*! BVH4 destruction */
    ~BVH4 ();

    /*! BVH4 instantiations */
    static Accel* BVH4Triangle4vMB(Scene* scene);

    static Accel* BVH4Bezier1v(Scene* scene);
    static Accel* BVH4Bezier1i(Scene* scene);
    
    static Accel* BVH4OBBBezier1v(Scene* scene, bool highQuality);
    static Accel* BVH4OBBBezier1i(Scene* scene, bool highQuality);
    static Accel* BVH4OBBBezier1iMB(Scene* scene, bool highQuality);

    static Accel* BVH4Triangle4(Scene* scene);
    static Accel* BVH4Triangle8(Scene* scene);
    static Accel* BVH4Triangle4v(Scene* scene);
    static Accel* BVH4Triangle4i(Scene* scene);
    static Accel* BVH4SubdivPatch1(Scene* scene);
    static Accel* BVH4SubdivPatch1Cached(Scene* scene);
    static Accel* BVH4SubdivGridEager(Scene* scene);
    static Accel* BVH4UserGeometry(Scene* scene);
    
    static Accel* BVH4BVH4Triangle4ObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle8ObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle4vObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle4iObjectSplit(Scene* scene);

    static Accel* BVH4Triangle4SpatialSplit(Scene* scene);
    static Accel* BVH4Triangle8SpatialSplit(Scene* scene);
    static Accel* BVH4Triangle4ObjectSplit(Scene* scene);
    static Accel* BVH4Triangle8ObjectSplit(Scene* scene);
    static Accel* BVH4Triangle4vObjectSplit(Scene* scene);
    static Accel* BVH4Triangle4iObjectSplit(Scene* scene);

    static Accel* BVH4Triangle4ObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle4vObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle4Refit(TriangleMesh* mesh);

    /*! clears the acceleration structure */
    void clear();

    /*! sets BVH members after build */
    void set (NodeRef root, const BBox3fa& bounds, size_t numPrimitives);

    /*! prints statistics about the BVH */
    void printStatistics();

    /*! Clears the barrier bits of a subtree. */
    void clearBarrier(NodeRef& node);

    /*! lays out N large nodes of the BVH */
    void layoutLargeNodes(size_t N);
    NodeRef layoutLargeNodesRecursion(NodeRef& node);

    /*! calculates the amount of bytes allocated */
    size_t bytesAllocated() {
      return alloc.getAllocatedBytes();
    }

    /*! called by all builders before build starts */
    double preBuild(const char* builderName);

    /*! called by all builders after build ended */
    void postBuild(double t0);

    /*! shrink allocated memory */
    void shrink() {
      alloc.shrink();
    }

    /*! post build cleanup */
    void cleanup() {
      alloc.cleanup();
    }

  public:

    /*! Encodes a node */
    static __forceinline NodeRef encodeNode(Node* node) {
      assert(!((size_t)node & align_mask)); 
      return NodeRef((size_t) node);
    }

    /*! Encodes a node */
    static __forceinline NodeRef encodeNode(NodeMB* node) { 
      assert(!((size_t)node & align_mask)); 
      return NodeRef((size_t) node | tyNodeMB);
    }

    /*! Encodes an unaligned node */
    static __forceinline NodeRef encodeNode(UnalignedNode* node) { 
      return NodeRef((size_t) node | tyUnalignedNode);
    }

    /*! Encodes an unaligned motion blur node */
    static __forceinline NodeRef encodeNode(UnalignedNodeMB* node) { 
      return NodeRef((size_t) node | tyUnalignedNodeMB);
    }
    
    /*! Encodes a leaf */
    static __forceinline NodeRef encodeLeaf(void* tri, size_t num) {
      assert(!((size_t)tri & align_mask)); 
      assert(num <= maxLeafBlocks);
      return NodeRef((size_t)tri | (tyLeaf+min(num,(size_t)maxLeafBlocks)));
    }

    /*! Encodes a leaf */
    static __forceinline NodeRef encodeTypedLeaf(void* ptr, size_t ty) {
      assert(!((size_t)ptr & align_mask)); 
      return NodeRef((size_t)ptr | (tyLeaf+ty));
    }

    /*! bvh type information */
  public:
    const PrimitiveType& primTy;       //!< primitive type stored in the BVH
    bool listMode;                     //!< true if number of leaf items not encoded in NodeRef

    /*! bvh data */
  public:
    NodeRef root;                      //!< Root node
    FastAllocator alloc;               //!< allocator used to allocate nodes
    Device* device;                    //!< device pointer
    Scene* scene;                      //!< scene pointer

    /*! statistics data */
  public:
    size_t numPrimitives;              //!< number of primitives the BVH is build over
    size_t numVertices;                //!< number of vertices the BVH references
    
    /*! data arrays for special builders */
  public:
    std::vector<BVH4*> objects;
    void* data_mem;                   //!< additional memory, currently used for subdivpatch1cached memory
    size_t size_data_mem;
  };
}
