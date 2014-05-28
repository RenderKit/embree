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
#include "geometry/bezier1.h"
#include "geometry/bezier1i.h"

#define BVH4HAIR_COMPRESS_ALIGNED_NODES 0
#define BVH4HAIR_COMPRESS_UNALIGNED_NODES 0

namespace embree
{
  /*! BVH4 with unaligned bounds. */
  class BVH4Hair : public Bounded
  {
    ALIGNED_CLASS;
  public:

    /*! forward declaration of node type */
    struct Node;
    struct CompressedAlignedNode;
    struct UncompressedAlignedNode;
    struct CompressedUnalignedNode;
    struct UncompressedUnalignedNode;
    
#if BVH4HAIR_COMPRESS_ALIGNED_NODES 
    typedef CompressedAlignedNode AlignedNode;
#else
    typedef UncompressedAlignedNode AlignedNode;
#endif

#if BVH4HAIR_COMPRESS_UNALIGNED_NODES
    typedef CompressedUnalignedNode UnalignedNode;
#else
    typedef UncompressedUnalignedNode UnalignedNode;
#endif

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
    static const size_t emptyNode = 2;

    /*! Invalid node, used as marker in traversal */
    static const size_t invalidNode = (((size_t)-1) & (~items_mask)) | 2;
      
    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 32;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+16;
    static const size_t maxDepth = maxBuildDepthLeaf+maxBuildDepthLeaf+maxBuildDepth;   
 
    /*! Maximal number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = items_mask-2;

    /*! Cost of one traversal step. */
    static const int travCostAligned = 1;
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
#if BVH4HAIR_COMPRESS_UNALIGNED_NODES
	prefetchL1(((char*)ptr)+0*64);
	prefetchL1(((char*)ptr)+1*64);
#else
	prefetchL1(((char*)ptr)+0*64);
	prefetchL1(((char*)ptr)+1*64);
	prefetchL1(((char*)ptr)+2*64);
	prefetchL1(((char*)ptr)+3*64);
#endif
      }

      __forceinline void prefetch_L2() const 
      {
#if BVH4HAIR_COMPRESS_UNALIGNED_NODES
	prefetchL2(((char*)ptr)+0*64);
	prefetchL2(((char*)ptr)+1*64);
#else
	prefetchL2(((char*)ptr)+0*64);
	prefetchL2(((char*)ptr)+1*64);
	prefetchL2(((char*)ptr)+2*64);
	prefetchL2(((char*)ptr)+3*64);
#endif
      }

      /*! checks if this is a leaf */
      __forceinline int isLeaf() const { return (ptr & (size_t)align_mask) > 1; }

      /*! checks if this is a node */
      __forceinline int isNode() const { return (ptr & (size_t)align_mask) <= 1; }
      
      /*! checks if this is a node with aligned bounding boxes */
      __forceinline int isAlignedNode() const { return (ptr & (size_t)align_mask) == 0; }

      /*! checks if this is a node with aligned bounding boxes */
      __forceinline int isUnalignedNode() const { return (ptr & (size_t)align_mask) == 1; }
      
      /*! returns node pointer */
      __forceinline       Node* node()       { assert(isNode()); return (      Node*)((size_t)ptr & ~(size_t)align_mask); }
      __forceinline const Node* node() const { assert(isNode()); return (const Node*)((size_t)ptr & ~(size_t)align_mask); }

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

    /*! Compressed node with aligned bounds */
#if BVH4HAIR_COMPRESS_ALIGNED_NODES
    struct CompressedAlignedNode : public Node
    {
      enum { stride = 4 };

      /*! Clears the node. */
      __forceinline void clear() 
      {
        offset = 0.0f; scale = 0.0f;
        lower_x[0] = lower_x[1] = lower_x[2] = lower_x[3] = 0;
        lower_y[0] = lower_y[1] = lower_y[2] = lower_y[3] = 0;
        lower_z[0] = lower_z[1] = lower_z[2] = lower_z[3] = 0;
        upper_x[0] = upper_x[1] = upper_x[2] = upper_x[3] = 0;
        upper_y[0] = upper_y[1] = upper_y[2] = upper_y[3] = 0;
        upper_z[0] = upper_z[1] = upper_z[2] = upper_z[3] = 0;
        Node::clear();
      }

      /*! Sets non-axis aligned space of node and parent bounding box. */
      __forceinline void set(const NAABBox3fa& naabb) {
        offset = naabb.bounds.lower;
        scale  = naabb.bounds.size()/255.0f;
      }

      /*! Sets bounding box. */
      __forceinline void set(size_t i, const BBox3fa& bounds) 
      {
        assert(i < N);
        const Vec3fa lower = select(eq_mask(scale,Vec3fa(0.0f)),0.0f,(bounds.lower-Vec3fa(offset))/Vec3fa(scale));
        assert(lower.x >= 0.0f && lower.x <= 255.01f);
        assert(lower.y >= 0.0f && lower.y <= 255.01f);
        assert(lower.z >= 0.0f && lower.z <= 255.01f);
        lower_x[i] = (unsigned char) clamp(floorf(lower.x),0.0f,255.0f);
        lower_y[i] = (unsigned char) clamp(floorf(lower.y),0.0f,255.0f);
        lower_z[i] = (unsigned char) clamp(floorf(lower.z),0.0f,255.0f);
        const Vec3fa upper = select(eq_mask(scale,Vec3fa(0.0f)),0.0f,(bounds.upper-Vec3fa(offset))/Vec3fa(scale));
        assert(upper.x >= 0.0f && upper.x <= 255.01f);
        assert(upper.y >= 0.0f && upper.y <= 255.01f);
        assert(upper.z >= 0.0f && upper.z <= 255.01f);
        upper_x[i] = (unsigned char) clamp(ceilf(upper.x),0.0f,255.0f);
        upper_y[i] = (unsigned char) clamp(ceilf(upper.y),0.0f,255.0f);
        upper_z[i] = (unsigned char) clamp(ceilf(upper.z),0.0f,255.0f);
      }

      /*! Sets ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
        Node::set(i,childID);
      }

      /*! returns ith bounding boxes */
      __forceinline BBox3fa getBounds(size_t i) const 
      {
        assert(i < N);
        const Vec3f lower((float)lower_x[i],lower_y[i],lower_z[i]);
        const Vec3f upper((float)upper_x[i],upper_y[i],upper_z[i]);
        return BBox3fa(Vec3fa(offset+scale*lower),Vec3fa(offset+scale*upper));
      }

      /*! returns 4 bounding boxes */
      __forceinline const BBoxSSE3f getBounds(const size_t nearX, const size_t nearY, const size_t nearZ) const 
      {
        const size_t farX  = nearX ^ 4, farY  = nearY ^ 4, farZ  = nearZ ^ 4;
        const ssef near_x = ssef(_mm_cvtepu8_epi32(*(ssei*)((char*)&this->lower_x+nearX)));
        const ssef near_y = ssef(_mm_cvtepu8_epi32(*(ssei*)((char*)&this->lower_y+nearY)));
        const ssef near_z = ssef(_mm_cvtepu8_epi32(*(ssei*)((char*)&this->lower_z+nearZ)));
        const ssef far_x  = ssef(_mm_cvtepu8_epi32(*(ssei*)((char*)&this->lower_x+farX )));
        const ssef far_y  = ssef(_mm_cvtepu8_epi32(*(ssei*)((char*)&this->lower_y+farY )));
        const ssef far_z  = ssef(_mm_cvtepu8_epi32(*(ssei*)((char*)&this->lower_z+farZ )));
        const sse3f offset = *(Vec3fa*)&this->offset;
        const sse3f scale  = *(Vec3fa*)&this->scale;
        return BBoxSSE3f(scale*sse3f(near_x,near_y,near_z)+offset,
                          scale*sse3f(far_x, far_y, far_z)+offset);
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend(size_t i) const {
        assert(i < N);
        return getBounds(i).size();
      }

    public:
      Vec3f offset;               //!< offset to decompress bounds
      Vec3f scale;                //!< scale  to decompress bounds
      unsigned char lower_x[N];   //!< X dimension of lower bounds of all 4 children.
      unsigned char upper_x[N];   //!< X dimension of upper bounds of all 4 children.
      unsigned char lower_y[N];   //!< Y dimension of lower bounds of all 4 children.
      unsigned char upper_y[N];   //!< Y dimension of upper bounds of all 4 children.
      unsigned char lower_z[N];   //!< Z dimension of lower bounds of all 4 children.
      unsigned char upper_z[N];   //!< Z dimension of upper bounds of all 4 children.
    };
#endif

    /*! Node with aligned bounds */
    struct UncompressedAlignedNode : public Node
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

    /*! Compressed node with unaligned bounds */
#if BVH4HAIR_COMPRESS_UNALIGNED_NODES
    struct CompressedUnalignedNode : public Node
    {
      /*! Clears the node. */
      __forceinline void clear() 
      {
        xfm_vx[0] = 1; xfm_vx[1] = 0; xfm_vx[2] = 0;
        xfm_vy[0] = 0; xfm_vy[1] = 1; xfm_vy[2] = 0;
        xfm_vz[0] = 0; xfm_vz[1] = 0; xfm_vz[2] = 1;
        offset = 0.0f; scale = 0.0f;
        lower_x[0] = lower_x[1] = lower_x[2] = lower_x[3] = 1;
        lower_y[0] = lower_y[1] = lower_y[2] = lower_y[3] = 1;
        lower_z[0] = lower_z[1] = lower_z[2] = lower_z[3] = 1;
        upper_x[0] = upper_x[1] = upper_x[2] = upper_x[3] = 1;
        upper_y[0] = upper_y[1] = upper_y[2] = upper_y[3] = 1;
        upper_z[0] = upper_z[1] = upper_z[2] = upper_z[3] = 1;
        align[0] = align[1] = align[2] = align[3] = 0;
        Node::clear();
      }

      /*! Sets non-axis aligned space of node and parent bounding box. */
      __forceinline void set(const NAABBox3fa& naabb) 
      {
        const LinearSpace3fa& space = naabb.space;

#if !defined(__WIN32__)
 		assert(127.0f*space.vx.x >= -127.0f && 127.0f*space.vx.x <= 127.0f && truncf(127.0f*space.vx.x) == 127.0f*space.vx.x);
		assert(127.0f*space.vx.y >= -127.0f && 127.0f*space.vx.y <= 127.0f && truncf(127.0f*space.vx.y) == 127.0f*space.vx.y);
		assert(127.0f*space.vx.z >= -127.0f && 127.0f*space.vx.z <= 127.0f && truncf(127.0f*space.vx.z) == 127.0f*space.vx.z);
		assert(127.0f*space.vy.x >= -127.0f && 127.0f*space.vy.x <= 127.0f && truncf(127.0f*space.vy.x) == 127.0f*space.vy.x);
		assert(127.0f*space.vy.y >= -127.0f && 127.0f*space.vy.y <= 127.0f && truncf(127.0f*space.vy.y) == 127.0f*space.vy.y);
		assert(127.0f*space.vy.z >= -127.0f && 127.0f*space.vy.z <= 127.0f && truncf(127.0f*space.vy.z) == 127.0f*space.vy.z);
		assert(127.0f*space.vz.x >= -127.0f && 127.0f*space.vz.x <= 127.0f && truncf(127.0f*space.vz.x) == 127.0f*space.vz.x);
		assert(127.0f*space.vz.y >= -127.0f && 127.0f*space.vz.y <= 127.0f && truncf(127.0f*space.vz.y) == 127.0f*space.vz.y);
		assert(127.0f*space.vz.z >= -127.0f && 127.0f*space.vz.z <= 127.0f && truncf(127.0f*space.vz.z) == 127.0f*space.vz.z);
#endif

        const BBox3fa& bounds = naabb.bounds;
        xfm_vx[0] = (char) (127.0f*space.vx.x); 
        xfm_vx[1] = (char) (127.0f*space.vx.y); 
        xfm_vx[2] = (char) (127.0f*space.vx.z); 
        xfm_vx[3] = 0;
        xfm_vy[0] = (char) (127.0f*space.vy.x); 
        xfm_vy[1] = (char) (127.0f*space.vy.y); 
        xfm_vy[2] = (char) (127.0f*space.vy.z); 
        xfm_vy[3] = 0;
        xfm_vz[0] = (char) (127.0f*space.vz.x); 
        xfm_vz[1] = (char) (127.0f*space.vz.y); 
        xfm_vz[2] = (char) (127.0f*space.vz.z); 
        xfm_vz[3] = 0;
        offset = 127.0f*bounds.lower;
        scale  = (127.0f*bounds.upper-127.0f*bounds.lower)/255.0f;
      }

      /*! Sets bounding box. */
      __forceinline void set(size_t i, const BBox3fa& bounds) 
      {
        assert(i < N);
        const Vec3fa lower = select(eq_mask(scale,Vec3fa(0.0f)),0.0f,(127.0f*bounds.lower-Vec3fa(offset))/Vec3fa(scale));
        assert(lower.x >= 0.0f && lower.x <= 255.01f); // FIXME: should be smaller than 255.0f
        assert(lower.y >= 0.0f && lower.y <= 255.01f);
        assert(lower.z >= 0.0f && lower.z <= 255.01f);
        lower_x[i] = (unsigned char) clamp(floorf(lower.x),0.0f,255.0f);
        lower_y[i] = (unsigned char) clamp(floorf(lower.y),0.0f,255.0f);
        lower_z[i] = (unsigned char) clamp(floorf(lower.z),0.0f,255.0f);
        const Vec3fa upper = select(eq_mask(scale,Vec3fa(0.0f)),0.0f,(127.0f*bounds.upper-Vec3fa(offset))/Vec3fa(scale));
        assert(upper.x >= 0.0f && upper.x <= 255.01f); // FIXME: should be smaller than 255.0f
        assert(upper.y >= 0.0f && upper.y <= 255.01f);
        assert(upper.z >= 0.0f && upper.z <= 255.01f);
        upper_x[i] = (unsigned char) clamp(ceilf(upper.x),0.0f,255.0f);
        upper_y[i] = (unsigned char) clamp(ceilf(upper.y),0.0f,255.0f);
        upper_z[i] = (unsigned char) clamp(ceilf(upper.z),0.0f,255.0f);
      }

      /*! Sets ID of child. */
      __forceinline void set(size_t i, const NodeRef& childID) {
        Node::set(i,childID);
      }

      /*! returns transformation */
      __forceinline const LinearSpace3fa getXfm() const 
      {
        const ssef vx = ssef(_mm_cvtepi8_epi32(*(ssei*)&xfm_vx));
        const ssef vy = ssef(_mm_cvtepi8_epi32(*(ssei*)&xfm_vy));
        const ssef vz = ssef(_mm_cvtepi8_epi32(*(ssei*)&xfm_vz));
        return LinearSpace3fa((Vec3fa)vx,(Vec3fa)vy,(Vec3fa)vz);
      }

      /*! returns ith bounding boxes */
      __forceinline NAABBox3fa getBounds(size_t i) const 
      {
        assert(i < N);
        const Vec3f lower((float)lower_x[i],lower_y[i],lower_z[i]);
        const Vec3f upper((float)upper_x[i],upper_y[i],upper_z[i]);
        return NAABBox3fa(getXfm(),BBox3fa(Vec3fa(offset+scale*lower),Vec3fa(offset+scale*upper)));
      }

      /*! returns 4 bounding boxes */
      __forceinline BBoxSSE3f getBounds() const 
      {
        const ssef lower_x = ssef(_mm_cvtepu8_epi32(*(ssei*)&this->lower_x));
        const ssef lower_y = ssef(_mm_cvtepu8_epi32(*(ssei*)&this->lower_y));
        const ssef lower_z = ssef(_mm_cvtepu8_epi32(*(ssei*)&this->lower_z));
        const ssef upper_x = ssef(_mm_cvtepu8_epi32(*(ssei*)&this->upper_x));
        const ssef upper_y = ssef(_mm_cvtepu8_epi32(*(ssei*)&this->upper_y));
        const ssef upper_z = ssef(_mm_cvtepu8_epi32(*(ssei*)&this->upper_z));
        const sse3f offset = *(Vec3fa*)&this->offset;
        const sse3f scale  = *(Vec3fa*)&this->scale;
        return BBoxSSE3f(scale*sse3f(lower_x,lower_y,lower_z)+offset,
                          scale*sse3f(upper_x,upper_y,upper_z)+offset);
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend(size_t i) const {
        assert(i < N);
        return getBounds(i).bounds.size()/127.0f;
      }

    public:
      char xfm_vx[4];             //!< 1st column of transformation
      char xfm_vy[4];             //!< 2nd column of transformation
      char xfm_vz[4];             //!< 3rd column of transformation
      char align[4];              
      Vec3f offset;               //!< offset to decompress bounds
      Vec3f scale;                //!< scale  to decompress bounds
      unsigned char lower_x[N];   //!< X dimension of lower bounds of all 4 children.
      unsigned char lower_y[N];   //!< Y dimension of lower bounds of all 4 children.
      unsigned char lower_z[N];   //!< Z dimension of lower bounds of all 4 children.
      unsigned char upper_x[N];   //!< X dimension of upper bounds of all 4 children.
      unsigned char upper_y[N];   //!< Y dimension of upper bounds of all 4 children.
      unsigned char upper_z[N];   //!< Z dimension of upper bounds of all 4 children.
    };
#endif

    /*! Node with unaligned bounds */
    struct UncompressedUnalignedNode : public Node
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

  public:

    /*! BVH4Hair default constructor. */
    BVH4Hair (const PrimitiveType& primTy, Scene* scene);

    /*! BVH4Hair destruction */
    ~BVH4Hair ();

    /*! BVH4Hair instantiations */
    static Accel* BVH4HairBezier1(Scene* scene, bool highQuality);
    static Accel* BVH4HairBezier1i(Scene* scene, bool highQuality);

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

    /*! allocates a block of primitives */
    __forceinline char* allocPrimitiveBlocks(size_t thread, size_t num) {
      return (char*) alloc.malloc(thread,num*primTy.bytes,1 << 4);
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
      return NodeRef((size_t)data | (2+min(num,(size_t)maxLeafBlocks)));
    }

  public:
    const PrimitiveType& primTy;       //!< primitive type stored in the BVH
    Scene* scene;
    NodeRef root;  //!< Root node
    size_t numPrimitives;
    size_t numVertices;
  };
}
