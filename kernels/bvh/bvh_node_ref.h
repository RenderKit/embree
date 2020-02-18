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

#include "../common/default.h"
#include "../common/alloc.h"
#include "../common/accel.h"
#include "../common/device.h"
#include "../common/scene.h"
#include "../geometry/primitive.h"
#include "../common/ray.h"

namespace embree
{
  /* BVH node reference with bounds */
  template<typename NodeRef>
  struct BVHNodeRecord
  {
    __forceinline BVHNodeRecord() {}
    __forceinline BVHNodeRecord(NodeRef ref, const BBox3fa& bounds) : ref(ref), bounds(bounds) {}

    NodeRef ref;
    BBox3fa bounds;
  };

  template<typename NodeRef>
  struct BVHNodeRecordMB
  {
    __forceinline BVHNodeRecordMB() {}
    __forceinline BVHNodeRecordMB(NodeRef ref, const LBBox3fa& lbounds) : ref(ref), lbounds(lbounds) {}

    NodeRef ref;
    LBBox3fa lbounds;
  };

  template<typename NodeRef>
  struct BVHNodeRecordMB4D
  {
    __forceinline BVHNodeRecordMB4D() {}
    __forceinline BVHNodeRecordMB4D(NodeRef ref, const LBBox3fa& lbounds, const BBox1f& dt) : ref(ref), lbounds(lbounds), dt(dt) {}

    NodeRef ref;
    LBBox3fa lbounds;
    BBox1f dt;
  };

  template<int N> struct BaseNode_t;
  template<int N> struct AlignedNode_t;
  template<int N> struct AlignedNodeMB_t;
  template<int N> struct AlignedNodeMB4D_t;
  template<int N> struct UnalignedNode_t;
  template<int N> struct UnalignedNodeMB_t;
  template<int N> struct QuantizedNode_t;
  template<int N> struct QuantizedNodeMB_t;
  
  /*! Pointer that points to a node or a list of primitives */
  template<int N>
    struct NodeRefPtr
  {
    template<int NN> friend class BVHN;

    /*! Number of bytes the nodes and primitives are minimally aligned to.*/
    static const size_t byteAlignment = 16;
    static const size_t byteNodeAlignment = 4*N;

    /*! highest address bit is used as barrier for some algorithms */
    static const size_t barrier_mask = (1LL << (8*sizeof(size_t)-1));

    /*! Masks the bits that store the number of items per leaf. */
    static const size_t align_mask = byteAlignment-1;
    static const size_t items_mask = byteAlignment-1;

    /*! different supported node types */
    static const size_t tyAlignedNode = 0;
    static const size_t tyAlignedNodeMB = 1;
    static const size_t tyAlignedNodeMB4D = 6;
    static const size_t tyUnalignedNode = 2;
    static const size_t tyUnalignedNodeMB = 3;
    static const size_t tyQuantizedNode = 5;
    static const size_t tyLeaf = 8;

    /*! Empty node */
    static const size_t emptyNode = tyLeaf;

    /*! Invalid node, used as marker in traversal */
    static const size_t invalidNode = (((size_t)-1) & (~items_mask)) | (tyLeaf+0);
    static const size_t popRay      = (((size_t)-1) & (~items_mask)) | (tyLeaf+1);

    /*! Maximum number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = items_mask-tyLeaf;
        
    /*! Default constructor */
    __forceinline NodeRefPtr () {}
    
    /*! Construction from integer */
    __forceinline NodeRefPtr (size_t ptr) : ptr(ptr) {}
    
    /*! Cast to size_t */
    __forceinline operator size_t() const { return ptr; }
    
    /*! Sets the barrier bit. */
    __forceinline void setBarrier() {
#if defined(__X86_64__)
      assert(!isBarrier());
      ptr |= barrier_mask;
#else
      assert(false);
#endif
    }
    
    /*! Clears the barrier bit. */
    __forceinline void clearBarrier() {
#if defined(__X86_64__)
      ptr &= ~barrier_mask;
#else
      assert(false);
#endif
    }
    
    /*! Checks if this is an barrier. A barrier tells the top level tree rotations how deep to enter the tree. */
    __forceinline bool isBarrier() const { return (ptr & barrier_mask) != 0; }
    
    /*! checks if this is a leaf */
    __forceinline size_t isLeaf() const { return ptr & tyLeaf; }
    
    /*! returns node type */
    __forceinline int type() const { return ptr & (size_t)align_mask; }
    
    /*! checks if this is a node */
    __forceinline int isAlignedNode() const { return (ptr & (size_t)align_mask) == tyAlignedNode; }
    
    /*! checks if this is a motion blur node */
    __forceinline int isAlignedNodeMB() const { return (ptr & (size_t)align_mask) == tyAlignedNodeMB; }
    
    /*! checks if this is a 4D motion blur node */
    __forceinline int isAlignedNodeMB4D() const { return (ptr & (size_t)align_mask) == tyAlignedNodeMB4D; }
    
    /*! checks if this is a node with unaligned bounding boxes */
    __forceinline int isUnalignedNode() const { return (ptr & (size_t)align_mask) == tyUnalignedNode; }
    
    /*! checks if this is a motion blur node with unaligned bounding boxes */
    __forceinline int isUnalignedNodeMB() const { return (ptr & (size_t)align_mask) == tyUnalignedNodeMB; }
    
    /*! checks if this is a quantized node */
    __forceinline int isQuantizedNode() const { return (ptr & (size_t)align_mask) == tyQuantizedNode; }

    /*! Encodes a node */
    static __forceinline NodeRefPtr encodeNode(AlignedNode_t<N>* node) {
      assert(!((size_t)node & align_mask));
      return NodeRefPtr((size_t) node);
    }

    static __forceinline NodeRefPtr encodeNode(AlignedNodeMB_t<N>* node) {
      assert(!((size_t)node & align_mask));
      return NodeRefPtr((size_t) node | tyAlignedNodeMB);
    }

    static __forceinline NodeRefPtr encodeNode(AlignedNodeMB4D_t<N>* node) {
      assert(!((size_t)node & align_mask));
      return NodeRefPtr((size_t) node | tyAlignedNodeMB4D);
    }

    /*! Encodes an unaligned node */
    static __forceinline NodeRefPtr encodeNode(UnalignedNode_t<N>* node) {
      return NodeRefPtr((size_t) node | tyUnalignedNode);
    }

    /*! Encodes an unaligned motion blur node */
    static __forceinline NodeRefPtr encodeNode(UnalignedNodeMB_t<N>* node) {
      return NodeRefPtr((size_t) node | tyUnalignedNodeMB);
    }

    /*! Encodes a leaf */
    static __forceinline NodeRefPtr encodeLeaf(void* tri, size_t num) {
      assert(!((size_t)tri & align_mask));
      assert(num <= maxLeafBlocks);
      return NodeRefPtr((size_t)tri | (tyLeaf+min(num,(size_t)maxLeafBlocks)));
    }

    /*! Encodes a leaf */
    static __forceinline NodeRefPtr encodeTypedLeaf(void* ptr, size_t ty) {
      assert(!((size_t)ptr & align_mask));
      return NodeRefPtr((size_t)ptr | (tyLeaf+ty));
    }
    
    /*! returns base node pointer */
    __forceinline BaseNode_t<N>* baseNode()
    {
      assert(!isLeaf());
      return (BaseNode_t<N>*)(ptr & ~(size_t)align_mask);
    }
    __forceinline const BaseNode_t<N>* baseNode() const
    {
      assert(!isLeaf());
      return (const BaseNode_t<N>*)(ptr & ~(size_t)align_mask);
    }
    
    /*! returns node pointer */
    __forceinline       AlignedNode_t<N>* alignedNode()       { assert(isAlignedNode()); return (      AlignedNode_t<N>*)ptr; }
    __forceinline const AlignedNode_t<N>* alignedNode() const { assert(isAlignedNode()); return (const AlignedNode_t<N>*)ptr; }
    
    /*! returns motion blur node pointer */
    __forceinline       AlignedNodeMB_t<N>* alignedNodeMB()       { assert(isAlignedNodeMB() || isAlignedNodeMB4D()); return (      AlignedNodeMB_t<N>*)(ptr & ~(size_t)align_mask); }
    __forceinline const AlignedNodeMB_t<N>* alignedNodeMB() const { assert(isAlignedNodeMB() || isAlignedNodeMB4D()); return (const AlignedNodeMB_t<N>*)(ptr & ~(size_t)align_mask); }
    
    /*! returns 4D motion blur node pointer */
    __forceinline       AlignedNodeMB4D_t<N>* alignedNodeMB4D()       { assert(isAlignedNodeMB4D()); return (      AlignedNodeMB4D_t<N>*)(ptr & ~(size_t)align_mask); }
    __forceinline const AlignedNodeMB4D_t<N>* alignedNodeMB4D() const { assert(isAlignedNodeMB4D()); return (const AlignedNodeMB4D_t<N>*)(ptr & ~(size_t)align_mask); }
    
    /*! returns unaligned node pointer */
    __forceinline       UnalignedNode_t<N>* unalignedNode()       { assert(isUnalignedNode()); return (      UnalignedNode_t<N>*)(ptr & ~(size_t)align_mask); }
    __forceinline const UnalignedNode_t<N>* unalignedNode() const { assert(isUnalignedNode()); return (const UnalignedNode_t<N>*)(ptr & ~(size_t)align_mask); }
    
    /*! returns unaligned motion blur node pointer */
    __forceinline       UnalignedNodeMB_t<N>* unalignedNodeMB()       { assert(isUnalignedNodeMB()); return (      UnalignedNodeMB_t<N>*)(ptr & ~(size_t)align_mask); }
    __forceinline const UnalignedNodeMB_t<N>* unalignedNodeMB() const { assert(isUnalignedNodeMB()); return (const UnalignedNodeMB_t<N>*)(ptr & ~(size_t)align_mask); }
    
    /*! returns quantized node pointer */
    __forceinline       QuantizedNode_t<N>* quantizedNode()       { assert(isQuantizedNode()); return (      QuantizedNode_t<N>*)(ptr  & ~(size_t)align_mask ); }
    __forceinline const QuantizedNode_t<N>* quantizedNode() const { assert(isQuantizedNode()); return (const QuantizedNode_t<N>*)(ptr  & ~(size_t)align_mask ); }
    
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
    
     /*! returns the wideness */
    __forceinline size_t getN() const { return N; }
    
  private:
    size_t ptr;
  };
}
