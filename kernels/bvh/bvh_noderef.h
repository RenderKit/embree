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
  /*! flags used to enable specific node types in intersectors */
  enum BVHNodeFlags
  {
    BVH_FLAG_ALIGNED_NODE = 0x00001,
    BVH_FLAG_ALIGNED_NODE_MB = 0x00010,
    BVH_FLAG_UNALIGNED_NODE = 0x00100,
    BVH_FLAG_UNALIGNED_NODE_MB = 0x01000,
    BVH_FLAG_QUANTIZED_NODE = 0x100000,
    BVH_FLAG_ALIGNED_NODE_MB4D = 0x1000000,

    /* short versions */
    BVH_AN1 = BVH_FLAG_ALIGNED_NODE,
    BVH_AN2 = BVH_FLAG_ALIGNED_NODE_MB,
    BVH_AN2_AN4D = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_ALIGNED_NODE_MB4D,
    BVH_UN1 = BVH_FLAG_UNALIGNED_NODE,
    BVH_UN2 = BVH_FLAG_UNALIGNED_NODE_MB,
    BVH_MB = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_UNALIGNED_NODE_MB | BVH_FLAG_ALIGNED_NODE_MB4D,
    BVH_AN1_UN1 = BVH_FLAG_ALIGNED_NODE | BVH_FLAG_UNALIGNED_NODE,
    BVH_AN2_UN2 = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_UNALIGNED_NODE_MB,
    BVH_AN2_AN4D_UN2 = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_ALIGNED_NODE_MB4D | BVH_FLAG_UNALIGNED_NODE_MB,
    BVH_QN1 = BVH_FLAG_QUANTIZED_NODE
  };

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

    static __forceinline unsigned int encodeQuantizedNode(size_t base, size_t node) {
      assert(!((size_t)node & align_mask));
      ssize_t node_offset = (ssize_t)node-(ssize_t)base;
      assert(node_offset != 0);
      assert((int64_t)node_offset >= -int64_t(0x80000000) && (int64_t)node_offset <= (int64_t)0x7fffffff);
      return (unsigned int)node_offset | tyQuantizedNode;
    }

    static __forceinline int encodeQuantizedLeaf(size_t base, size_t node) {
      ssize_t leaf_offset = (ssize_t)node-(ssize_t)base;
      assert((int64_t)leaf_offset >= -int64_t(0x80000000) && (int64_t)leaf_offset <= (int64_t)0x7fffffff);
      return (int)leaf_offset;
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
  
  /*! BVHN Base Node */
  template<int N>
    struct BaseNode_t
  {
    /*! Clears the node. */
    __forceinline void clear()
    {
      for (size_t i=0; i<N; i++)
        children[i] = NodeRefPtr<N>::emptyNode;
    }
    
    /*! Returns reference to specified child */
    __forceinline       NodeRefPtr<N>& child(size_t i)       { assert(i<N); return children[i]; }
    __forceinline const NodeRefPtr<N>& child(size_t i) const { assert(i<N); return children[i]; }
    
    /*! verifies the node */
    __forceinline bool verify() const
    {
      for (size_t i=0; i<N; i++) {
        if (child(i) == NodeRefPtr<N>::emptyNode) {
          for (; i<N; i++) {
            if (child(i) != NodeRefPtr<N>::emptyNode)
              return false;
          }
          break;
        }
      }
      return true;
    }
    
    NodeRefPtr<N> children[N];    //!< Pointer to the N children (can be a node or leaf)
  };
  
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
  
  /*! Motion Blur AlignedNode */
  template<int N>
    struct AlignedNodeMB_t : public BaseNode_t<N>
  {
    using BaseNode_t<N>::children;
    typedef BVHNodeRecord<NodeRefPtr<N>>     NodeRecord;
    typedef BVHNodeRecordMB<NodeRefPtr<N>>   NodeRecordMB;
    typedef BVHNodeRecordMB4D<NodeRefPtr<N>> NodeRecordMB4D;
        
    struct Create
    {
      __forceinline NodeRefPtr<N> operator() (const FastAllocator::CachedAllocator& alloc) const
      {
        AlignedNodeMB_t<N>* node = (AlignedNodeMB_t<N>*) alloc.malloc0(sizeof(AlignedNodeMB_t<N>),NodeRefPtr<N>::byteNodeAlignment); node->clear();
        return NodeRefPtr<N>::encodeNode(node);
      }
    };
    
    struct Set
    {
      __forceinline void operator() (NodeRefPtr<N> node, size_t i, const NodeRecordMB4D& child) const {
        node.alignedNodeMB()->set(i,child);
      }
    };
    
    struct Create2
    {
      template<typename BuildRecord>
      __forceinline NodeRefPtr<N> operator() (BuildRecord* children, const size_t num, const FastAllocator::CachedAllocator& alloc) const
      {
        AlignedNodeMB_t<N>* node = (AlignedNodeMB_t<N>*) alloc.malloc0(sizeof(AlignedNodeMB_t<N>),NodeRefPtr<N>::byteNodeAlignment); node->clear();
        return NodeRefPtr<N>::encodeNode(node);
      }
    };
    
    struct Set2
    { 
      template<typename BuildRecord>
      __forceinline NodeRecordMB operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRefPtr<N> ref, NodeRecordMB* children, const size_t num) const
      {
        AlignedNodeMB_t<N>* node = ref.alignedNodeMB();
        
        LBBox3fa bounds = empty;
        for (size_t i=0; i<num; i++) {
          node->setRef(i,children[i].ref);
          node->setBounds(i,children[i].lbounds);
          bounds.extend(children[i].lbounds);
        }
        return NodeRecordMB(ref,bounds);
      }
    };
    
    struct Set2TimeRange
    {
      __forceinline Set2TimeRange(BBox1f tbounds) : tbounds(tbounds) {}
      
      template<typename BuildRecord>
      __forceinline NodeRecordMB operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRefPtr<N> ref, NodeRecordMB* children, const size_t num) const
      {
        AlignedNodeMB_t<N>* node = ref.alignedNodeMB();
        
        LBBox3fa bounds = empty;
        for (size_t i=0; i<num; i++) {
          node->setRef(i, children[i].ref);
          node->setBounds(i, children[i].lbounds, tbounds);
          bounds.extend(children[i].lbounds);
        }
        return NodeRecordMB(ref,bounds);
      }
      
      BBox1f tbounds;
    };
    
    /*! Clears the node. */
    __forceinline void clear()  {
      lower_x = lower_y = lower_z = vfloat<N>(nan);
      upper_x = upper_y = upper_z = vfloat<N>(nan);
      lower_dx = lower_dy = lower_dz = vfloat<N>(nan); // initialize with NAN and update during refit
      upper_dx = upper_dy = upper_dz = vfloat<N>(nan);
      BaseNode_t<N>::clear();
    }
    
    /*! Sets ID of child. */
    __forceinline void setRef(size_t i, NodeRefPtr<N> ref) {
      children[i] = ref;
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
    __forceinline void setBounds(size_t i, const LBBox3fa& bounds, const BBox1f& tbounds) {
      setBounds(i, bounds.global(tbounds));
    }
    
    /*! Sets bounding box and ID of child. */
    __forceinline void set(size_t i, NodeRefPtr<N> ref, const BBox3fa& bounds) {
      lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
      upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
      children[i] = ref;
    }
    
    /*! Sets bounding box and ID of child. */
    __forceinline void set(size_t i, const NodeRecordMB4D& child)
    {
      setRef(i, child.ref);
      setBounds(i, child.lbounds, child.dt);
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
    
    /*! Returns bounds of node. */
    __forceinline BBox3fa bounds() const {
      return BBox3fa(Vec3fa(reduce_min(min(lower_x,lower_x+lower_dx)),
                            reduce_min(min(lower_y,lower_y+lower_dy)),
                            reduce_min(min(lower_z,lower_z+lower_dz))),
                     Vec3fa(reduce_max(max(upper_x,upper_x+upper_dx)),
                            reduce_max(max(upper_y,upper_y+upper_dy)),
                            reduce_max(max(upper_z,upper_z+upper_dz))));
    }
    
    /*! Return bounding box of child i */
    __forceinline BBox3fa bounds(size_t i) const {
      return merge(bounds0(i),bounds1(i));
    }
    
    /*! Return linear bounding box of child i */
    __forceinline LBBox3fa lbounds(size_t i) const {
      return LBBox3fa(bounds0(i),bounds1(i));
    }
    
    /*! Return bounding box of child i at specified time */
    __forceinline BBox3fa bounds(size_t i, float time) const {
      return lerp(bounds0(i),bounds1(i),time);
    }
    
    /*! Returns the expected surface area when randomly sampling the time. */
    __forceinline float expectedHalfArea(size_t i) const {
      return lbounds(i).expectedHalfArea();
    }
    
    /*! Returns the expected surface area when randomly sampling the time. */
    __forceinline float expectedHalfArea(size_t i, const BBox1f& t0t1) const {
      return lbounds(i).expectedHalfArea(t0t1); 
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
    __forceinline       NodeRefPtr<N>& child(size_t i)       { assert(i<N); return children[i]; }
    __forceinline const NodeRefPtr<N>& child(size_t i) const { assert(i<N); return children[i]; }
    
    /*! stream output operator */
    friend std::ostream& operator<<(std::ostream& cout, const AlignedNodeMB_t<N>& n) 
    {
      cout << "AlignedNodeMB {" << std::endl;
      for (size_t i=0; i<N; i++) 
      {
        const BBox3fa b0 = n.bounds0(i);
        const BBox3fa b1 = n.bounds1(i);
        cout << "  child" << i << " { " << std::endl;
        cout << "    bounds0 = " << b0 << ", " << std::endl;
        cout << "    bounds1 = " << b1 << ", " << std::endl;
        cout << "  }";
      }
      cout << "}";
      return cout;
    }
    
  public:
    vfloat<N> lower_x;        //!< X dimension of lower bounds of all N children.
    vfloat<N> upper_x;        //!< X dimension of upper bounds of all N children.
    vfloat<N> lower_y;        //!< Y dimension of lower bounds of all N children.
    vfloat<N> upper_y;        //!< Y dimension of upper bounds of all N children.
    vfloat<N> lower_z;        //!< Z dimension of lower bounds of all N children.
    vfloat<N> upper_z;        //!< Z dimension of upper bounds of all N children.
    
    vfloat<N> lower_dx;        //!< X dimension of lower bounds of all N children.
    vfloat<N> upper_dx;        //!< X dimension of upper bounds of all N children.
    vfloat<N> lower_dy;        //!< Y dimension of lower bounds of all N children.
    vfloat<N> upper_dy;        //!< Y dimension of upper bounds of all N children.
    vfloat<N> lower_dz;        //!< Z dimension of lower bounds of all N children.
    vfloat<N> upper_dz;        //!< Z dimension of upper bounds of all N children.
  };
  
  /*! Aligned 4D Motion Blur Node */
  template<int N>
    struct AlignedNodeMB4D_t : public AlignedNodeMB_t<N>
  {
    using BaseNode_t<N>::children;
    using AlignedNodeMB_t<N>::set;
    using AlignedNodeMB_t<N>::bounds;
    using AlignedNodeMB_t<N>::lower_x;
    using AlignedNodeMB_t<N>::lower_y;
    using AlignedNodeMB_t<N>::lower_z;
    using AlignedNodeMB_t<N>::upper_x;
    using AlignedNodeMB_t<N>::upper_y;
    using AlignedNodeMB_t<N>::upper_z;
    using AlignedNodeMB_t<N>::lower_dx;
    using AlignedNodeMB_t<N>::lower_dy;
    using AlignedNodeMB_t<N>::lower_dz;
    using AlignedNodeMB_t<N>::upper_dx;
    using AlignedNodeMB_t<N>::upper_dy;
    using AlignedNodeMB_t<N>::upper_dz;

    typedef BVHNodeRecord<NodeRefPtr<N>>     NodeRecord;
    typedef BVHNodeRecordMB<NodeRefPtr<N>>   NodeRecordMB;
    typedef BVHNodeRecordMB4D<NodeRefPtr<N>> NodeRecordMB4D;
    
    struct Create
    {
      __forceinline NodeRefPtr<N> operator() (const FastAllocator::CachedAllocator& alloc, bool hasTimeSplits = true) const
      {
        if (hasTimeSplits)
        {
          AlignedNodeMB4D_t<N>* node = (AlignedNodeMB4D_t<N>*) alloc.malloc0(sizeof(AlignedNodeMB4D_t<N>),NodeRefPtr<N>::byteNodeAlignment); node->clear();
          return NodeRefPtr<N>::encodeNode(node);
        }
        else
        {
          AlignedNodeMB_t<N>* node = (AlignedNodeMB_t<N>*) alloc.malloc0(sizeof(AlignedNodeMB_t<N>),NodeRefPtr<N>::byteNodeAlignment); node->clear();
          return NodeRefPtr<N>::encodeNode(node);
        }
      }
    };
    
    struct Set
    {
      __forceinline void operator() (NodeRefPtr<N> ref, size_t i, const NodeRecordMB4D& child) const
      {
        if (likely(ref.isAlignedNodeMB())) {
          ref.alignedNodeMB()->set(i, child);
        } else {
          ref.alignedNodeMB4D()->set(i, child);
        }
      }
    };
    
    /*! Clears the node. */
    __forceinline void clear()  {
      lower_t = vfloat<N>(pos_inf);
      upper_t = vfloat<N>(neg_inf);
      AlignedNodeMB_t<N>::clear();
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
    __forceinline void set(size_t i, NodeRefPtr<N> childID, const LBBox3fa& bounds, const BBox1f& tbounds) 
    {
      AlignedNodeMB_t<N>::setRef(i,childID);
      setBounds(i, bounds, tbounds);
    }
    
    /*! Sets bounding box and ID of child. */
    __forceinline void set(size_t i, const NodeRecordMB4D& child) {
      set(i, child.ref, child.lbounds, child.dt);
    }
    
    /*! Returns reference to specified child */
    __forceinline       NodeRefPtr<N>& child(size_t i)       { assert(i<N); return children[i]; }
    __forceinline const NodeRefPtr<N>& child(size_t i) const { assert(i<N); return children[i]; }
    
    /*! Returns the expected surface area when randomly sampling the time. */
    __forceinline float expectedHalfArea(size_t i) const {
      return AlignedNodeMB_t<N>::lbounds(i).expectedHalfArea(timeRange(i));
    }
    
    /*! returns time range for specified child */
    __forceinline BBox1f timeRange(size_t i) const {
      return BBox1f(lower_t[i],upper_t[i]);
    }
    
    /*! stream output operator */
    friend std::ostream& operator<<(std::ostream& cout, const AlignedNodeMB4D_t<N>& n) 
    {
      cout << "AlignedNodeMB4D {" << std::endl;
      for (size_t i=0; i<N; i++) 
      {
        const BBox3fa b0 = n.bounds0(i);
        const BBox3fa b1 = n.bounds1(i);
        cout << "  child" << i << " { " << std::endl;
        cout << "    bounds0 = " << lerp(b0,b1,n.lower_t[i]) << ", " << std::endl;
        cout << "    bounds1 = " << lerp(b0,b1,n.upper_t[i]) << ", " << std::endl;
        cout << "    time_bounds = " << n.lower_t[i] << ", " << n.upper_t[i] << std::endl;
        cout << "  }";
      }
      cout << "}";
      return cout;
    }
    
  public:
    vfloat<N> lower_t;        //!< time dimension of lower bounds of all N children
    vfloat<N> upper_t;        //!< time dimension of upper bounds of all N children
  };
  
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

  template<int N>
    struct UnalignedNodeMB_t : public BaseNode_t<N>
  {
    using BaseNode_t<N>::children;
    
    struct Create
    {
      __forceinline NodeRefPtr<N> operator() (const FastAllocator::CachedAllocator& alloc) const
      {
        UnalignedNodeMB_t<N>* node = (UnalignedNodeMB_t<N>*) alloc.malloc0(sizeof(UnalignedNodeMB_t<N>),NodeRefPtr<N>::byteNodeAlignment); node->clear();
        return NodeRefPtr<N>::encodeNode(node);
      }
    };
    
    struct Set
    {
      __forceinline void operator() (NodeRefPtr<N> node, size_t i, NodeRefPtr<N> child, const LinearSpace3fa& space, const LBBox3fa& lbounds, const BBox1f dt) const {
        node.unalignedNodeMB()->setRef(i,child);
        node.unalignedNodeMB()->setBounds(i,space,lbounds.global(dt));
      }
    };
    
    /*! Clears the node. */
    __forceinline void clear()
    {
      space0 = one;
      //b0.lower = b0.upper = Vec3fa(nan);
      b1.lower = b1.upper = Vec3fa(nan);
      BaseNode_t<N>::clear();
    }
    
    /*! Sets space and bounding boxes. */
    __forceinline void setBounds(size_t i, const AffineSpace3fa& space, const LBBox3fa& lbounds) {
      setBounds(i,space,lbounds.bounds0,lbounds.bounds1);
    }
    
    /*! Sets space and bounding boxes. */
    __forceinline void setBounds(size_t i, const AffineSpace3fa& s0, const BBox3fa& a, const BBox3fa& c)
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
    __forceinline void setRef(size_t i, const NodeRefPtr<N>& ref) {
      assert(i < N);
      children[i] = ref;
    }
    
    /*! Returns the extent of the bounds of the ith child */
    __forceinline Vec3fa extent0(size_t i) const {
      assert(i < N);
      const Vec3fa vx(space0.l.vx.x[i],space0.l.vx.y[i],space0.l.vx.z[i]);
      const Vec3fa vy(space0.l.vy.x[i],space0.l.vy.y[i],space0.l.vy.z[i]);
      const Vec3fa vz(space0.l.vz.x[i],space0.l.vz.y[i],space0.l.vz.z[i]);
      return rsqrt(vx*vx + vy*vy + vz*vz);
    }
    
  public:
    AffineSpace3vf<N> space0;
    //BBox3vf<N> b0; // these are the unit bounds
    BBox3vf<N> b1;
  };
  
  /*! BVHN Quantized Node */
  template<int N>
    struct __aligned(8) QuantizedBaseNode_t
  {
    typedef unsigned char T;
    static const T MIN_QUAN = 0;
    static const T MAX_QUAN = 255;
    
    /*! Clears the node. */
    __forceinline void clear() {
      for (size_t i=0; i<N; i++) lower_x[i] = lower_y[i] = lower_z[i] = MAX_QUAN;
      for (size_t i=0; i<N; i++) upper_x[i] = upper_y[i] = upper_z[i] = MIN_QUAN;
    }
    
    /*! Returns bounds of specified child. */
    __forceinline BBox3fa bounds(size_t i) const
    {
      assert(i < N);
      const Vec3fa lower(madd(scale.x,(float)lower_x[i],start.x),
                         madd(scale.y,(float)lower_y[i],start.y),
                         madd(scale.z,(float)lower_z[i],start.z));
      const Vec3fa upper(madd(scale.x,(float)upper_x[i],start.x),
                         madd(scale.y,(float)upper_y[i],start.y),
                         madd(scale.z,(float)upper_z[i],start.z));
      return BBox3fa(lower,upper);
    }
    
    /*! Returns extent of bounds of specified child. */
    __forceinline Vec3fa extent(size_t i) const {
      return bounds(i).size();
    }
    
    static __forceinline void init_dim(const vfloat<N> &lower,
                                       const vfloat<N> &upper,
                                       T lower_quant[N],
                                       T upper_quant[N],
                                       float &start,
                                       float &scale)
    {
      /* quantize bounds */
      const vbool<N> m_valid = lower != vfloat<N>(pos_inf);
      const float minF = reduce_min(lower);
      const float maxF = reduce_max(upper);
      float diff = (1.0f+2.0f*float(ulp))*(maxF - minF);
      float decode_scale = diff / float(MAX_QUAN);
      if (decode_scale == 0.0f) decode_scale = 2.0f*FLT_MIN; // result may have been flushed to zero
      assert(madd(decode_scale,float(MAX_QUAN),minF) >= maxF);
      const float encode_scale = diff > 0 ? (float(MAX_QUAN) / diff) : 0.0f;
      vint<N> ilower = max(vint<N>(floor((lower - vfloat<N>(minF))*vfloat<N>(encode_scale))),MIN_QUAN);
      vint<N> iupper = min(vint<N>(ceil ((upper - vfloat<N>(minF))*vfloat<N>(encode_scale))),MAX_QUAN);
      
      /* lower/upper correction */
      vbool<N> m_lower_correction = (madd(vfloat<N>(ilower),decode_scale,minF)) > lower;
      vbool<N> m_upper_correction = (madd(vfloat<N>(iupper),decode_scale,minF)) < upper;
      ilower = max(select(m_lower_correction,ilower-1,ilower),MIN_QUAN);
      iupper = min(select(m_upper_correction,iupper+1,iupper),MAX_QUAN);
      
      /* disable invalid lanes */
      ilower = select(m_valid,ilower,MAX_QUAN);
      iupper = select(m_valid,iupper,MIN_QUAN);
      
      /* store as uchar to memory */
      vint<N>::store(lower_quant,ilower);
      vint<N>::store(upper_quant,iupper);
      start = minF;
      scale = decode_scale;
      
#if defined(DEBUG)
      vfloat<N> extract_lower( vint<N>::loadu(lower_quant) );
      vfloat<N> extract_upper( vint<N>::loadu(upper_quant) );
      vfloat<N> final_extract_lower = madd(extract_lower,decode_scale,minF);
      vfloat<N> final_extract_upper = madd(extract_upper,decode_scale,minF);
      assert( (movemask(final_extract_lower <= lower ) & movemask(m_valid)) == movemask(m_valid));
      assert( (movemask(final_extract_upper >= upper ) & movemask(m_valid)) == movemask(m_valid));
#endif
    }
    
    __forceinline void init_dim(AlignedNode_t<N>& node)
    {
      init_dim(node.lower_x,node.upper_x,lower_x,upper_x,start.x,scale.x);
      init_dim(node.lower_y,node.upper_y,lower_y,upper_y,start.y,scale.y);
      init_dim(node.lower_z,node.upper_z,lower_z,upper_z,start.z,scale.z);
    }
    
    __forceinline vbool<N> validMask() const { return vint<N>::loadu(lower_x) <= vint<N>::loadu(upper_x); }
    
#if defined(__AVX512F__) // KNL
    __forceinline vbool16 validMask16() const { return le(0xff,vint<16>::loadu(lower_x),vint<16>::loadu(upper_x)); }
#endif
    __forceinline vfloat<N> dequantizeLowerX() const { return madd(vfloat<N>(vint<N>::loadu(lower_x)),scale.x,vfloat<N>(start.x)); }
    
    __forceinline vfloat<N> dequantizeUpperX() const { return madd(vfloat<N>(vint<N>::loadu(upper_x)),scale.x,vfloat<N>(start.x)); }
    
    __forceinline vfloat<N> dequantizeLowerY() const { return madd(vfloat<N>(vint<N>::loadu(lower_y)),scale.y,vfloat<N>(start.y)); }
    
    __forceinline vfloat<N> dequantizeUpperY() const { return madd(vfloat<N>(vint<N>::loadu(upper_y)),scale.y,vfloat<N>(start.y)); }
    
    __forceinline vfloat<N> dequantizeLowerZ() const { return madd(vfloat<N>(vint<N>::loadu(lower_z)),scale.z,vfloat<N>(start.z)); }
    
    __forceinline vfloat<N> dequantizeUpperZ() const { return madd(vfloat<N>(vint<N>::loadu(upper_z)),scale.z,vfloat<N>(start.z)); }
    
    template <int M>
      __forceinline vfloat<M> dequantize(const size_t offset) const { return vfloat<M>(vint<M>::loadu(all_planes+offset)); }
    
#if defined(__AVX512F__)
    __forceinline vfloat16 dequantizeLowerUpperX(const vint16 &p) const { return madd(vfloat16(permute(vint<16>::loadu(lower_x),p)),scale.x,vfloat16(start.x)); }
    __forceinline vfloat16 dequantizeLowerUpperY(const vint16 &p) const { return madd(vfloat16(permute(vint<16>::loadu(lower_y),p)),scale.y,vfloat16(start.y)); }
    __forceinline vfloat16 dequantizeLowerUpperZ(const vint16 &p) const { return madd(vfloat16(permute(vint<16>::loadu(lower_z),p)),scale.z,vfloat16(start.z)); }      
#endif
    
    union {
      struct {
        T lower_x[N]; //!< 8bit discretized X dimension of lower bounds of all N children
        T upper_x[N]; //!< 8bit discretized X dimension of upper bounds of all N children
        T lower_y[N]; //!< 8bit discretized Y dimension of lower bounds of all N children
        T upper_y[N]; //!< 8bit discretized Y dimension of upper bounds of all N children
        T lower_z[N]; //!< 8bit discretized Z dimension of lower bounds of all N children
        T upper_z[N]; //!< 8bit discretized Z dimension of upper bounds of all N children
      };
      T all_planes[6*N];
    };
    
    Vec3f start;
    Vec3f scale;
    
    friend std::ostream& operator<<(std::ostream& o, const QuantizedBaseNode_t& n)
    {
      o << "QuantizedBaseNode { " << std::endl;
      o << "  start   " << n.start << std::endl;
      o << "  scale   " << n.scale << std::endl;
      o << "  lower_x " << vuint<N>::loadu(n.lower_x) << std::endl;
      o << "  upper_x " << vuint<N>::loadu(n.upper_x) << std::endl;
      o << "  lower_y " << vuint<N>::loadu(n.lower_y) << std::endl;
      o << "  upper_y " << vuint<N>::loadu(n.upper_y) << std::endl;
      o << "  lower_z " << vuint<N>::loadu(n.lower_z) << std::endl;
      o << "  upper_z " << vuint<N>::loadu(n.upper_z) << std::endl;
      o << "}" << std::endl;
      return o;
    }
    
  };

  template<int N>
    struct __aligned(8) QuantizedNode_t : public BaseNode_t<N>, QuantizedBaseNode_t<N>
  {
    using BaseNode_t<N>::children;
    using QuantizedBaseNode_t<N>::lower_x;
    using QuantizedBaseNode_t<N>::upper_x;
    using QuantizedBaseNode_t<N>::lower_y;
    using QuantizedBaseNode_t<N>::upper_y;
    using QuantizedBaseNode_t<N>::lower_z;
    using QuantizedBaseNode_t<N>::upper_z;
    using QuantizedBaseNode_t<N>::start;
    using QuantizedBaseNode_t<N>::scale;
    using QuantizedBaseNode_t<N>::init_dim;
    
    __forceinline void setRef(size_t i, const NodeRefPtr<N>& ref) {
      assert(i < N);
      children[i] = ref;
    }
    
    struct Create2
    {
      template<typename BuildRecord>
      __forceinline NodeRefPtr<N> operator() (BuildRecord* children, const size_t n, const FastAllocator::CachedAllocator& alloc) const
      {
        __aligned(64) AlignedNode_t<N> node;
        node.clear();
        for (size_t i=0; i<n; i++) {
          node.setBounds(i,children[i].bounds());
        }
        QuantizedNode_t *qnode = (QuantizedNode_t*) alloc.malloc0(sizeof(QuantizedNode_t), NodeRefPtr<N>::byteAlignment);
        qnode->init(node);
        
        return (size_t)qnode | NodeRefPtr<N>::tyQuantizedNode;
      }
    };
    
    struct Set2
    {
      template<typename BuildRecord>
      __forceinline NodeRefPtr<N> operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRefPtr<N> ref, NodeRefPtr<N>* children, const size_t num) const
      {
        QuantizedNode_t* node = ref.quantizedNode();
        for (size_t i=0; i<num; i++) node->setRef(i,children[i]);
        return ref;
      }
    };
    
    __forceinline void init(AlignedNode_t<N>& node)
    {
      for (size_t i=0;i<N;i++) children[i] = NodeRefPtr<N>::emptyNode;
      init_dim(node);
    }
    
  };
  
  
  /*! BVHN Quantized Node */
  template<int N>
    struct __aligned(8) QuantizedBaseNodeMB_t
  {
    QuantizedBaseNode_t<N> node0;
    QuantizedBaseNode_t<N> node1;
    
    /*! Clears the node. */
    __forceinline void clear() {
      node0.clear();
      node1.clear();
    }
    
    /*! Returns bounds of specified child. */
    __forceinline BBox3fa bounds(size_t i) const
    {
      assert(i < N);
      BBox3fa bounds0 = node0.bounds(i);
      BBox3fa bounds1 = node1.bounds(i);
      bounds0.extend(bounds1);
      return bounds0;
    }
    
    /*! Returns extent of bounds of specified child. */
    __forceinline Vec3fa extent(size_t i) const {
      return bounds(i).size();
    }
    
    __forceinline vbool<N> validMask() const { return node0.validMask(); }
    
    template<typename T>
      __forceinline vfloat<N> dequantizeLowerX(const T t) const { return lerp(node0.dequantizeLowerX(),node1.dequantizeLowerX(),t); }
    template<typename T>
      __forceinline vfloat<N> dequantizeUpperX(const T t) const { return lerp(node0.dequantizeUpperX(),node1.dequantizeUpperX(),t); }
    template<typename T>
      __forceinline vfloat<N> dequantizeLowerY(const T t) const { return lerp(node0.dequantizeLowerY(),node1.dequantizeLowerY(),t); }
    template<typename T>
      __forceinline vfloat<N> dequantizeUpperY(const T t) const { return lerp(node0.dequantizeUpperY(),node1.dequantizeUpperY(),t); }
    template<typename T>
      __forceinline vfloat<N> dequantizeLowerZ(const T t) const { return lerp(node0.dequantizeLowerZ(),node1.dequantizeLowerZ(),t); }
    template<typename T>
      __forceinline vfloat<N> dequantizeUpperZ(const T t) const { return lerp(node0.dequantizeUpperZ(),node1.dequantizeUpperZ(),t); }
    
    
    template<int M>
      __forceinline vfloat<M> dequantizeLowerX(const size_t i, const vfloat<M> &t) const { return lerp(vfloat<M>(node0.dequantizeLowerX()[i]),vfloat<M>(node1.dequantizeLowerX()[i]),t); }
    template<int M>
      __forceinline vfloat<M> dequantizeUpperX(const size_t i, const vfloat<M> &t) const { return lerp(vfloat<M>(node0.dequantizeUpperX()[i]),vfloat<M>(node1.dequantizeUpperX()[i]),t); }
    template<int M>
      __forceinline vfloat<M> dequantizeLowerY(const size_t i, const vfloat<M> &t) const { return lerp(vfloat<M>(node0.dequantizeLowerY()[i]),vfloat<M>(node1.dequantizeLowerY()[i]),t); }
    template<int M>
      __forceinline vfloat<M> dequantizeUpperY(const size_t i, const vfloat<M> &t) const { return lerp(vfloat<M>(node0.dequantizeUpperY()[i]),vfloat<M>(node1.dequantizeUpperY()[i]),t); }
    template<int M>
      __forceinline vfloat<M> dequantizeLowerZ(const size_t i, const vfloat<M> &t) const { return lerp(vfloat<M>(node0.dequantizeLowerZ()[i]),vfloat<M>(node1.dequantizeLowerZ()[i]),t); }
    template<int M>
      __forceinline vfloat<M> dequantizeUpperZ(const size_t i, const vfloat<M> &t) const { return lerp(vfloat<M>(node0.dequantizeUpperZ()[i]),vfloat<M>(node1.dequantizeUpperZ()[i]),t); }
    
  };
}
