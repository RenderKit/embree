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

/* include all node types */
#include "bvh_node_aabb.h"
#include "bvh_node_aabb_mb.h"
#include "bvh_node_aabb_mb4d.h"
#include "bvh_node_obb.h"
#include "bvh_node_obb_mb.h"
#include "bvh_node_qaabb.h"

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
  
  /*! Multi BVH with N children. Each node stores the bounding box of
   * it's N children as well as N child references. */
  template<int N>
    class BVHN : public AccelData
  {
    ALIGNED_CLASS_(16);
  public:
    
    /*! forward declaration of node ref type */
    typedef NodeRefPtr<N> NodeRef;
    typedef BaseNode_t<N> BaseNode;
    typedef AlignedNode_t<N> AlignedNode;
    typedef AlignedNodeMB_t<N> AlignedNodeMB;
    typedef AlignedNodeMB4D_t<N> AlignedNodeMB4D;
    typedef UnalignedNode_t<N> UnalignedNode;
    typedef UnalignedNodeMB_t<N> UnalignedNodeMB;
    typedef QuantizedBaseNode_t<N> QuantizedBaseNode;
    typedef QuantizedBaseNodeMB_t<N> QuantizedBaseNodeMB;
    typedef QuantizedNode_t<N> QuantizedNode;
    
    /*! Number of bytes the nodes and primitives are minimally aligned to.*/
    static const size_t byteAlignment = 16;
    static const size_t byteNodeAlignment = 4*N;
    
    /*! Empty node */
    static const size_t emptyNode = NodeRef::emptyNode;
    
    /*! Invalid node, used as marker in traversal */
    static const size_t invalidNode = NodeRef::invalidNode;
    static const size_t popRay      = NodeRef::popRay;
    
    /*! Maximum depth of the BVH. */
    static const size_t maxBuildDepth = 32;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+8;
    static const size_t maxDepth = 2*maxBuildDepthLeaf; // 2x because of two level builder
    
    /*! Maximum number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = NodeRef::maxLeafBlocks;
    
  public:
    
    /*! Builder interface to create allocator */
    struct CreateAlloc : public FastAllocator::Create {
      __forceinline CreateAlloc (BVHN* bvh) : FastAllocator::Create(&bvh->alloc) {}
    };
    
    typedef BVHNodeRecord<NodeRef>     NodeRecord;
    typedef BVHNodeRecordMB<NodeRef>   NodeRecordMB;
    typedef BVHNodeRecordMB4D<NodeRef> NodeRecordMB4D;
    
  public:
    
    /*! BVHN default constructor. */
    BVHN (const PrimitiveType& primTy, Scene* scene);
    
    /*! BVHN destruction */
    ~BVHN ();
    
    /*! clears the acceleration structure */
    void clear();
    
    /*! sets BVH members after build */
    void set (NodeRef root, const LBBox3fa& bounds, size_t numPrimitives);
    
    /*! Clears the barrier bits of a subtree. */
    void clearBarrier(NodeRef& node);
    
    /*! lays out num large nodes of the BVH */
    void layoutLargeNodes(size_t num);
    NodeRef layoutLargeNodesRecursion(NodeRef& node, const FastAllocator::CachedAllocator& allocator);
    
    /*! called by all builders before build starts */
    double preBuild(const std::string& builderName);
    
    /*! called by all builders after build ended */
    void postBuild(double t0);
    
    /*! allocator class */
    struct Allocator {
      BVHN* bvh;
      Allocator (BVHN* bvh) : bvh(bvh) {}
      __forceinline void* operator() (size_t bytes) const { 
        return bvh->alloc._threadLocal()->malloc(&bvh->alloc,bytes); 
      }
    };
    
    /*! post build cleanup */
    void cleanup() {
      alloc.cleanup();
    }
    
  public:
    
    /*! Encodes a node */
    static __forceinline NodeRef encodeNode(AlignedNode* node) {
      assert(!((size_t)node & NodeRef::align_mask));
      return NodeRef((size_t) node);
    }
    
    static __forceinline unsigned int encodeQuantizedNode(size_t base, size_t node) {
      assert(!((size_t)node & NodeRef::align_mask));
      ssize_t node_offset = (ssize_t)node-(ssize_t)base;
      assert(node_offset != 0);
      assert((int64_t)node_offset >= -int64_t(0x80000000) && (int64_t)node_offset <= (int64_t)0x7fffffff);
      return (unsigned int)node_offset | NodeRef::tyQuantizedNode;
    }
    
    static __forceinline int encodeQuantizedLeaf(size_t base, size_t node) {
      ssize_t leaf_offset = (ssize_t)node-(ssize_t)base;
      assert((int64_t)leaf_offset >= -int64_t(0x80000000) && (int64_t)leaf_offset <= (int64_t)0x7fffffff);
      return (int)leaf_offset;
    }
    
    static __forceinline NodeRef encodeNode(AlignedNodeMB* node) {
      assert(!((size_t)node & NodeRef::align_mask));
      return NodeRef((size_t) node | NodeRef::tyAlignedNodeMB);
    }
    
    static __forceinline NodeRef encodeNode(AlignedNodeMB4D* node) {
      assert(!((size_t)node & NodeRef::align_mask));
      return NodeRef((size_t) node | NodeRef::tyAlignedNodeMB4D);
    }
    
    /*! Encodes an unaligned node */
    static __forceinline NodeRef encodeNode(UnalignedNode* node) {
      return NodeRef((size_t) node | NodeRef::tyUnalignedNode);
    }
    
    /*! Encodes an unaligned motion blur node */
    static __forceinline NodeRef encodeNode(UnalignedNodeMB* node) {
      return NodeRef((size_t) node | NodeRef::tyUnalignedNodeMB);
    }
    
    /*! Encodes a leaf */
    static __forceinline NodeRef encodeLeaf(void* tri, size_t num) {
      assert(!((size_t)tri & NodeRef::align_mask));
      assert(num <= maxLeafBlocks);
      return NodeRef((size_t)tri | (NodeRef::tyLeaf+min(num,(size_t)maxLeafBlocks)));
    }
    
    /*! Encodes a leaf */
    static __forceinline NodeRef encodeTypedLeaf(void* ptr, size_t ty) {
      assert(!((size_t)ptr & NodeRef::align_mask));
      return NodeRef((size_t)ptr | (NodeRef::tyLeaf+ty));
    }

  public:
    
    /*! Prefetches the node this reference points to */
    __forceinline static void prefetch(const NodeRef ref, int types=0)
    {
#if defined(__AVX512PF__) // MIC
      if (types != BVH_FLAG_QUANTIZED_NODE) {
        prefetchL2(((char*)ref.ptr)+0*64);
        prefetchL2(((char*)ref.ptr)+1*64);
        if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
          prefetchL2(((char*)ref.ptr)+2*64);
          prefetchL2(((char*)ref.ptr)+3*64);
        }
        if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
          /* KNL still needs L2 prefetches for large nodes */
          prefetchL2(((char*)ref.ptr)+4*64);
          prefetchL2(((char*)ref.ptr)+5*64);
          prefetchL2(((char*)ref.ptr)+6*64);
          prefetchL2(((char*)ref.ptr)+7*64);
        }
      }
      else
      {
        /* todo: reduce if 32bit offsets are enabled */
        prefetchL2(((char*)ref.ptr)+0*64);
        prefetchL2(((char*)ref.ptr)+1*64);
        prefetchL2(((char*)ref.ptr)+2*64);
      }
#else
      if (types != BVH_FLAG_QUANTIZED_NODE) {
        prefetchL1(((char*)ref.ptr)+0*64);
        prefetchL1(((char*)ref.ptr)+1*64);
        if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
          prefetchL1(((char*)ref.ptr)+2*64);
          prefetchL1(((char*)ref.ptr)+3*64);
        }
        if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
          /* deactivate for large nodes on Xeon, as it introduces regressions */
          //prefetchL1(((char*)ref.ptr)+4*64);
          //prefetchL1(((char*)ref.ptr)+5*64);
          //prefetchL1(((char*)ref.ptr)+6*64);
          //prefetchL1(((char*)ref.ptr)+7*64);
        }
      }
      else
      {
        /* todo: reduce if 32bit offsets are enabled */
        prefetchL1(((char*)ref.ptr)+0*64);
        prefetchL1(((char*)ref.ptr)+1*64);
        prefetchL1(((char*)ref.ptr)+2*64);
      }
#endif
    }
    
    __forceinline static void prefetchW(const NodeRef ref, int types=0)
    {
      embree::prefetchEX(((char*)ref.ptr)+0*64);
      embree::prefetchEX(((char*)ref.ptr)+1*64);
      if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
        embree::prefetchEX(((char*)ref.ptr)+2*64);
        embree::prefetchEX(((char*)ref.ptr)+3*64);
      }
      if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
        embree::prefetchEX(((char*)ref.ptr)+4*64);
        embree::prefetchEX(((char*)ref.ptr)+5*64);
        embree::prefetchEX(((char*)ref.ptr)+6*64);
        embree::prefetchEX(((char*)ref.ptr)+7*64);
      }
    }
    
    /*! bvh type information */
  public:
    const PrimitiveType* primTy;       //!< primitive type stored in the BVH
    
    /*! bvh data */
  public:
    Device* device;                    //!< device pointer
    Scene* scene;                      //!< scene pointer
    NodeRef root;                      //!< root node
    FastAllocator alloc;               //!< allocator used to allocate nodes
    
    /*! statistics data */
  public:
    size_t numPrimitives;              //!< number of primitives the BVH is build over
    size_t numVertices;                //!< number of vertices the BVH references
    
    /*! data arrays for special builders */
  public:
    std::vector<BVHN*> objects;
    vector_t<char,aligned_allocator<char,32>> subdiv_patches;
  };
  
  template<>
    __forceinline void BVHN<4>::AlignedNode::bounds(BBox<vfloat4>& bounds0, BBox<vfloat4>& bounds1, BBox<vfloat4>& bounds2, BBox<vfloat4>& bounds3) const {
    transpose(lower_x,lower_y,lower_z,vfloat4(zero),bounds0.lower,bounds1.lower,bounds2.lower,bounds3.lower);
    transpose(upper_x,upper_y,upper_z,vfloat4(zero),bounds0.upper,bounds1.upper,bounds2.upper,bounds3.upper);
  }
  
  typedef BVHN<4> BVH4;
  typedef BVHN<8> BVH8;
}
