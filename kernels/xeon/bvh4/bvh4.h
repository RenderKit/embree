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

namespace embree
{
  /*! Multi BVH with 4 children. Each node stores the bounding box of
   * it's 4 children as well as 4 child pointers. */
  class BVH4 : public Bounded
  {
    ALIGNED_CLASS;
  public:
    
    /*! forward declaration of node type */
    struct Node;
    struct NodeMB;

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
    static const size_t tyLeaf = 2;

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
      __forceinline void prefetch() const {
	prefetchL1(((char*)ptr)+0*64);
	prefetchL1(((char*)ptr)+1*64);
      }

      /*! Sets the barrier bit. */
      __forceinline void setBarrier() { ptr |= barrier_mask; }
      
      /*! Clears the barrier bit. */
      __forceinline void clearBarrier() { ptr &= ~barrier_mask; }

      /*! Checks if this is an barrier. A barrier tells the top level tree rotations how deep to enter the tree. */
      __forceinline bool isBarrier() const { return (ptr & barrier_mask) != 0; }

      /*! checks if this is a leaf */
      __forceinline int isLeaf() const { return (ptr & (size_t)align_mask) >= tyLeaf; }
      
      /*! checks if this is a node */
      __forceinline int isNode() const { return (ptr & (size_t)align_mask) == tyNode; }

      /*! checks if this is a motion blur node */
      __forceinline int isNodeMB() const { return (ptr & (size_t)align_mask) == tyNodeMB; }
      
      /*! returns node pointer */
      __forceinline       Node* node()       { assert(isNode()); return (      Node*)ptr; }
      __forceinline const Node* node() const { assert(isNode()); return (const Node*)ptr; }

      /*! returns motion blur node pointer */
      __forceinline       NodeMB* nodeMB()       { assert(isNodeMB()); return (      NodeMB*)ptr; }
      __forceinline const NodeMB* nodeMB() const { assert(isNodeMB()); return (const NodeMB*)ptr; }
      
      /*! returns leaf pointer */
      __forceinline char* leaf(size_t& num) const {
        assert(isLeaf());
        num = (ptr & (size_t)items_mask)-tyLeaf;
        return (char*)(ptr & ~(size_t)align_mask);
      }

    private:
      size_t ptr;
    };

    /*! BVH4 Node */
    struct Node
    {
      /*! Clears the node. */
      __forceinline void clear() {
        lower_x = lower_y = lower_z = pos_inf; 
        upper_x = upper_y = upper_z = neg_inf;
	for (size_t i=0; i<N; i++) children[i] = emptyNode;
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

      /*! Returns bounds of all children */
      __forceinline void bounds(BBox<ssef>& bounds0, BBox<ssef>& bounds1, BBox<ssef>& bounds2, BBox<ssef>& bounds3) const {
        transpose(lower_x,lower_y,lower_z,ssef(zero),bounds0.lower,bounds1.lower,bounds2.lower,bounds3.lower);
        transpose(upper_x,upper_y,upper_z,ssef(zero),bounds0.upper,bounds1.upper,bounds2.upper,bounds3.upper);
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
      ssef lower_x;           //!< X dimension of lower bounds of all 4 children.
      ssef upper_x;           //!< X dimension of upper bounds of all 4 children.
      ssef lower_y;           //!< Y dimension of lower bounds of all 4 children.
      ssef upper_y;           //!< Y dimension of upper bounds of all 4 children.
      ssef lower_z;           //!< Z dimension of lower bounds of all 4 children.
      ssef upper_z;           //!< Z dimension of upper bounds of all 4 children.
      NodeRef children[N];    //!< Pointer to the 4 children (can be a node or leaf)
    };

    /*! Motion Blur Node */
    struct NodeMB
    {
      /*! Clears the node. */
      __forceinline void clear()  {
        lower_x = lower_y = lower_z = ssef(pos_inf);
        upper_x = upper_y = upper_z = ssef(neg_inf);
        lower_dx = lower_dy = lower_dz = ssef(nan); // initialize with NAN and update during refit
        upper_dx = upper_dy = upper_dz = ssef(nan);
        children[0] = children[1] = children[2] = children[3] = emptyNode;
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
      ssef lower_x;        //!< X dimension of lower bounds of all 4 children.
      ssef upper_x;        //!< X dimension of upper bounds of all 4 children.
      ssef lower_y;        //!< Y dimension of lower bounds of all 4 children.
      ssef upper_y;        //!< Y dimension of upper bounds of all 4 children.
      ssef lower_z;        //!< Z dimension of lower bounds of all 4 children.
      ssef upper_z;        //!< Z dimension of upper bounds of all 4 children.

      ssef lower_dx;        //!< X dimension of lower bounds of all 4 children.
      ssef upper_dx;        //!< X dimension of upper bounds of all 4 children.
      ssef lower_dy;        //!< Y dimension of lower bounds of all 4 children.
      ssef upper_dy;        //!< Y dimension of upper bounds of all 4 children.
      ssef lower_dz;        //!< Z dimension of lower bounds of all 4 children.
      ssef upper_dz;        //!< Z dimension of upper bounds of all 4 children.

      NodeRef children[4];           //!< Pointer to the 4 children (can be a node or leaf)
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
    BVH4 (const PrimitiveType& primTy, void* geometry = NULL);

    /*! BVH4 destruction */
    ~BVH4 ();

    /*! BVH4 instantiations */
    static Accel* BVH4Bezier1(Scene* scene);
    static Accel* BVH4Bezier1i(Scene* scene);
    static Accel* BVH4Triangle1(Scene* scene);
    static Accel* BVH4Triangle4(Scene* scene);
    static Accel* BVH4Triangle8(Scene* scene);
    static Accel* BVH4Triangle1v(Scene* scene);
    static Accel* BVH4Triangle4v(Scene* scene);
    static Accel* BVH4Triangle4i(Scene* scene);
    static Accel* BVH4UserGeometry(Scene* scene);
    
    static Accel* BVH4BVH4Triangle1Morton(Scene* scene);
    static Accel* BVH4BVH4Triangle1ObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle4ObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle1vObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle4vObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle4iObjectSplit(Scene* scene);

    static Accel* BVH4Triangle1SpatialSplit(Scene* scene);
    static Accel* BVH4Triangle4SpatialSplit(Scene* scene);
    static Accel* BVH4Triangle8SpatialSplit(Scene* scene);
    static Accel* BVH4Triangle1ObjectSplit(Scene* scene);
    static Accel* BVH4Triangle4ObjectSplit(Scene* scene);
    static Accel* BVH4Triangle8ObjectSplit(Scene* scene);
    static Accel* BVH4Triangle1vObjectSplit(Scene* scene);
    static Accel* BVH4Triangle4vObjectSplit(Scene* scene);
    static Accel* BVH4Triangle4iObjectSplit(Scene* scene);

    static Accel* BVH4Triangle1ObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle4ObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle1vObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle4vObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle4Refit(TriangleMesh* mesh);

    /*! initializes the acceleration structure */
    void init (size_t numPrimitives = 0, size_t numThreads = 1);

    /*! Clears the barrier bits of a subtree. */
    void clearBarrier(NodeRef& node);

    /*! Propagate bounds for time t0 and time t1 up the tree. */
    std::pair<BBox3fa,BBox3fa> refit(void* geom, NodeRef node);

    LinearAllocatorPerThread alloc;

    __forceinline Node* allocNode(size_t thread) {
      Node* node = (Node*) alloc.malloc(thread,sizeof(Node),1 << alignment); node->clear(); return node;
    }

    __forceinline NodeMB* allocNodeMB(size_t thread) {
      NodeMB* node = (NodeMB*) alloc.malloc(thread,sizeof(NodeMB),1 << alignment); node->clear(); return node;
    }

    __forceinline char* allocPrimitiveBlocks(size_t thread, size_t num) {
      return (char*) alloc.malloc(thread,num*primTy.bytes,1 << alignment);
    }

    /*! Encodes a node */
    __forceinline NodeRef encodeNode(Node* node) { 
      assert(!((size_t)node & align_mask)); 
      return NodeRef((size_t) node);
    }

    /*! Encodes a node */
    __forceinline NodeRef encodeNode(NodeMB* node) { 
      assert(!((size_t)node & align_mask)); 
      return NodeRef((size_t) node + tyNodeMB);
    }
    
    /*! Encodes a leaf */
    __forceinline NodeRef encodeLeaf(void* tri, size_t num) {
      assert(!((size_t)tri & align_mask)); 
      return NodeRef((size_t)tri | (tyLeaf+min(num,(size_t)maxLeafBlocks)));
    }
    
  public:
    
    /*! calculates the amount of bytes allocated */
    size_t bytesAllocated() {
      return alloc.bytes();
    }

  public:
    const PrimitiveType& primTy;       //!< primitive type stored in the BVH
    void* geometry;                    //!< pointer to additional data for primitive intersector
    NodeRef root;                      //!< Root node
    size_t numPrimitives;
    size_t numVertices;

    /*! data arrays for fast builders */
  public:
    std::vector<BVH4*> objects;
  };

  // FIXME: move the below code to somewhere else
  typedef void (*createTriangleMeshAccelTy)(TriangleMesh* mesh, BVH4*& accel, Builder*& builder); 
  typedef Builder* (*BVH4BuilderTopLevelFunc)(BVH4* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel);

#define DECLARE_TOPLEVEL_BUILDER(symbol)                                         \
  namespace isa   { extern Builder* symbol(BVH4* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  namespace sse41 { extern Builder* symbol(BVH4* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  namespace avx   { extern Builder* symbol(BVH4* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  namespace avx2  { extern Builder* symbol(BVH4* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  BVH4BuilderTopLevelFunc symbol;
}
