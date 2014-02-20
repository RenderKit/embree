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
#include "geometry/bezier1i.h"

#define BVH4HAIR_WIDTH 8

namespace embree
{
  /*! BVH4 with unaligned bounds. */
  class BVH4Hair : public Bounded
  {
    ALIGNED_CLASS;
  public:
#if BVH4HAIR_WIDTH == 8
    typedef avxb simdb;
    typedef avxi simdi;
    typedef avxf simdf;
#elif BVH4HAIR_WIDTH == 4
    typedef sseb simdb;
    typedef ssei simdi;
    typedef ssef simdf;
#endif
    
    /*! forward declaration of node type */
    struct Node;
    struct AlignedNode;
    struct UnalignedNode;
    typedef AffineSpaceT<LinearSpace3<Vec3<simdf> > > AffineSpaceSOA4;

    /*! branching width of the tree */
    static const size_t N = BVH4HAIR_WIDTH;

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
    static const size_t maxDepth = 32;
    static const size_t maxBuildDepth = 32;
    
    /*! Maximal number of primitive blocks in a leaf. */
    static const size_t maxLeafBlocks = items_mask-2;

    /*! Cost of one traversal step. */
    static const int travCostAligned = 1;
    static const int travCostUnaligned = 3;
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
      BBox3fa bounds;       //!< bounds in transformed space
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
      /*! Clears the node. */
      __forceinline void clear() {
        lower_x = lower_y = lower_z = pos_inf; 
        upper_x = upper_y = upper_z = neg_inf;
        Node::clear();
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds, const NodeRef& childID) 
      {
        assert(i < N);
        lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
        upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
        Node::set(i,childID);
      }

      /*! Returns bounds of specified child. */
      __forceinline const BBox3fa bounds(size_t i) const { 
        assert(i < N);
        const Vec3fa lower(lower_x[i],lower_y[i],lower_z[i]);
        const Vec3fa upper(upper_x[i],upper_y[i],upper_z[i]);
        return BBox3fa(lower,upper);
      }

      /*! Returns the extend of the bounds of the ith child */
      __forceinline Vec3fa extend(size_t i) const {
        assert(i < N);
        return bounds(i).size();
      }

    public:
      simdf lower_x;           //!< X dimension of lower bounds of all 4 children.
      simdf upper_x;           //!< X dimension of upper bounds of all 4 children.
      simdf lower_y;           //!< Y dimension of lower bounds of all 4 children.
      simdf upper_y;           //!< Y dimension of upper bounds of all 4 children.
      simdf lower_z;           //!< Z dimension of lower bounds of all 4 children.
      simdf upper_z;           //!< Z dimension of upper bounds of all 4 children.
    };

    /*! Node with unaligned bounds */
    struct UnalignedNode : public Node
    {
      /*! Clears the node. */
      __forceinline void clear() {
        naabb = one;
        Node::clear();
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const NAABBox3fa& b, const NodeRef& childID) 
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
      AffineSpaceSOA4 naabb;   //!< non-axis aligned bounding boxes (bounds are [0,1] in specified space)
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
      __forceinline const BBox3fa bounds() const 
      {
	const BezierCurve3D curve2D(p0,p1,p2,p3,0.0f,1.0f,0);
	const avx4f pi = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
	const Vec3fa lower(reduce_min(pi.x),reduce_min(pi.y),reduce_min(pi.z));
	const Vec3fa upper(reduce_max(pi.x),reduce_max(pi.y),reduce_max(pi.z));
	const Vec3fa upper_r = reduce_max(pi.w);
        return enlarge(BBox3fa(lower,upper),upper_r);
      }

      /*! calculate bounds in specified coordinate space */
      __forceinline const BBox3fa bounds(const AffineSpace3fa& space) const 
      {
        Vec3fa b0 = xfmPoint(space,p0); b0.w = p0.w;
        Vec3fa b1 = xfmPoint(space,p1); b1.w = p1.w;
        Vec3fa b2 = xfmPoint(space,p2); b2.w = p2.w;
        Vec3fa b3 = xfmPoint(space,p3); b3.w = p3.w;
	const BezierCurve3D curve2D(b0,b1,b2,b3,0.0f,1.0f,0);
	const avx4f pi = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
	const Vec3fa lower(reduce_min(pi.x),reduce_min(pi.y),reduce_min(pi.z));
	const Vec3fa upper(reduce_max(pi.x),reduce_max(pi.y),reduce_max(pi.z));
	const Vec3fa upper_r = reduce_max(pi.w);
        return enlarge(BBox3fa(lower,upper),upper_r);
      }

      /*! subdivide the bezier curve */
      __forceinline void subdivide(Bezier1& left, Bezier1& right) const
      {
        const Vec3fa p00 = p0;
        const Vec3fa p01 = p1;
        const Vec3fa p02 = p2;
        const Vec3fa p03 = p3;
      
        const Vec3fa p10 = (p00 + p01) * 0.5f;
        const Vec3fa p11 = (p01 + p02) * 0.5f;
        const Vec3fa p12 = (p02 + p03) * 0.5f;
        const Vec3fa p20 = (p10 + p11) * 0.5f;
        const Vec3fa p21 = (p11 + p12) * 0.5f;
        const Vec3fa p30 = (p20 + p21) * 0.5f;
        
        const float t01 = t0 + dt * 0.5f;
        const float t1  = t0 + dt;
        const unsigned int geomID = this->geomID;
        const unsigned int primID = this->primID;
        
        new (&left ) Bezier1(p00,p10,p20,p30,t0,t01,geomID,primID);
        new (&right) Bezier1(p30,p21,p12,p03,t01,t1,geomID,primID);
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

    /*! BVH4Hair default constructor. */
    BVH4Hair (Scene* scene);

    /*! BVH4Hair destruction */
    ~BVH4Hair ();

    /*! BVH4Hair instantiations */
    static Accel* BVH4HairBezier1(Scene* scene);

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
      return NodeRef((size_t)data | (2+min(num,(size_t)maxLeafBlocks)));
    }

  public:
    Scene* scene;
    NodeRef root;  //!< Root node
    size_t numPrimitives;
    size_t numVertices;
  };
}
