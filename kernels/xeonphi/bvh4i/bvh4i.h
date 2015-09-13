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

#include "../../common/alloc.h"
#include "../../common/accel.h"
#include "../../common/device.h"
#include "../../common/scene.h"
#include "../geometry/primitive.h"
#include "../geometry/triangle1.h"
#include "../geometry/virtual_accel.h"
#include "../geometry/subdivpatch1.h"

#define BVH4I_TOP_LEVEL_MARKER 0x80000000

namespace embree
{
  /*! Multi BVH with 4 children. Each node stores the bounding box of
   * it's 4 children as well as a 4 child indices. */
  class BVH4i : public AccelData
  {
  public:

    /*! forward declaration of node type */
    struct Node;

    /*! branching width of the tree */
    static const size_t N = 4;

    /*! Masks the bits that store the number of items per leaf. */
    static const unsigned int encodingBits  = 4;
    static const unsigned int offset_mask   = 0xFFFFFFFF << encodingBits;
    static const unsigned int leaf_shift    = 3;
    static const unsigned int leaf_mask     = 1<<leaf_shift;  
    static const unsigned int items_mask    = (1<<(leaf_shift-1))-1;//leaf_mask-1;  
    static const unsigned int aux_flag_mask = 1<<(leaf_shift-1);
    /*! Empty node */
    static const unsigned int emptyNode = leaf_mask;

    /*! Invalid node */
    static const unsigned int invalidNode = (unsigned int)-1;

    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 26;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+6;
    static const size_t maxDepth = maxBuildDepth + maxBuildDepthLeaf;
    
    static const size_t hybridSIMDUtilSwitchThreshold = 7;

    /*! References a Node or list of Triangles */
    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (unsigned id) : _id(id) { }

      /*! Cast to unsigned */
      __forceinline operator unsigned int() const { return _id; }
     
      /*! checks if this is a leaf */
      __forceinline unsigned int isLeaf() const { return _id & leaf_mask; }

      /*! checks if this is a leaf */
      __forceinline unsigned int isLeaf(const unsigned int mask) const { return _id & mask; }
      
      /*! checks if this is a node */
      __forceinline unsigned int isNode() const { return (_id & leaf_mask) == 0; }

      /*! checks if the aux flag is set */
      __forceinline unsigned int isAuxFlagSet() const { return _id & aux_flag_mask; }

      /*! checks if the aux flag is set */
      __forceinline unsigned int clearAuxFlag() const { return _id & (~aux_flag_mask); }
      
      /*! returns node pointer */

      // use free adressing mode: lea reg,reg*2 as adressing is done with respect to 32 bytes blocks
      __forceinline       Node* node(      void* base) const { return (      Node*)((      short*)base + (size_t)_id); }
      __forceinline const Node* node(const void* base) const { return (const Node*)((const short*)base + (size_t)_id); }
      

      __forceinline unsigned int nodeID() const { return (_id*2) / sizeof(Node);  }

      __forceinline unsigned int items() const {
        return (_id & items_mask)+1;
      }
      
      /*! returns leaf pointer */
      template<unsigned int scale=4>
      __forceinline const char* leaf(const void* base, unsigned int& num) const {
        assert(isLeaf());
        num = items();
	if (scale == 4)
	  return (const char*)((const int*)base + (size_t)(_id & offset_mask));
	else
	  return (const char*)base + (_id & offset_mask)*scale;
      }

      /*! returns leaf pointer */
      template<unsigned int scale=4>
      __forceinline const char* leaf(const void* base) const {
        assert(isLeaf());
	if (scale == 4)
	  return (const char*)((const int*)base + (_id & offset_mask));
	else
	  return (const char*)base + (_id & offset_mask)*scale;
      }

      __forceinline unsigned int offset() const {
        return _id & offset_mask;
      }

      __forceinline unsigned int offsetIndex() const {
        return _id >> encodingBits;
      }

      
      __forceinline unsigned int &id() { return _id; }
    private:
      unsigned int _id;
    };

    /*! BVH4i Node */

    struct __aligned(64) Node
    {
    public:
      struct NodeStruct {
        float x,y,z;           // x,y, and z coordinates of bounds
        NodeRef child;         
      } lower[4], upper[4];    

      /*! Returns bounds of specified child. */
      __forceinline BBox3fa bounds(size_t i) const {
	assert( i < 4 );
        Vec3fa l = *(Vec3fa*)&lower[i];
        Vec3fa u = *(Vec3fa*)&upper[i];
        return BBox3fa(l,u);
      }

      __forceinline size_t numChildren() const {
	int16 c = load16i((int*)lower);
	const size_t children = countbits(ne(0x8888,c,int16(BVH4i::invalidNode))); 
	assert(children >=2 && children <= 4);
	return children;
      }

      __forceinline BBox3fa bounds() const {
	return merge( bounds(0),bounds(1),bounds(2),bounds(3) );
      }

      __forceinline float halfAreaBounds(size_t i) const {
	assert( i < 4 );
        return halfArea( bounds(i) );
      }

      __forceinline float16 halfAreaBounds() const {
	const float16 l = load16f(lower);
	const float16 u = load16f(upper);
	const float16 diag = u-l;
	const float16 dx = swAAAA(diag);
	const float16 dy = swBBBB(diag);
	const float16 dz = swCCCC(diag);
	const float16 half_area = dx*(dy+dz)+dy*dz; 
	return half_area;
      }

      __forceinline void setBounds(size_t i, const BBox3fa &b) {
	assert( i < 4 );
	lower[i].x = b.lower.x;
	lower[i].y = b.lower.y;
	lower[i].z = b.lower.z;

	upper[i].x = b.upper.x;
	upper[i].y = b.upper.y;
	upper[i].z = b.upper.z;
      }

      __forceinline void setBounds(size_t i, const Node *__restrict__ const n) {
	assert( i < 4 );
	const float16 l = min(min(n->lowerXYZ(0),n->lowerXYZ(1)),min(n->lowerXYZ(2),n->lowerXYZ(3)));
	const float16 u = max(max(n->upperXYZ(0),n->upperXYZ(1)),max(n->upperXYZ(2),n->upperXYZ(3)));

	store3f(&lower[i],l);
	store3f(&upper[i],u);
      }

      __forceinline float16 lowerXYZ(size_t i) const {
	return broadcast4to16f(&lower[i]);
      }

      __forceinline float16 upperXYZ(size_t i) const {
	return broadcast4to16f(&upper[i]);
      }

      __forceinline bool isPoint(size_t i) const {
	bool16 m_lane = ((unsigned int)0x7) << (4*i);
	bool16 m_box  = eq(m_lane,load16f(lower),load16f(upper));
	return (unsigned int)m_box == (unsigned int)m_lane;
      }

      struct Helper { float x,y,z; int a; }; 
      static Helper initQBVHNode[4];

      __forceinline void setInvalid(size_t i) 
      {
	lower[i].x = pos_inf;
	lower[i].y = pos_inf;
	lower[i].z = pos_inf;
	lower[i].child = invalidNode; 

	upper[i].x = neg_inf;
	upper[i].y = neg_inf;
	upper[i].z = neg_inf;
	upper[i].child = NodeRef(0);
      }

      __forceinline void setValid(size_t i) 
      {
	lower[i].x = neg_inf;
	lower[i].y = neg_inf;
	lower[i].z = neg_inf;
	lower[i].child = invalidNode; 

	upper[i].x = pos_inf;
	upper[i].y = pos_inf;
	upper[i].z = pos_inf;
	upper[i].child = NodeRef(0);
      }

      __forceinline void setInvalid() 
      {
#if 1
	float16 lower = broadcast4to16f(&initQBVHNode[0]);
	float16 upper = broadcast4to16f(&initQBVHNode[1]);
	store16f_ngo(((float16*)this)+0,lower); 
	store16f_ngo(((float16*)this)+1,upper);             
#else
	for (size_t i=0;i<4;i++)
	  setInvalid(i);
#endif
      }

      __forceinline void setValid() 
      {
	for (size_t i=0;i<4;i++)
	  setValid(i);
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return lower[i].child; }
      __forceinline const NodeRef& child(size_t i) const { return lower[i].child; }


      /*! Returns reference to specified child */
      __forceinline       NodeRef& data(size_t i)       { return upper[i].child; }
      __forceinline const NodeRef& data(size_t i) const { return upper[i].child; }

      template<int PFHINT>
	__forceinline void prefetchNode() const
	{
	  prefetch<PFHINT>(lower);
	  prefetch<PFHINT>(upper);
	}


      __forceinline std::ostream& operator<<(std::ostream &o)
      {
	for (size_t i=0;i<4;i++)
	  {
	    o << "lower: [" << lower[i].x << "," << lower[i].y << "," << lower[i].z << "] ";
	    o << "upper: [" << upper[i].x << "," << upper[i].y << "," << upper[i].z << "] ";
	    o << std::endl;
	  }
	return o;
      }



    };


    /*! 64bit byte-quantized BVH4i Node */

    struct __aligned(64) QuantizedNode
    {
    public:
      Vec3f start;
      NodeRef child0;
      Vec3f diff;
      NodeRef child1;
      unsigned char lower[12];
      NodeRef child2;
      unsigned char upper[12];
      NodeRef child3;

      /*! Returns reference to specified child */
      __forceinline       NodeRef &child(size_t i)       { return ((NodeRef*)this)[3+4*i]; }
      __forceinline const NodeRef &child(size_t i) const { return ((NodeRef*)this)[3+4*i]; }

      __forceinline float16 lowerXYZ() const
      {
	return uload16f_low_uint8(0x7777,lower,float16::zero());
      }

      __forceinline float16 decompress_lowerXYZ(const float16 &s, const float16 &d)  const
      {
	return madd_round_down(d,lowerXYZ(),s);
      }

      __forceinline float16 decompress_upperXYZ(const float16 &s, const float16 &d)  const
      {	
	return madd_round_up(d,upperXYZ(),s);
      }

      __forceinline float16 upperXYZ()  const
      {
	return uload16f_low_uint8(0x7777,upper,float16::zero());
      }

      __forceinline bool isPoint(size_t i) const {
	bool16 m_lane = ((unsigned int)0x7) << (4*i);
	bool16 m_box  = eq(m_lane,lowerXYZ(),upperXYZ());
	return (unsigned int)m_box == (unsigned int)m_lane;
      }

      __forceinline BBox3fa bounds(size_t i) const {
	assert( i < 4 );
	const float16 s = decompress_startXYZ();
	const float16 d = decompress_diffXYZ();

	const float16 decompress_lower_XYZ = decompress_lowerXYZ(s,d);
	const float16 decompress_upper_XYZ = decompress_upperXYZ(s,d);

        Vec3fa l = ((Vec3fa*)&decompress_lower_XYZ)[i];
        Vec3fa u = ((Vec3fa*)&decompress_upper_XYZ)[i];
        return BBox3fa(l,u);
      }


      __forceinline float16 decompress_startXYZ() const
      {
	return broadcast4to16f(&start);
      }

      __forceinline float16 decompress_diffXYZ() const
      {
	return broadcast4to16f(&diff);
      }

      __forceinline void init( const Node &node) 
      {
	float16 l0 = node.lowerXYZ(0);
	float16 l1 = node.lowerXYZ(1);
	float16 l2 = node.lowerXYZ(2);
	float16 l3 = node.lowerXYZ(3);

	float16 u0 = node.upperXYZ(0);
	float16 u1 = node.upperXYZ(1);
	float16 u2 = node.upperXYZ(2);
	float16 u3 = node.upperXYZ(3);

	const float16 minXYZ = select(0x7777,min(min(l0,l1),min(l2,l3)),float16::zero());
	const float16 maxXYZ = select(0x7777,max(max(u0,u1),max(u2,u3)),float16::one());
	const float16 diffXYZ = maxXYZ - minXYZ;

	const float16 nlower = load16f(node.lower);
	const float16 nupper = load16f(node.upper);
	const bool16 isInvalid = eq(0x7777,nlower,pos_inf);

	const float16 node_lowerXYZ = select(bool16(0x7777) ^ isInvalid,nlower,minXYZ); 
	const float16 node_upperXYZ = select(bool16(0x7777) ^ isInvalid,nupper,minXYZ); 

	float16 local_lowerXYZ = floor(( (node_lowerXYZ - minXYZ) * float16(255.0f) / diffXYZ) /* - 0.5f */);
	float16 local_upperXYZ =  ceil(( (node_upperXYZ - minXYZ) * float16(255.0f) / diffXYZ) /* + 0.5f */);

	store4f(&start,minXYZ);
	store4f(&diff ,mul_round_up(diffXYZ,(1.0f/255.0f)));
	compactustore16f_low_uint8(0x7777,lower,local_lowerXYZ);
	compactustore16f_low_uint8(0x7777,upper,local_upperXYZ);

	child0 = node.child(0);
	child1 = node.child(1);
	child2 = node.child(2);
	child3 = node.child(3);

#if DEBUG

	const float16 s = decompress_startXYZ();
	const float16 d = decompress_diffXYZ();

	const float16 decompress_lower_XYZ = decompress_lowerXYZ(s,d);
	const float16 decompress_upper_XYZ = decompress_upperXYZ(s,d);

	if ( any(gt(0x7777,decompress_lower_XYZ,node_lowerXYZ)) ) 
	   { 
	     PRINT(node_lowerXYZ);  
	     PRINT(decompress_lower_XYZ); 
	     PRINT(decompress_lower_XYZ-node_lowerXYZ); 
	   } 

	if ( any(lt(0x7777,decompress_upper_XYZ,node_upperXYZ)) )
	  {
	    PRINT(node_upperXYZ);
	    PRINT(decompress_upper_XYZ);
	    PRINT(decompress_upper_XYZ-node_upperXYZ);
	  }
#endif
      }

      template<int PFHINT>
	__forceinline void prefetchNode() const
	{
	  prefetch<PFHINT>(this);
	}

    };

  public:

    /*! BVH4 default constructor. */
    BVH4i (const PrimitiveType& primTy, Scene* scene)
      : AccelData(AccelData::TY_UNKNOWN), primTy(primTy),
      device(scene->device),
      geometry(scene), 
      root(emptyNode), 
      qbvh(nullptr), 
      accel(nullptr),
      size_node(0),
      size_accel(0),
      numAllocated64BytesBlocks(0),
      numPrimitives(0)
    {
    }

    ~BVH4i();

    void clear() {}

    /*! BVH4i instantiations */
    static Accel* BVH4iTriangle1ObjectSplitBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1ObjectSplitMorton(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1ObjectSplitEnhancedMorton(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1PreSplitsBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iVirtualGeometryBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1MemoryConservativeBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1ObjectSplitMorton64Bit(Scene* scene,bool robust);
    static Accel* BVH4iSubdivMeshBinnedSAH(Scene* scene,bool robust);

    /*! Calculates the SAH of the BVH */
    float sah ();

    /*! Data of the BVH */
  public:
    NodeRef root;                  //!< Root node (can also be a leaf).

    const PrimitiveType& primTy;   //!< triangle type stored in BVH
    Device* device;
    Scene* geometry;                  //!< pointer to geometry for intersection

 /*! Memory allocation */
  public:

    __forceinline       void* nodePtr()       { return qbvh; }
    __forceinline const void* nodePtr() const { return qbvh; }

    __forceinline       void* triPtr()       { return accel; }
    __forceinline const void* triPtr() const { return accel; }


    size_t bytes () const {
      return size_node+size_accel;
    }

    size_t size_node;
    size_t size_accel;

    Node *qbvh;
    void *accel;
    

    size_t numAllocated64BytesBlocks;
    size_t numPrimitives;

    /*! swap the children of two nodes */
    __forceinline static void swap(Node* a, size_t i, Node* b, size_t j)
    {
      assert(i<N && j<N);
      const float16 lower_a = broadcast4to16f(&a->lower[i]);
      const float16 upper_a = broadcast4to16f(&a->upper[i]);
      const float16 lower_b = broadcast4to16f(&b->lower[j]);
      const float16 upper_b = broadcast4to16f(&b->upper[j]);

      store4f(&a->lower[i],lower_b);
      store4f(&a->upper[i],upper_b);
      store4f(&b->lower[j],lower_a);
      store4f(&b->upper[j],upper_a);
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
          swap(a,i,a,j);
          for (j=j-1; j>i; j--)
            if (a->child(j) != emptyNode)
              break;
        }
      }
    }

  private:
    float sah (NodeRef& node, BBox3fa bounds);
  };


  __forceinline std::ostream &operator<<(std::ostream &o, const BVH4i::Node &v)
    {
      o << std::endl;
      o << "lower: ";
      for (size_t i=0;i<4;i++) o << "[" << v.lower[i].x << "," << v.lower[i].y << "," << v.lower[i].z << "," << v.lower[i].child <<"] ";
      o << std::endl;
      o << "upper: ";
      for (size_t i=0;i<4;i++) o << "[" << v.upper[i].x << "," << v.upper[i].y << "," << v.upper[i].z << "," << v.upper[i].child <<"] ";
      o << std::endl;
      return o;
    } 

  __forceinline std::ostream& operator<<(std::ostream &o, BVH4i::QuantizedNode &v)
    {
      o << "start " << v.start << " diff " << v.diff << std::endl;
      o << "lower " << v.decompress_lowerXYZ(v.decompress_startXYZ(),v.decompress_diffXYZ()) << std::endl;
      o << "upper " << v.decompress_upperXYZ(v.decompress_startXYZ(),v.decompress_diffXYZ()) << std::endl;
      o << "child0 " << v.child(0).nodeID() << " child1 " << v.child(1).nodeID() << " child2 " << v.child(2).nodeID() << " child3 " << v.child(3).nodeID() << std::endl;
      return o;
    }



  __forceinline float16 initTriangle1(const float16 &v0,
				    const float16 &v1,
				    const float16 &v2,
				    const int16 &geomID,
				    const int16 &primID,
				    const int16 &mask)
  {
    const float16 e1 = v0 - v1;
    const float16 e2 = v2 - v0;	     
    const float16 normal = lcross_xyz(e1,e2);
    const float16 _v0 = select(0x8888,cast((__m512i)primID),v0);
    const float16 _v1 = select(0x8888,cast((__m512i)geomID),v1);
    const float16 _v2 = select(0x8888,cast((__m512i)mask),v2);
    const float16 _v3 = select(0x8888,float16::zero(),normal);
    const float16 final = lane_shuffle_gather<0>(_v0,_v1,_v2,_v3);
    return final;
  }


  __forceinline void createBVH4iLeaf(BVH4i::NodeRef &ref,
				     const unsigned int offset,
				     const unsigned int entries) 
  {
    assert(entries <= 4);
    ref = (offset << BVH4i::encodingBits) | BVH4i::leaf_mask | (entries-1);
  }

  template<unsigned int scale>
  __forceinline void createBVH4iNode(BVH4i::NodeRef &ref,
				     const unsigned int index) {
    ref = ((index*scale) << BVH4i::encodingBits);
  }



}
