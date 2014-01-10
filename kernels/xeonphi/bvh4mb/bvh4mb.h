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

#ifndef __EMBREE_BVH4MB_MIC_H__
#define __EMBREE_BVH4MB_MIC_H__

#include "common/alloc.h"
#include "common/accel.h"
#include "common/scene.h"
#include "geometry/primitive.h"

namespace embree
{
  /*! Multi BVH with 4 children. Each node stores the bounding box of
   * it's 4 children as well as a 4 child indices. */
  class BVH4mb : public Bounded
  {
  public:

    /*! forward declaration of node type */
    struct Node;

    /*! branching width of the tree */
    static const size_t N = 4;

    /*! Masks the bits that store the number of items per leaf. */
    static const unsigned int offset_mask = 0xFFFFFFFF << 6;
    static const unsigned int barrier_mask = 1<<31;
    static const unsigned int leaf_mask = 1<<5;  
    static const unsigned int items_mask = leaf_mask-1;  
    
    /*! Empty node */
    static const unsigned int emptyNode = leaf_mask;

    /*! Invalid node */
    //static const unsigned invalidNode = leaf_mask;
    static const unsigned int invalidNode = 0xFFFFFFE0;

    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 26;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+6;
    static const size_t maxDepth = maxBuildDepth + maxBuildDepthLeaf;
    
    /*! Cost of one traversal step. */
    static const int travCost = 1;

    static const size_t hybridSIMDUtilSwitchThreshold = 8;

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
      __forceinline unsigned isNode() const { return (_id & leaf_mask) == 0; }
      
      /*! returns node pointer */
      __forceinline       Node* node(      void* base) const { return (      Node*)((      char*)base + _id); }
      __forceinline const Node* node(const void* base) const { return (const Node*)((const char*)base + _id); }
      
      /*! returns leaf pointer */
      __forceinline const char* leaf(const void* base, unsigned int& num) const {
        assert(isLeaf());
        num = _id & items_mask;
        return (const char*)base + (_id & offset_mask);
      }

      /*! returns leaf pointer */
      __forceinline const char* leaf(const void* base) const {
        assert(isLeaf());
        return (const char*)base + (_id & offset_mask);
      }

      __forceinline unsigned int offset() const {
        return _id & offset_mask;
      }

      __forceinline unsigned int items() const {
        return _id & items_mask;
      }
      
      __forceinline unsigned int &id() { return _id; }
    private:
      unsigned int _id;
    };

    /*! BVH4mb Node */

    struct Node
    {
    public:
      struct NodeStruct {
        float x,y,z;           // x,y, and z coordinates of bounds
        NodeRef child;         // encodes 1 is-leaf bit, 25 offset bits, and 6 num-items bits
      } lower[4], upper[4];    // lower and upper bounds of all 4 children

      struct Delta {
        float x,y,z,w;         
      } delta_lower[4], delta_upper[4];    // lower and upper bounds of all 4 children

      /*! Returns bounds of specified child. */
      __forceinline BBox3f bounds(size_t i) const {
        Vec3fa l = *(Vec3fa*)&lower[i];
        Vec3fa u = *(Vec3fa*)&upper[i];
        return BBox3f(l,u);
      }

      __forceinline void setInvalid(size_t i)
      {
	lower[i].x = pos_inf;
	lower[i].y = pos_inf;
	lower[i].z = pos_inf;
	lower[i].child = NodeRef(leaf_mask);

	upper[i].x = neg_inf;
	upper[i].y = neg_inf;
	upper[i].z = neg_inf;
	upper[i].child = NodeRef(0);
      }
      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return lower[i].child; }
      __forceinline const NodeRef& child(size_t i) const { return lower[i].child; }


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



  public:

    /*! BVH4 default constructor. */
    BVH4mb (const PrimitiveType& primTy, void* geometry = NULL)
      : primTy(primTy), 
      geometry(geometry), 
      root(emptyNode), 
      qbvh(NULL), 
      accel(NULL),
      size_node(0),
      size_accel(0)
    {
    }

    ~BVH4mb();

    /*! BVH4i instantiations */
    static Accel* BVH4mbTriangle1ObjectSplitBinnedSAH(Scene* scene);

    /*! Calculates the SAH of the BVH */
    float sah ();

    /*! Data of the BVH */
  public:
    //const size_t maxLeafPrims;          //!< maximal number of triangles per leaf
    NodeRef root;                      //!< Root node (can also be a leaf).

    const PrimitiveType& primTy;   //!< triangle type stored in BVH
    void* geometry;                    //!< pointer to geometry for intersection

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


    struct Helper { float x,y,z; int a; }; 

    static Helper initQBVHNode[4];

  private:
    float sah (NodeRef& node, BBox3f bounds);
  };


  __forceinline std::ostream &operator<<(std::ostream &o, const BVH4mb::Node &v)
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

}

#endif
