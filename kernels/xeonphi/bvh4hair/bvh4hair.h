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

#include "common/alloc.h"
#include "common/accel.h"
#include "common/scene.h"
#include "geometry/primitive.h"
#include "bvh4i/bvh4i.h"

namespace embree
{


  class BVH4Hair : public BVH4i
  {
  public:
    /*! branching width of the tree */
    static const size_t N = 4;

    /*! Masks the bits that store the number of items per leaf. */
    static const size_t encodingBits = 4;
    static const size_t offset_mask  = ((size_t)-1) << encodingBits;
    static const size_t leaf_shift   = 3;
    static const size_t leaf_mask    = 1<<leaf_shift;  
    static const size_t items_mask   = leaf_mask-1;  
    
    /*! Empty node */
    static const size_t emptyNode = leaf_mask;

    /*! Invalid node */
    static const size_t invalidNode = (size_t)-1;

    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 32;


    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (size_t id) : _id(id) { }

      /*! Cast to size_t*/
      __forceinline operator size_t() const { return _id; }
     
      /*! checks if this is a leaf */
      __forceinline size_t isLeaf() const { return _id & leaf_mask; }

      /*! checks if this is a leaf */
      __forceinline size_t isLeaf(const size_t mask) const { return _id & mask; }
      
      /*! checks if this is a node */
      __forceinline size_t isNode() const { return (_id & leaf_mask) == 0; }
      
      /*! returns node pointer */
      __forceinline       Node* node()       { assert(isNode()); return (Node*)((size_t)_id & offset_mask); }

      
      /*! returns leaf pointer */
      __forceinline char* leaf(size_t& num) const {
        assert(isLeaf());
        num = (_id & (size_t)items_mask);
        return (char*)(_id & offset_mask);
      }


      __forceinline size_t offset() const {
        return _id & offset_mask;
      }

      __forceinline size_t items() const {
        return _id & items_mask;
      }
      
    private:
      size_t _id;
    };

    struct __aligned(256) UnalignedNode
    {
      mic_f matrixColumnXYZW[3];
      NodeRef child[4];
      unsigned int geomID[4];
      unsigned int primID[4];

      static float identityMatrix[16];

      __forceinline void setIdentityMatrix() 
      {
	const mic_f c0 = broadcast4to16f(&identityMatrix[ 0]);
	const mic_f c1 = broadcast4to16f(&identityMatrix[ 4]);
	const mic_f c2 = broadcast4to16f(&identityMatrix[ 8]);
	
	matrixColumnXYZW[0] = c0;
	matrixColumnXYZW[1] = c1;
	matrixColumnXYZW[2] = c2;
      }

      void convertFromBVH4iNode(const BVH4i::Node &node);

      
    };

  BVH4Hair(const PrimitiveType& primTy, void* geometry = NULL) : BVH4i(primTy,geometry)
      {	
	DBG_PRINT(sizeof(UnalignedNode));
	assert( sizeof(UnalignedNode) == 256 );
	unaligned_nodes = NULL;
      }
    
    static Accel* BVH4HairBinnedSAH(Scene* scene);
    

    UnalignedNode *unaligned_nodes;

  };
};
