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

#ifndef __EMBREE_BVH8I_H__
#define __EMBREE_BVH8I_H__

#include "common/alloc.h"
#include "common/primitive.h"
#include "common/accel.h"
#include "bvh4i/bvh4i.h"
#include "geometry/triangle4.h"

namespace embree
{

#define BVH8_MAX_STACK_DEPTH 128

  /* ------------ */
  /* --- BVH8 --- */
  /* ------------ */

#define BVH8_INDEX_SHIFT 7
#define BVH8_LEAF_BIT_SHIFT 5
#define BVH8_LEAF_MASK     ((unsigned int)1 << BVH8_LEAF_BIT_SHIFT)
#define BVH8_ITEMS_MASK   (BVH8_LEAF_MASK-1)
#define BVH8_OFFSET_MASK  (~(BVH8_ITEMS_MASK | BVH8_LEAF_MASK))
#define BVH8_TERMINAL_TOKEN BVH8_LEAF_MASK

  /*! Multi BVH with 8 children. Each node stores the bounding box of
   * it's 8 children as well as a 8 child indices. */
  class BVH8i : public BVH4i
  {
  public:

    /*! BVH8i instantiations */
    static Accel* BVH8iTriangle1(Scene* scene);

#if defined (__AVX__)

    /*! BVH8 Node */
    struct __align(64) BVH8iNode
    {
      avxf min_x;
      avxf max_x;
      avxf min_y;
      avxf max_y;
      avxf min_z;
      avxf max_z;
      avxi min_d;
      avxi max_d;

      __forceinline void set(const size_t index,const BBox3f &node)
      {
	min_x[index] = node.lower[0];
	min_y[index] = node.lower[1];
	min_z[index] = node.lower[2];

	max_x[index] = node.upper[0];
	max_y[index] = node.upper[1];
	max_z[index] = node.upper[2];

	min_d[index] = node.lower.a;
	max_d[index] = node.upper.a;
      }

      __forceinline void set(const size_t index,const BVH8iNode &node, const size_t source_index)
      {
	assert(index < 8);
	assert(source_index < 8);

	min_x[index] = node.min_x[source_index];
	min_y[index] = node.min_y[source_index];
	min_z[index] = node.min_z[source_index];

	max_x[index] = node.max_x[source_index];
	max_y[index] = node.max_y[source_index];
	max_z[index] = node.max_z[source_index];

	min_d[index] = node.min_d[source_index];
	max_d[index] = node.max_d[source_index];
      }

      __forceinline BBox3f extract(const size_t index)
      {
	assert(index < 8);

	BBox3f node;
	node.lower[0] = min_x[index];
	node.lower[1] = min_y[index];
	node.lower[2] = min_z[index];
	node.lower.a  = min_d[index];

	node.upper[0] = max_x[index];
	node.upper[1] = max_y[index];
	node.upper[2] = max_z[index];
	node.upper.a  = max_d[index];
	return node;
      }

      __forceinline avxf area()
      {
	const avxf x = max_x - min_x;
	const avxf y = max_y - min_y;
	const avxf z = max_z - min_z;
	return (x*y+x*z+y*z) * 2.0f; 
      }

      __forceinline void shift(const size_t index)
      {
	assert(index < 8);

	for (size_t i=index+1;i<8;i++)
	  {
	    min_x[i-1] = min_x[i];
	    min_y[i-1] = min_y[i];
	    min_z[i-1] = min_z[i];

	    max_x[i-1] = max_x[i];
	    max_y[i-1] = max_y[i];
	    max_z[i-1] = max_z[i];

	    min_d[i-1] = min_d[i];
	    max_d[i-1] = max_d[i];	
	  }
      }

      __forceinline void reset()
      {
    
	min_x = avxf(1E14);
	min_y = avxf(1E14);
	min_z = avxf(1E14);

	max_x = avxf(1E14);
	max_y = avxf(1E14);
	max_z = avxf(1E14);

	min_d = avxi(BVH8_LEAF_MASK);
	max_d = avxi(0);
      }

      __forceinline void reset(const unsigned int a,
			       const unsigned int b = 0)
      {
    
	min_x = avxf(1E14);
	min_y = avxf(1E14);
	min_z = avxf(1E14);

	max_x = avxf(1E14);
	max_y = avxf(1E14);
	max_z = avxf(1E14);

	min_d = avxi(a);
	max_d = avxi(b);
      }

    };

    static __forceinline BVH8iNode *bvh8ChildPtrNoMask(const BVH8iNode * __restrict__ const ptr, const unsigned int node) {
      return (BVH8iNode*)((char*)ptr + (unsigned long)node);
    };

#endif

  public:
    
    /*! BVH4 default constructor. */
    BVH8i (const PrimitiveType& primTy, void* geometry = NULL)
      : BVH4i(primTy,geometry) {}
  };


#if defined (__AVX__)

    __forceinline std::ostream &operator<<(std::ostream &o, const BVH8i::BVH8iNode &v)
    {
      o << "min_x " << v.min_x << std::endl;
      o << "max_x " << v.max_x << std::endl;

      o << "min_y " << v.min_y << std::endl;
      o << "max_y " << v.max_y << std::endl;

      o << "min_z " << v.min_z << std::endl;
      o << "max_z " << v.max_z << std::endl;

      o << "min_d " << v.min_d << std::endl;
      o << "max_d " << v.max_d << std::endl;

      return o;
    }
#endif

};

#endif
