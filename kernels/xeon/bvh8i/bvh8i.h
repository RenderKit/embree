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

#include "bvh4i/bvh4i.h"
#include "common/alloc.h"
#include "common/accel.h"
#include "geometry/primitive.h"
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
    /*! branching width of the tree */
    static const size_t N = 8;

    /*! BVH8i instantiations */
    static Accel* BVH8iTriangle8(Scene* scene);

#if defined (__AVX__)

    /*! BVH8 Node */
    struct __align(64) Node
    {
      avxf lower_x;
      avxf upper_x;
      avxf lower_y;
      avxf upper_y;
      avxf lower_z;
      avxf upper_z;
      BVH4i::NodeRef children[8];
      unsigned int data[8]; 

      __forceinline void set(const size_t index,const BVH4i::Node &node4, const size_t i)
      {
	lower_x[index] = node4.lower_x[i];
	lower_y[index] = node4.lower_y[i];
	lower_z[index] = node4.lower_z[i];

	upper_x[index] = node4.upper_x[i];
	upper_y[index] = node4.upper_y[i];
	upper_z[index] = node4.upper_z[i];

	children[index] = node4.children[i];
	data[index]     = node4.data[i];
      }

      // -------------------------
      __forceinline void set(const size_t index,const BBox3fa &node)
      {
	lower_x[index] = node.lower[0];
	lower_y[index] = node.lower[1];
	lower_z[index] = node.lower[2];

	upper_x[index] = node.upper[0];
	upper_y[index] = node.upper[1];
	upper_z[index] = node.upper[2];

	children[index] = node.lower.a;
	data[index] = node.upper.a;
      }

      __forceinline void set(const size_t index,const Node &node, const size_t source_index)
      {
	assert(index < N);
	assert(source_index < N);

	lower_x[index] = node.lower_x[source_index];
	lower_y[index] = node.lower_y[source_index];
	lower_z[index] = node.lower_z[source_index];

	upper_x[index] = node.upper_x[source_index];
	upper_y[index] = node.upper_y[source_index];
	upper_z[index] = node.upper_z[source_index];

	children[index] = node.children[source_index];
	data[index] = node.data[source_index];
      }

      __forceinline BBox3fa extract(const size_t index)
      {
	assert(index < N);

	BBox3fa node;
	node.lower[0] = lower_x[index];
	node.lower[1] = lower_y[index];
	node.lower[2] = lower_z[index];
	node.lower.a  = children[index];

	node.upper[0] = upper_x[index];
	node.upper[1] = upper_y[index];
	node.upper[2] = upper_z[index];
	node.upper.a  = data[index];
	return node;
      }

      __forceinline avxf area()
      {
	const avxf x = upper_x - lower_x;
	const avxf y = upper_y - lower_y;
	const avxf z = upper_z - lower_z;
	return (x*y+x*z+y*z) * 2.0f; 
      }

      __forceinline void shift(const size_t index)
      {
	assert(index < N);

	for (size_t i=index+1;i<N;i++)
	  {
	    lower_x[i-1] = lower_x[i];
	    lower_y[i-1] = lower_y[i];
	    lower_z[i-1] = lower_z[i];

	    upper_x[i-1] = upper_x[i];
	    upper_y[i-1] = upper_y[i];
	    upper_z[i-1] = upper_z[i];

	    children[i-1] = children[i];
	    data[i-1] = data[i];	
	  }
      }

      __forceinline void reset()
      {
    
	lower_x = pos_inf;
	lower_y = pos_inf;
	lower_z = pos_inf;

	upper_x = neg_inf;
	upper_y = neg_inf;
	upper_z = neg_inf;

	for (size_t i=0;i<N;i++) children[i] = emptyNode;
	for (size_t i=0;i<N;i++) data[i] = 0;

      }

      __forceinline BBox3fa bounds() const {
        const Vec3fa lower(reduce_min(lower_x),reduce_min(lower_y),reduce_min(lower_z));
        const Vec3fa upper(reduce_max(upper_x),reduce_max(upper_y),reduce_max(upper_z));
        return BBox3fa(lower,upper);
      }

      /*! Returns bounds of specified child. */
      __forceinline BBox3fa bounds(size_t i) const {
        Vec3fa lower(lower_x[i],lower_y[i],lower_z[i]);
        Vec3fa upper(upper_x[i],upper_y[i],upper_z[i]);
        return BBox3fa(lower,upper);
      }

      /*! Returns number of valid children */
      __forceinline size_t numValidChildren() const  {
	size_t valid = 0;
	for (size_t i=0;i<N;i++)
	  if (children[i] != emptyNode)
	    valid++;
	return valid;
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { return children[i]; }


    };

    static __forceinline Node *bvh8ChildPtrNoMask(const Node * __restrict__ const ptr, const unsigned int node) {
      return (Node*)((char*)ptr + (unsigned long)node);
    };

    float sah8 ();
    float sah8 (Node*base, BVH4i::NodeRef& node, const BBox3fa& bounds);

    struct __align(8) CompressedNode
    {
      float min_x;
      float max_x;
      float min_y;
      float max_y;
      float min_z;
      float max_z;
      float dummy[2];

      unsigned char lower_x[8];
      unsigned char upper_x[8];
      unsigned char lower_y[8];
      unsigned char upper_y[8];
      unsigned char lower_z[8];
      unsigned char upper_z[8];

      BVH4i::NodeRef children[8];            

      __forceinline CompressedNode() {}

      __forceinline float clamp255(float i)
      {
        return min(max(i,0.0f),255.0f);
      }

#define ULPS 1

      __forceinline float roundUp(float v)
      {
	const float s_up   = 1.0f + ULPS * (float)ulp;
	const float s_down = 1.0f - ULPS * (float)ulp;
	const float new_v = v < 0.0f ? v*s_down : v*s_up;
	return new_v;
      }

      __forceinline float roundDown(float v)
      {
	const float s_up   = 1.0f + ULPS * (float)ulp;
	const float s_down = 1.0f - ULPS * (float)ulp;
	const float new_v = v >= 0.0f ? v*s_down : v*s_up;
	return new_v;
      }


      CompressedNode( const BVH8i::Node &node8 )
        {
          min_x = reduce_min(node8.lower_x);
          max_x = reduce_max(node8.upper_x);
          min_y = reduce_min(node8.lower_y);
          max_y = reduce_max(node8.upper_y);
          min_z = reduce_min(node8.lower_z);
          max_z = reduce_max(node8.upper_z);

          for (size_t i=0;i<8;i++) children[i] = node8.children[i];		

          const float diff_x = 1.0f / (max_x - min_x); 
          const float diff_y = 1.0f / (max_y - min_y);
          const float diff_z = 1.0f / (max_z - min_z);

          for (size_t i=0;i<8;i++)
	    {
	      lower_x[i] = 0;
	      upper_x[i] = 0;
	      lower_y[i] = 0;
	      upper_y[i] = 0;
	      lower_z[i] = 0;
	      upper_z[i] = 0;
	    }
	  
          for (size_t i=0;i<node8.numValidChildren();i++)
            {
              lower_x[i] = (unsigned int)clamp255(floorf(255.0f * roundDown(((node8.lower_x[i] - min_x) * diff_x))));
              upper_x[i] = (unsigned int)clamp255(ceilf(255.0f * roundUp( ((node8.upper_x[i] - min_x) * diff_x)) ));

              float t0_x = ((float)floorf(lower_x[i]) * 1.0f/255.0f);
              float t1_x = ((float)ceilf(upper_x[i]) * 1.0f/255.0f);

              float decompress_min_x = floorf(t0_x * max_x + (1.0f - t0_x) * min_x);
              float decompress_max_x = ceilf(t1_x * max_x + (1.0f - t1_x) * min_x);


#if 0
              std::cout << std::endl;

              DBG_PRINT( min_x );
              DBG_PRINT( max_x );
              DBG_PRINT( (unsigned int)lower_x[i] );
              DBG_PRINT( (unsigned int)upper_x[i] );

              DBG_PRINT(t0_x);
              DBG_PRINT(t1_x);

              DBG_PRINT( decompress_min_x );
              DBG_PRINT( decompress_max_x );

              DBG_PRINT( node8.lower_x[i] );
              DBG_PRINT( node8.upper_x[i] );
#endif

              assert( decompress_min_x <= node8.lower_x[i] );
              assert( decompress_max_x >= node8.upper_x[i] );

              lower_y[i] = (unsigned int)clamp255(floorf(255.0f * roundDown(((node8.lower_y[i] - min_y) * diff_y))));
              upper_y[i] = (unsigned int)clamp255(ceilf(255.0f * roundUp(((node8.upper_y[i] - min_y) * diff_y))));

              float t0_y = ((float)(lower_y[i]) * 1.0f/255.0f);
              float t1_y = ((float)(upper_y[i]) * 1.0f/255.0f);

              float decompress_min_y = floorf(t0_y * max_y + (1.0f - t0_y) * min_y);
              float decompress_max_y = ceilf(t1_y * max_y + (1.0f - t1_y) * min_y);

              lower_z[i] = (unsigned int)clamp255(floorf(255.0f * roundDown(((node8.lower_z[i] - min_z) * diff_z))));
              upper_z[i] = (unsigned int)clamp255(ceilf(255.0f * roundUp(((node8.upper_z[i] - min_z) * diff_z))));

              float t0_z = ((float)(lower_z[i]) * 1.0f/255.0f);
              float t1_z = ((float)(upper_z[i]) * 1.0f/255.0f);

              float decompress_min_z = floorf(t0_z * max_z + (1.0f - t0_z) * min_z);
              float decompress_max_z = ceilf(t1_z * max_z + (1.0f - t1_z) * min_z);


              assert( decompress_min_y <= node8.lower_y[i] );
              assert( decompress_max_y >= node8.upper_y[i] );

              assert( decompress_min_z <= node8.lower_z[i] );
              assert( decompress_max_z >= node8.upper_z[i] );

            }

          
        }
    };

    

#endif

  public:

    /*! BVH4 default constructor. */
    BVH8i (const PrimitiveType& primTy, void* geometry = NULL) : BVH4i(primTy,geometry) {}

    
  };


#if defined (__AVX__)

    __forceinline std::ostream &operator<<(std::ostream &o, const BVH8i::Node &v)
    {
      o << "lower_x " << v.lower_x << std::endl;
      o << "upper_x " << v.upper_x << std::endl;

      o << "lower_y " << v.lower_y << std::endl;
      o << "upper_y " << v.upper_y << std::endl;

      o << "lower_z " << v.lower_z << std::endl;
      o << "upper_z " << v.upper_z << std::endl;

      o << "children " << *(avxi*)v.children << std::endl;
      o << "data     " << *(avxi*)v.data << std::endl;

      return o;
    }
#endif

};
