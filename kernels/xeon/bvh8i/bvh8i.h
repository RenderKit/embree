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


#define USE_QUANTIZED_NODES

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
    struct __aligned(64) Node
    {
      avxf lower_x;
      avxf upper_x;
      avxf lower_y;
      avxf upper_y;
      avxf lower_z;
      avxf upper_z;
      BVH4i::NodeRef children[8];
      unsigned int data[8]; 

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
        size_t valid =__popcnt(movemask((*(avxi*)children) != avxi(emptyNode)));
        assert(valid <= 8);
	return valid;
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { return children[i]; }


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

      __forceinline void setBounds(const size_t index,const BBox3fa &node)
      {
	lower_x[index] = node.lower[0];
	lower_y[index] = node.lower[1];
	lower_z[index] = node.lower[2];

	upper_x[index] = node.upper[0];
	upper_y[index] = node.upper[1];
	upper_z[index] = node.upper[2];

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

      __forceinline void merge(const Node &source)
      {
        const size_t numValidDest   = numValidChildren();
        const size_t numValidSource = source.numValidChildren();
        assert(numValidDest + numValidSource <= 8);
        for (size_t i=0;i<numValidSource;i++)
          set(numValidDest+i,source,i);
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

      __forceinline float area(const size_t index)
      {
        avxf area8 = area();
	return area8[index];
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

      __forceinline void setInvalid(const size_t i)
      {
    
	lower_x[i] = pos_inf;
	lower_y[i] = pos_inf;
	lower_z[i] = pos_inf;

	upper_x[i] = neg_inf;
	upper_y[i] = neg_inf;
	upper_z[i] = neg_inf;

	children[i] = emptyNode;
	data[i] = 0;
      }



    };

    static __forceinline Node *bvh8ChildPtrNoMask(const Node * __restrict__ const ptr, const unsigned int node) {
      return (Node*)((char*)ptr + (unsigned long)node);
    };


    static float sah8 (Node* base, BVH4i::NodeRef& root, avxi &bvh8i_node_dist);
    static float sah8 (Node* base, BVH4i::NodeRef& node, const BBox3fa& bounds, avxi &bvh8i_node_dist);

    struct __aligned(64) Quantized8BitNode
    {
      unsigned char lower_x[8];
      unsigned char upper_x[8];
      unsigned char lower_y[8];
      unsigned char upper_y[8];
      unsigned char lower_z[8];
      unsigned char upper_z[8];

      float min_x;
      float max_x;
      float min_y;
      float max_y;
      float min_z;
      float max_z;

      float diff_x;
      float diff_y;
      float diff_z;

      float dummy[3];


      BVH4i::NodeRef children[8];            

      __forceinline Quantized8BitNode() {}

#define ULPS 1

      __forceinline float clamp(float v)
      {
        return min(max(v,0.0f),255.0f);     
      }

      __forceinline float roundUp(float v)
      {
	/* const float s_up   = 1.0f + ULPS * (float)ulp; */
	/* const float s_down = 1.0f - ULPS * (float)ulp; */
	/* const float new_v = v < 0.0f ? v*s_down : v*s_up; */
	/* return new_v; */
        return v;
      }

      __forceinline float roundDown(float v)
      {
	/* const float s_up   = 1.0f + ULPS * (float)ulp; */
	/* const float s_down = 1.0f - ULPS * (float)ulp; */
	/* const float new_v = v >= 0.0f ? v*s_down : v*s_up; */
	/* return new_v; */
        return v;
      }

      __forceinline float lowerX(const size_t i) const
      {
        return min_x + ((float)lower_x[i] * diff_x);
      }

      __forceinline float upperX(const size_t i) const
      {
        return min_x + ((float)upper_x[i] * diff_x);
      }

      __forceinline float lowerY(const size_t i) const
      {
        return min_y + ((float)lower_y[i] * diff_y);
      }

      __forceinline float upperY(const size_t i) const
      {
        return min_y + ((float)upper_y[i] * diff_y);        
      }

      __forceinline float lowerZ(const size_t i) const
      {
        return min_z + ((float)lower_z[i] * diff_z);
      }

      __forceinline float upperZ(const size_t i) const
      {
        return min_z + ((float)upper_z[i] * diff_z);        
      }


      void init( const BVH8i::Node &node8 )
      {
        min_x = reduce_min(node8.lower_x);
        max_x = reduce_max(node8.upper_x);
        min_y = reduce_min(node8.lower_y);
        max_y = reduce_max(node8.upper_y);
        min_z = reduce_min(node8.lower_z);
        max_z = reduce_max(node8.upper_z);

        for (size_t i=0;i<8;i++) 
          {
            if (node8.children[i].isNode())
              children[i] = sizeof(BVH8i::Quantized8BitNode) * ((unsigned int)node8.children[i] / sizeof(BVH8i::Node));
            else
              {
                children[i] = node8.children[i];
              }		
          }

        diff_x = max_x - min_x;
        diff_y = max_y - min_y;
        diff_z = max_z - min_z;

        const float rcp_diff_x = 255.0f / diff_x; 
        const float rcp_diff_y = 255.0f / diff_y;
        const float rcp_diff_z = 255.0f / diff_z;

        diff_x *= 1.0f / 255.0f;
        diff_y *= 1.0f / 255.0f;
        diff_z *= 1.0f / 255.0f;

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
            lower_x[i] = (unsigned int)clamp(floorf(roundDown(((node8.lower_x[i] - min_x) * rcp_diff_x))));
            upper_x[i] = (unsigned int)clamp(ceilf(roundUp( ((node8.upper_x[i] - min_x) * rcp_diff_x))));

            float decompress_min_x = lowerX(i); 
            float decompress_max_x = upperX(i); 

#if 0
            DBG_PRINT(decompress_min_x);
            DBG_PRINT(decompress_max_x);
            DBG_PRINT(node8.lower_x[i]);
            DBG_PRINT(node8.upper_x[i]);

            assert( decompress_min_x <= node8.lower_x[i] );
            assert( decompress_max_x >= node8.upper_x[i] );
#endif
            lower_y[i] = (unsigned int)clamp(floorf(roundDown(((node8.lower_y[i] - min_y) * rcp_diff_y))));
            upper_y[i] = (unsigned int)clamp(ceilf(roundUp(((node8.upper_y[i] - min_y) * rcp_diff_y))));

            float decompress_min_y = lowerY(i);
            float decompress_max_y = upperY(i);
#if 0
            assert( decompress_min_y <= node8.lower_y[i] );
            assert( decompress_max_y >= node8.upper_y[i] );
#endif
            lower_z[i] = (unsigned int)clamp(floorf(roundDown(((node8.lower_z[i] - min_z) * rcp_diff_z))));
            upper_z[i] = (unsigned int)clamp(ceilf(roundUp(((node8.upper_z[i] - min_z) * rcp_diff_z))));

            float decompress_min_z = lowerZ(i);
            float decompress_max_z = upperZ(i);
#if 0
            assert( decompress_min_z <= node8.lower_z[i] );
            assert( decompress_max_z >= node8.upper_z[i] );
#endif
          }
          
        assert(node8.numValidChildren() == numValidChildren());
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { return children[i]; }

      __forceinline BBox3fa bounds(size_t i) const {
        Vec3fa lower(lowerX(i),lowerY(i),lowerZ(i));
        Vec3fa upper(upperX(i),upperY(i),upperZ(i));
        return BBox3fa(lower,upper);
      }

      __forceinline BBox3fa bounds() const {
        BBox3fa b( empty );
        for (size_t i=0;i<8;i++)
          {
            if (children[i] == emptyNode) break;           
            Vec3fa l(lowerX(i),lowerY(i),lowerZ(i));
            Vec3fa u(upperX(i),upperY(i),upperZ(i));
            b.extend( BBox3fa(l,u) );
          }
        return b;
      }

      __forceinline size_t numValidChildren() const  {
        size_t valid =__popcnt(movemask((*(avxi*)children) != avxi(emptyNode)));
        assert(valid <= 8);
	return valid;
      }
      
    };

    
    static float sah8_quantized (Quantized8BitNode*base, BVH4i::NodeRef& node, const BBox3fa& bounds, avxi &bvh8i_node_dist);
    static float sah8_quantized (Quantized8BitNode* base, BVH4i::NodeRef& root, avxi &bvh8i_node_dist);





#if defined (__AVX2__)

    struct __aligned(64) NodeHF16
    {
      ssei lower_x;
      ssei upper_x;
      ssei lower_y;
      ssei upper_y;
      ssei lower_z;
      ssei upper_z;

      BVH4i::NodeRef children[8];            

      __forceinline avxf lowerX() const { return convert_from_hf16(lower_x); }
      __forceinline avxf upperX() const { return convert_from_hf16(upper_x); }

      __forceinline avxf lowerY() const { return convert_from_hf16(lower_y); }
      __forceinline avxf upperY() const { return convert_from_hf16(upper_y); }

      __forceinline avxf lowerZ() const { return convert_from_hf16(lower_z); }
      __forceinline avxf upperZ() const { return convert_from_hf16(upper_z); }


      void init( const BBox3fa& root, const BVH8i::Node &node8 )
      {

        for (size_t i=0;i<8;i++) 
          {
            if (node8.children[i].isNode())
              children[i] = sizeof(BVH8i::NodeHF16) * ((unsigned int)node8.children[i] / sizeof(BVH8i::Node));
            else
              children[i] = node8.children[i];		
          }

        avx3f root_lower(root.lower.x,root.lower.y,root.lower.z);
        avx3f root_upper(root.upper.x,root.upper.y,root.upper.z);
        avx3f root_length = root_upper - root_lower;
        avx3f root_inv_length = avx3f( one ) / root_length;

        const avxf lower_x_t = (node8.lower_x - root_lower.x) * root_inv_length.x;
        const avxf upper_x_t = (node8.upper_x - root_lower.x) * root_inv_length.x;

        const avxf lower_y_t = (node8.lower_y - root_lower.y) * root_inv_length.y;
        const avxf upper_y_t = (node8.upper_y - root_lower.y) * root_inv_length.y;

        const avxf lower_z_t = (node8.lower_z - root_lower.z) * root_inv_length.z;
        const avxf upper_z_t = (node8.upper_z - root_lower.z) * root_inv_length.z;

        lower_x = convert_to_hf16<_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC>(lower_x_t);
        upper_x = convert_to_hf16<_MM_FROUND_TO_POS_INF |_MM_FROUND_NO_EXC>(upper_x_t);

        lower_y = convert_to_hf16<_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC>(lower_y_t);
        upper_y = convert_to_hf16<_MM_FROUND_TO_POS_INF |_MM_FROUND_NO_EXC>(upper_y_t);

        lower_z = convert_to_hf16<_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC>(lower_z_t);
        upper_z = convert_to_hf16<_MM_FROUND_TO_POS_INF |_MM_FROUND_NO_EXC>(upper_z_t);

#if 0
        for (size_t i=node8.numValidChildren();i<8;i++)
          {
            ((short*)&lower_x)[i] = 0;
            ((short*)&upper_x)[i] = 0;
            ((short*)&lower_y)[i] = 0;
            ((short*)&upper_y)[i] = 0;
            ((short*)&lower_z)[i] = 0;
            ((short*)&upper_z)[i] = 0;            
          }
#endif
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { return children[i]; }

      __forceinline BBox3fa bounds(size_t i) const {
        Vec3fa lower(lowerX()[i],lowerY()[i],lowerZ()[i]);
        Vec3fa upper(upperX()[i],upperY()[i],upperZ()[i]);
        return BBox3fa(lower,upper);
      }


      __forceinline BBox3fa bounds() const {
        const Vec3fa lower(reduce_min(lowerX()),reduce_min(lowerY()),reduce_min(lowerZ()));
        const Vec3fa upper(reduce_max(upperX()),reduce_max(upperY()),reduce_max(upperZ()));
        return BBox3fa(lower,upper);
      }

      __forceinline size_t numValidChildren() const  {
	size_t valid = 0;
	for (size_t i=0;i<N;i++)
	  if (children[i] != emptyNode)
	    valid++;
	return valid;
      }

    };
#endif

#endif

  public:

    /*! BVH4 default constructor. */
    BVH8i (const PrimitiveType& primTy, void* geometry = NULL) : BVH4i(primTy,geometry) {}

    
  };


#if defined (__AVX2__)

    __forceinline std::ostream &operator<<(std::ostream &o, const BVH8i::NodeHF16 &v)
    {
      o << "lower_x " << v.lowerX() << std::endl;
      o << "upper_x " << v.upperX() << std::endl;

      o << "lower_y " << v.lowerY() << std::endl;
      o << "upper_y " << v.upperY() << std::endl;

      o << "lower_z " << v.lowerZ() << std::endl;
      o << "upper_z " << v.upperZ() << std::endl;

      o << "children " << *(avxi*)v.children << std::endl;

      return o;
    }
#endif


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
      o << "empytNode " << (avxi(BVH8i::emptyNode) == *(avxi*)v.children) << std::endl;

      return o;
    }


    __forceinline std::ostream &operator<<(std::ostream &o, const BVH8i::Quantized8BitNode &v)
    {
      o << "min " << v.min_x << " " << v.min_y << " " << v.min_z << std::endl;
      o << "max " << v.max_x << " " << v.max_y << " " << v.max_z << std::endl;
      o << "lower_x ";
      for (size_t i=0;i<8;i++) o << v.lowerX(i) << " ";
      o << std::endl;

      o << "upper_x ";
      for (size_t i=0;i<8;i++) o << v.upperX(i) << " ";
      o << std::endl;

      o << "lower_y ";
      for (size_t i=0;i<8;i++) o << v.lowerY(i) << " ";
      o << std::endl;

      o << "upper_y ";
      for (size_t i=0;i<8;i++) o << v.upperY(i) << " ";
      o << std::endl;

      o << "lower_z ";
      for (size_t i=0;i<8;i++) o << v.lowerZ(i) << " ";
      o << std::endl;

      o << "upper_z ";
      for (size_t i=0;i<8;i++) o << v.upperZ(i) << " ";
      o << std::endl;

      o << "children  " << *(avxi*)v.children << std::endl;

      return o;
    }

#endif

};
