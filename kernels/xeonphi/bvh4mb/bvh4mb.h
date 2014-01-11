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

#include "bvh4i/bvh4i.h"

namespace embree
{
  /*! Multi BVH with 4 children. Each node stores the bounding box of
   * it's 4 children as well as a 4 child indices. */
  class BVH4mb : public BVH4i
  {
  public:

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

    void *accel_t1;

    /*! BVH4 default constructor. */
    BVH4mb (const PrimitiveType& primTy, void* geometry = NULL) : BVH4i(primTy,geometry)
    {
      accel_t1 = NULL;
    }

    ~BVH4mb();


    static Accel* BVH4mbTriangle1ObjectSplitBinnedSAH(Scene* scene);

  };

  __forceinline std::ostream &operator<<(std::ostream &o, const BVH4mb::Node &v)
    {
      o << std::endl;
      o << "lower: ";
      for (size_t i=0;i<4;i++) o << "[" << v.lower[i].x << "," << v.lower[i].y << "," << v.lower[i].z << "," << v.lower[i].child <<"] ";
      o << std::endl;
      o << "upper: ";
      for (size_t i=0;i<4;i++) o << "[" << v.upper[i].x << "," << v.upper[i].y << "," << v.upper[i].z << "," << v.upper[i].child <<"] ";

      o << "delta_lower: ";
      for (size_t i=0;i<4;i++) o << "[" << v.delta_lower[i].x << "," << v.delta_lower[i].y << "," << v.delta_lower[i].z << "," << v.delta_lower[i].w <<"] ";
      o << std::endl;
      o << "delta_upper: ";
      for (size_t i=0;i<4;i++) o << "[" << v.delta_upper[i].x << "," << v.delta_upper[i].y << "," << v.delta_upper[i].z << "," << v.delta_upper[i].w <<"] ";
      o << std::endl;
      return o;
    } 

}

#endif
