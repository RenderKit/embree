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

#ifndef __EMBREE_BVH16I_MIC_H__
#define __EMBREE_BVH16I_MIC_H__

#include "bvh4i/bvh4i.h"

namespace embree
{
  /*! Multi BVH with 16 children. Each node stores the bounding box of
   * it's 16 children as well as a 16 child indices. */
  class BVH16i : public BVH4i
  {
  public:

    /*! BVH16i Node */

    struct __align(64) Node
    {
    public:

      mic_f min_x;
      mic_f max_x;
      mic_f min_y;
      mic_f max_y;
      mic_f min_z;
      mic_f max_z;
      mic_i child;
      mic_i data;


      __forceinline std::ostream& operator<<(std::ostream &o)
      {
	o << min_x << std::endl;
	o << max_x << std::endl;
	o << min_y << std::endl;
	o << max_y << std::endl;
	o << min_z << std::endl;
	o << max_z << std::endl;
	o << child << std::endl;
	o << data  << std::endl;
	return o;
      }

    };


  public:

    /*! BVH4 default constructor. */
    BVH16i (const PrimitiveType& primTy, void* geometry = NULL) : BVH4i(primTy,geometry)
    {
    }


    static Accel* BVH16iTriangle1ObjectSplitBinnedSAH(Scene* scene);

  };

}

#endif
