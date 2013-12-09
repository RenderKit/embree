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

#ifndef __EMBREE_BVH8I_BUILDER_H__
#define __EMBREE_BVH8I_BUILDER_H__

#include "bvh4i/bvh4i.h"
#include "bvh4i/bvh4i_builder_fast.h"
#include "geometry/triangle4.h"

namespace embree
{
  namespace isa
  {
    class BVH8iBuilder : public BVH4iBuilderFast
    {
      ALIGNED_CLASS;
    public:
      
      /*! Constructor. */
      BVH8iBuilder(BVH4i* bvh, BuildSource* source, void* geometry, const size_t minLeafSize = 1, const size_t maxLeafSize = inf);
            
      /*! parallel task to iterate over the triangles */
      TASK_RUN_FUNCTION(BVH8iBuilder,build_parallel);
      
      /* build function */
      void build(size_t threadIndex, size_t threadCount);
      
      Triangle4 *accel4;
    };
  }  
}
 
#endif
  
