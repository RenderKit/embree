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

#ifndef __EMBREE_BVH4MB_BUILDER_MIC_H__
#define __EMBREE_BVH4MB_BUILDER_MIC_H__

#include "bvh4i/bvh4i_builder.h"
#include "bvh4mb.h"

namespace embree
{


  /*! derived binned-SAH builder supporting virtual geometry */  
  class BVH4mbBuilder : public BVH4iBuilder
  {
  public:

    /*! creates the builder */
    static Builder* create (void* accel, BuildSource* source, void* geometry, size_t mode = BVH4I_BUILDER_DEFAULT);

  BVH4mbBuilder(BVH4mb* bvh, BuildSource* source, void* geometry) : BVH4iBuilder((BVH4i*)bvh,source,geometry) 
      {
	numNodesToAllocate = 2 * BVH4i::N; /* 8 */
      }

    virtual void allocateData(const size_t threadCount, const size_t newNumPrimitives);
    virtual void convertQBVHLayout(const size_t threadIndex, const size_t threadCount);

    virtual void createAccel(const size_t threadIndex, const size_t threadCount);
    virtual void printBuilderName();

    void refit(const size_t index, BBox3f& dest);

  protected:
    TASK_FUNCTION(BVH4mbBuilder,createTriangle01AccelMB);    
    TASK_FUNCTION(BVH4mbBuilder,convertToSOALayoutMB);    

  };

}

#endif
