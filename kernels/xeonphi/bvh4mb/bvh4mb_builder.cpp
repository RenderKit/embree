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

#include "bvh4mb/bvh4mb_builder.h"

namespace embree
{
  

  Builder* BVH4mbBuilder::create (void* accel, BuildSource* source, void* geometry, size_t mode ) 
  { 
    Builder* builder = new BVH4mbBuilder((BVH4mb*)accel,source,geometry);
    return builder;
  }

  void BVH4mbBuilder::printBuilderName()
  {
    std::cout << "building BVH4mb with binned SAH builder (MIC) ... " << std::endl;    
  }


  void BVH4mbBuilder::createAccel(const size_t threadIndex, const size_t threadCount)
  {
    LockStepTaskScheduler::dispatchTask( task_createTriangle1Accel, this, threadIndex, threadCount );   
  }

}
