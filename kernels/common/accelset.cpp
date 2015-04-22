// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "accelset.h"
#include "scene.h"

namespace embree
{
  AccelSet::AccelSet (Scene* parent, size_t numItems) 
    : Geometry(parent,Geometry::USER_GEOMETRY,numItems,1,RTC_GEOMETRY_STATIC), numItems(numItems) 
  {
    intersectors.ptr = nullptr; 
    enabling();
  }

  void AccelSet::enabling () { 
    atomic_add(&parent->numUserGeometries1,numItems); 
  }
  
  void AccelSet::disabling() { 
    atomic_add(&parent->numUserGeometries1,-(ssize_t)numItems); 
  }
}
