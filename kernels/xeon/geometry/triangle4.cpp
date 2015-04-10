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

#include "triangle4.h"
#if defined(__TARGET_AVX__)
#include "triangle8.h"
#endif
#include "common/scene.h"

namespace embree
{
  Triangle4Type Triangle4Type::type;
  TriangleMeshTriangle4 TriangleMeshTriangle4::type;

  Triangle4Type::Triangle4Type () 
  : PrimitiveType("triangle4",sizeof(Triangle4),4,false,1) {} 

#if defined(__TARGET_AVX__)
  Triangle8Type Triangle8Type::type;
  TriangleMeshTriangle8 TriangleMeshTriangle8::type;

  Triangle8Type::Triangle8Type () 
    : PrimitiveType("triangle8",2*sizeof(Triangle4),8,false,1) {}
#endif
  
  size_t Triangle4Type::blocks(size_t x) const {
    return (x+3)/4;
  }
  
  size_t Triangle4Type::size(const char* This) const {
    return ((Triangle4*)This)->size();
  }
  
  size_t Triangle4Type::hash(const char* This, size_t num) const 
  {
    size_t hash = 0;
    for (size_t i=0; i<num; i++)
      hash += (i+1)*((Triangle4*)This)[i].hash();
    return hash;
  }
}

