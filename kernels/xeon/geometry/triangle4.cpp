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

namespace embree
{
  Triangle4::Type Triangle4::type;
  TriangleMeshTriangle4 TriangleMeshTriangle4::type; // FIXME: remove

  Triangle4::Type::Type () 
    : PrimitiveType("triangle4",sizeof(Triangle4),4) {} 

  size_t Triangle4::Type::size(const char* This) const {
    return ((Triangle4*)This)->size();
  }

#if defined(__TARGET_AVX__)
  Triangle8::Type Triangle8::type;
  TriangleMeshTriangle8 TriangleMeshTriangle8::type; // FIXME: remove

  Triangle8::Type::Type () 
    : PrimitiveType("triangle8",2*sizeof(Triangle4),8) {}
#endif
}

