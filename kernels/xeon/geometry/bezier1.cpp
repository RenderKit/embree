// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "bezier1.h"

namespace embree
{
  Bezier1Type Bezier1Type::type;

  Bezier1Type::Bezier1Type () 
    : PrimitiveType("bezier1",sizeof(Bezier1<listMode>),1,true,1) {} 
  
  size_t Bezier1Type::blocks(size_t x) const {
    return x;
  }
    
  size_t Bezier1Type::size(const char* This) const {
    return 1;
  }
}
