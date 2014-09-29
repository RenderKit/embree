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

#include "subdivpatchdispl1.h"
#include "common/scene.h"

namespace embree
{
  SubdivPatchDispl1::Type SubdivPatchDispl1::type;
  
  SubdivPatchDispl1::Type::Type () 
    : PrimitiveType("subdivpatchdispl1",sizeof(SubdivPatchDispl1),1,false,1) {} 
  
  size_t SubdivPatchDispl1::Type::blocks(size_t x) const {
    return x;
  }
    
  size_t SubdivPatchDispl1::Type::size(const char* This) const {
    return 1;
  }
}
