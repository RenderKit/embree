// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "primitive.h"
#include "triangle1v.h"
#include "quad1v.h"
#include "triangle1vmb.h"
#include "quad1vmb.h"

namespace embree
{

  /********************** Triangle1v **************************/
  Triangle1v::Type Triangle1v::type;
  
  const char* Triangle1v::Type::name () const {
    return "triangle1v";
  }

  size_t Triangle1v::Type::sizeActive(const char* This) const {
    return ((Triangle1v*)This)->size();
  }

  size_t Triangle1v::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t Triangle1v::Type::getBytes(const char* This) const {
    return sizeof(Triangle1v);
  }

  /********************** Quad1v **************************/
  Quad1v::Type Quad1v::type;
  
  const char* Quad1v::Type::name () const {
    return "quad1v";
  }

  size_t Quad1v::Type::sizeActive(const char* This) const {
    return ((Quad1v*)This)->size();
  }

  size_t Quad1v::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t Quad1v::Type::getBytes(const char* This) const {
    return sizeof(Quad1v);
  }


  /********************** Triangle1vMB **************************/
  Triangle1vMB::Type Triangle1vMB::type;
  
  const char* Triangle1vMB::Type::name () const {
    return "triangle1vMB";
  }

  size_t Triangle1vMB::Type::sizeActive(const char* This) const {
    return ((Triangle1vMB*)This)->size();
  }

  size_t Triangle1vMB::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t Triangle1vMB::Type::getBytes(const char* This) const {
    return sizeof(Triangle1vMB);
  }

  /********************** Quad1vMB **************************/
  Quad1vMB::Type Quad1vMB::type;
  
  const char* Quad1vMB::Type::name () const {
    return "quad1vMB";
  }

  size_t Quad1vMB::Type::sizeActive(const char* This) const {
    return ((Quad1vMB*)This)->size();
  }

  size_t Quad1vMB::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t Quad1vMB::Type::getBytes(const char* This) const {
    return sizeof(Quad1vMB);
  }

}
