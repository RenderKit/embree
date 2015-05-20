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

#include "primitive.h"
#include "bezier1v.h"
#include "bezier1i.h"
#include "triangle4.h"
#include "triangle4v.h"
#include "triangle4i.h"
#include "triangle4v_mb.h"
#include "triangle8.h"
#include "triangle8v.h"
#include "trianglepairs8.h"
#include "subdivpatch1.h"
#include "subdivpatch1cached.h"
#include "object.h"

namespace embree
{
#if !defined(__AVX__)
  PrimitiveType2 PrimitiveType2::type; 
#endif

  /********************** Bezier1v **************************/

#if !defined(__AVX__)
  Bezier1v::Type Bezier1v::type;

  Bezier1v::Type::Type () 
    : PrimitiveType("bezier1v",sizeof(Bezier1v),1) {} 
  
  size_t Bezier1v::Type::size(const char* This) const {
    return 1;
  }
#endif

  /********************** Bezier1i **************************/

#if !defined(__AVX__)
  Bezier1i::Type Bezier1i::type;

  Bezier1i::Type::Type () 
    : PrimitiveType("bezier1i",sizeof(Bezier1i),1) {} 
  
  size_t Bezier1i::Type::size(const char* This) const {
    return 1;
  }
#endif
  
  /********************** Triangle4 **************************/

#if !defined(__AVX__)
  Triangle4::Type Triangle4::type;

  Triangle4::Type::Type () 
    : PrimitiveType("triangle4",sizeof(Triangle4),4) {} 

  size_t Triangle4::Type::size(const char* This) const {
    return ((Triangle4*)This)->size();
  }
#endif

  /********************** Triangle4v **************************/

#if !defined(__AVX__)
  Triangle4v::Type Triangle4v::type;

  Triangle4v::Type::Type () 
  : PrimitiveType("triangle4v",sizeof(Triangle4v),4) {} 
  
  size_t Triangle4v::Type::size(const char* This) const {
    return ((Triangle4v*)This)->size();
  }
#endif

  /********************** Triangle4i **************************/

#if !defined(__AVX__)
  Triangle4i::Type Triangle4i::type;

  Triangle4i::Type::Type () 
    : PrimitiveType("triangle4i",sizeof(Triangle4i),4) {} 
  
  size_t Triangle4i::Type::size(const char* This) const {
    return ((Triangle4i*)This)->size();
  }
#endif

  /********************** Triangle4vMB **************************/

#if !defined(__AVX__)
  Triangle4vMB::Type Triangle4vMB::type;

  Triangle4vMB::Type::Type () 
  : PrimitiveType("triangle4vmb",sizeof(Triangle4vMB),4) {} 
  
  size_t Triangle4vMB::Type::size(const char* This) const {
    return ((Triangle4vMB*)This)->size();
  }
#endif

  /********************** Triangle8 **************************/

#if defined(__TARGET_AVX__)
#if !defined(__AVX__)
  Triangle8::Type Triangle8::type;

  Triangle8::Type::Type () 
    : PrimitiveType("triangle8",2*sizeof(Triangle4),8) {}
#else
  size_t Triangle8::Type::size(const char* This) const {
    return ((Triangle8*)This)->size();
  }
#endif
#endif

  /********************** Triangle8v **************************/

#if defined(__TARGET_AVX__)
#if !defined(__AVX__)
  Triangle8v::Type Triangle8v::type;

  Triangle8v::Type::Type () 
    : PrimitiveType("triangle8v",11*32,8) {}
#else
  size_t Triangle8v::Type::size(const char* This) const {
    return ((Triangle8v*)This)->size();
  }
#endif
#endif

  /********************** TrianglePairs8 **************************/

#if defined(__TARGET_AVX__)
#if !defined(__AVX__)
  TrianglePairs8::Type TrianglePairs8::type;

  TrianglePairs8::Type::Type () 
    : PrimitiveType("trianglepairs8",11*32,8) {}
#else
  size_t TrianglePairs8::Type::size(const char* This) const {
    return ((TrianglePairs8*)This)->size();
  }
#endif
#endif

  /********************** SubdivPatch1 **************************/

#if !defined(__AVX__)
  SubdivPatch1::Type SubdivPatch1::type;
  
  SubdivPatch1::Type::Type () 
    : PrimitiveType("subdivpatch1",sizeof(SubdivPatch1),1) {} 
  
  size_t SubdivPatch1::Type::size(const char* This) const {
    return 1;
  }
#endif

  /********************** SubdivPatch1Cached **************************/

#if !defined(__AVX__)
  SubdivPatch1Cached::Type SubdivPatch1Cached::type;
  
  SubdivPatch1Cached::Type::Type () 
    : PrimitiveType("subdivpatch1",sizeof(SubdivPatch1Cached),1) {} 
  
  size_t SubdivPatch1Cached::Type::size(const char* This) const {
    return 1;
  }
#endif

  /********************** Virtual Object **************************/

#if !defined(__AVX__)
  Object::Type Object::type;

  Object::Type::Type () 
    : PrimitiveType("object",sizeof(Object),1) {} 

  size_t Object::Type::size(const char* This) const {
    return 1;
  }
#endif
}
