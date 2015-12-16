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
#include "linei.h"
#include "triangle.h"
#include "trianglev.h"
#include "trianglei.h"
#include "trianglev_mb.h"
#include "quadv.h"
#include "quadi.h"
#include "quadi_mb.h"
#include "subdivpatch1cached.h"
#include "object.h"

namespace embree
{

  /********************** Bezier1v **************************/

#if !defined(__AVX__)
  Bezier1v::Type::Type () 
    : PrimitiveType("bezier1v",sizeof(Bezier1v),1) {} 
  
  size_t Bezier1v::Type::size(const char* This) const {
    return 1;
  }

  Bezier1v::Type Bezier1v::type;
#endif

  /********************** Bezier1i **************************/

#if !defined(__AVX__)
  Bezier1i::Type::Type () 
    : PrimitiveType("bezier1i",sizeof(Bezier1i),1) {} 
  
  size_t Bezier1i::Type::size(const char* This) const {
    return 1;
  }

  Bezier1i::Type Bezier1i::type;
#endif

  /********************** Line4i **************************/

#if !defined(__AVX__)
  template<>
  Line4i::Type::Type ()
    : PrimitiveType("line4i",sizeof(Line4i),4) {}

  template<>
  size_t Line4i::Type::size(const char* This) const {
    return ((Line4i*)This)->size();
  }
#endif
  
  /********************** Triangle4 **************************/

#if !defined(__AVX__)
  template<>
  Triangle4::Type::Type () 
    : PrimitiveType("triangle4",sizeof(Triangle4),4) {} 

  template<>
  size_t Triangle4::Type::size(const char* This) const {
    return ((Triangle4*)This)->size();
  }
#endif

  /********************** Triangle4v **************************/

#if !defined(__AVX__)
  template<>
  Triangle4v::Type::Type () 
  : PrimitiveType("triangle4v",sizeof(Triangle4v),4) {} 
  
  template<>
  size_t Triangle4v::Type::size(const char* This) const {
    return ((Triangle4v*)This)->size();
  }
#endif

  /********************** Triangle4i **************************/

#if !defined(__AVX__)
  template<>
  Triangle4i::Type::Type () 
    : PrimitiveType("triangle4i",sizeof(Triangle4i),4) {} 

  template<>
  size_t Triangle4i::Type::size(const char* This) const {
    return ((Triangle4i*)This)->size();
  }
#endif

  /********************** Triangle4vMB **************************/

#if !defined(__AVX__)
  template<>
  Triangle4vMB::Type::Type () 
  : PrimitiveType("triangle4vmb",sizeof(Triangle4vMB),4) {} 
  
  template<>
  size_t Triangle4vMB::Type::size(const char* This) const {
    return ((Triangle4vMB*)This)->size();
  }
#endif

  /********************** Triangle8 **************************/

#if defined(__TARGET_AVX__)
#if !defined(__AVX__)
  template<>
  Triangle8::Type::Type () 
    : PrimitiveType("triangle8",2*sizeof(Triangle4),8) {}
#else
  template<>
  size_t Triangle8::Type::size(const char* This) const {
    return ((Triangle8*)This)->size();
  }
#endif
#endif

  /********************** Quad4v **************************/

#if !defined(__AVX__)
  template<>
  Quad4v::Type::Type () 
    : PrimitiveType("quad4v",sizeof(Quad4v),4) {}

  template<>
  size_t Quad4v::Type::size(const char* This) const {
    return ((Quad4v*)This)->size();
  }
#endif

  /********************** Quad4i **************************/

#if !defined(__AVX__)
  template<>
  Quad4i::Type::Type () 
    : PrimitiveType("quad4i",sizeof(Quad4i),4) {} 

  template<>
  size_t Quad4i::Type::size(const char* This) const {
    return ((Quad4i*)This)->size();
  }
#endif

  /********************** Quad4iMB **************************/

#if !defined(__AVX__)
  template<>
  Quad4iMB::Type::Type () 
  : PrimitiveType("quad4imb",sizeof(Quad4iMB),4) {} 
  
  template<>
  size_t Quad4iMB::Type::size(const char* This) const {
    return ((Quad4iMB*)This)->size();
  }
#endif


  /********************** SubdivPatch1Cached **************************/

#if !defined(__AVX__)
  SubdivPatch1Cached::Type::Type () 
    : PrimitiveType("subdivpatch1cached",sizeof(SubdivPatch1Cached),1) {} 
  
  size_t SubdivPatch1Cached::Type::size(const char* This) const {
    return 1;
  }

  SubdivPatch1Cached::Type SubdivPatch1Cached::type;
#endif

  /********************** SubdivPatch1Eager **************************/

#if !defined(__AVX__)
  SubdivPatch1Eager::Type::Type () 
    : PrimitiveType("subdivpatch1eager",sizeof(SubdivPatch1Eager),1) {} 
  
  size_t SubdivPatch1Eager::Type::size(const char* This) const {
    return 1;
  }

  SubdivPatch1Eager::Type SubdivPatch1Eager::type;
#endif

  /********************** Virtual Object **************************/

#if !defined(__AVX__)
  Object::Type::Type () 
    : PrimitiveType("object",sizeof(Object),1) {} 

  size_t Object::Type::size(const char* This) const {
    return 1;
  }

  Object::Type Object::type;
#endif
}


