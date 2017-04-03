// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
#include "trianglei_mb.h"
#include "quadv.h"
#include "quadi.h"
#include "quadi_mb.h"
#include "subdivpatch1cached.h"
#include "object.h"

namespace embree
{
namespace isa
{
  /********************** Bezier1v **************************/

  Bezier1v::Type::Type () 
    : PrimitiveType("bezier1v",sizeof(Bezier1v),1) {} 
  
  size_t Bezier1v::Type::size(const char* This) const {
    return 1;
  }

  const Bezier1v::Type& Bezier1v::type() 
  {
    static Bezier1v::Type ty;
    return ty;
  }

  /********************** Bezier1i **************************/

  Bezier1i::Type::Type () 
    : PrimitiveType("bezier1i",sizeof(Bezier1i),1) {} 
  
  size_t Bezier1i::Type::size(const char* This) const {
    return 1;
  }

  const Bezier1i::Type& Bezier1i::type() 
  {
    static Bezier1i::Type ty;
    return ty;
  }

  /********************** Line4i **************************/

  template<>
  Line4i::Type::Type ()
    : PrimitiveType("line4i",sizeof(Line4i),4) {}

  template<>
  size_t Line4i::Type::size(const char* This) const {
    return ((Line4i*)This)->size();
  }

  template<>
  const Line4i::Type& Line4i::type() 
  {
    static Line4i::Type ty;
    return ty;
  }
  
  /********************** Triangle4 **************************/

  template<>
  Triangle4::Type::Type () 
    : PrimitiveType("triangle4",sizeof(Triangle4),4) {} 

  template<>
  size_t Triangle4::Type::size(const char* This) const {
    return ((Triangle4*)This)->size();
  }

  template<>
  const Triangle4::Type& Triangle4::type() 
  {
    static Triangle4::Type ty;
    return ty;
  }

  /********************** Triangle4v **************************/

  template<>
  Triangle4v::Type::Type () 
  : PrimitiveType("triangle4v",sizeof(Triangle4v),4) {} 
  
  template<>
  size_t Triangle4v::Type::size(const char* This) const {
    return ((Triangle4v*)This)->size();
  }

  template<>
  const Triangle4v::Type& Triangle4v::type() 
  {
    static Triangle4v::Type ty;
    return ty;
  }

  /********************** Triangle4i **************************/

  template<>
  Triangle4i::Type::Type () 
    : PrimitiveType("triangle4i",sizeof(Triangle4i),4) {} 

  template<>
  size_t Triangle4i::Type::size(const char* This) const {
    return ((Triangle4i*)This)->size();
  }

  template<>
  const Triangle4i::Type& Triangle4i::type() 
  {
    static Triangle4i::Type ty;
    return ty;
  }

  /********************** Triangle4vMB **************************/

  template<>
  Triangle4vMB::Type::Type () 
  : PrimitiveType("triangle4vmb",sizeof(Triangle4vMB),4) {} 
  
  template<>
  size_t Triangle4vMB::Type::size(const char* This) const {
    return ((Triangle4vMB*)This)->size();
  }

  template<>
  const Triangle4vMB::Type& Triangle4vMB::type() 
  {
    static Triangle4vMB::Type ty;
    return ty;
  }

  /********************** Triangle4iMB **************************/

  template<>
  Triangle4iMB::Type::Type ()
  : PrimitiveType("triangle4imb",sizeof(Triangle4iMB),4) {}

  template<>
  size_t Triangle4iMB::Type::size(const char* This) const {
    return ((Triangle4iMB*)This)->size();
  }

  template<>
  const Triangle4iMB::Type& Triangle4iMB::type() 
  {
    static Triangle4iMB::Type ty;
    return ty;
  }

  /********************** Quad4v **************************/

  template<>
  Quad4v::Type::Type () 
    : PrimitiveType("quad4v",sizeof(Quad4v),4) {}

  template<>
  size_t Quad4v::Type::size(const char* This) const {
    return ((Quad4v*)This)->size();
  }

  template<>
  const Quad4v::Type& Quad4v::type() 
  {
    static Quad4v::Type ty;
    return ty;
  }

  /********************** Quad4i **************************/

  template<>
  Quad4i::Type::Type () 
    : PrimitiveType("quad4i",sizeof(Quad4i),4) {} 

  template<>
  size_t Quad4i::Type::size(const char* This) const {
    return ((Quad4i*)This)->size();
  }

  template<>
  const Quad4i::Type& Quad4i::type() 
  {
    static Quad4i::Type ty;
    return ty;
  }

  /********************** Quad4iMB **************************/

  template<>
  Quad4iMB::Type::Type () 
  : PrimitiveType("quad4imb",sizeof(Quad4iMB),4) {} 
  
  template<>
  size_t Quad4iMB::Type::size(const char* This) const {
    return ((Quad4iMB*)This)->size();
  }

  template<>
  const Quad4iMB::Type& Quad4iMB::type() 
  {
    static Quad4iMB::Type ty;
    return ty;
  }

  /********************** SubdivPatch1 **************************/

  SubdivPatch1Cached::Type::Type () 
    : PrimitiveType("subdivpatch1",sizeof(SubdivPatch1Cached),1) {} 
  
  size_t SubdivPatch1Cached::Type::size(const char* This) const {
    return 1;
  }

  const SubdivPatch1Cached::Type& SubdivPatch1Cached::type() 
  {
    static SubdivPatch1Cached::Type ty;
    return ty;
  }

  /********************** SubdivPatch1Cached **************************/

  SubdivPatch1Cached::TypeCached::TypeCached () 
    : PrimitiveType("subdivpatch1cached",sizeof(SubdivPatch1Cached),1) {} 
  
  size_t SubdivPatch1Cached::TypeCached::size(const char* This) const {
    return 1;
  }

  const SubdivPatch1Cached::TypeCached& SubdivPatch1Cached::type_cached() 
  {
    static SubdivPatch1Cached::TypeCached ty;
    return ty;
  }

  /********************** Virtual Object **************************/

  Object::Type::Type () 
    : PrimitiveType("object",sizeof(Object),1) {} 

  size_t Object::Type::size(const char* This) const {
    return 1;
  }

  const Object::Type& Object::type() 
  {
    static Object::Type ty;
    return ty;
  }
}
}
