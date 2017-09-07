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
#include "trianglev_mb.h"
#include "trianglei.h"
#include "quadv.h"
#include "quadi.h"
#include "subdivpatch1cached.h"
#include "object.h"

namespace embree
{
  /********************** Bezier1v **************************/

  Bezier1v::Type::Type ()
    : PrimitiveType("bezier1v",sizeof(Bezier1v),1,2,false) {}

  size_t Bezier1v::Type::size(const char* This) const {
    return 1;
  }

  bool Bezier1v::Type::last(const char* This) const {
    return ((Bezier1v*)This)->last();
  }

  Bezier1v::Type Bezier1v::type;

  /********************** Bezier1i **************************/

  Bezier1i::Type::Type ()
    : PrimitiveType("bezier1i",sizeof(Bezier1i),1,2,false) {}

  size_t Bezier1i::Type::size(const char* This) const {
    return 1;
  }

  bool Bezier1i::Type::last(const char* This) const {
    return ((Bezier1i*)This)->last();
  }

  Bezier1i::Type Bezier1i::type;

  /********************** Line4i **************************/

  template<>
  Line4i::Type::Type ()
    : PrimitiveType("line4i",sizeof(Line4i),4,4,false) {}

  template<>
  size_t Line4i::Type::size(const char* This) const {
    return ((Line4i*)This)->size();
  }

  template<>
  bool Line4i::Type::last(const char* This) const {
    return ((Line4i*)This)->last();
  }

  /********************** Triangle4 **************************/

  template<>
  Triangle4::Type::Type ()
    : PrimitiveType("triangle4",sizeof(Triangle4),4,4,false) {}

  template<>
  size_t Triangle4::Type::size(const char* This) const {
    return ((Triangle4*)This)->size();
  }

  template<>
  bool Triangle4::Type::last(const char* This) const {
    return ((Triangle4*)This)->last();
  }

  /********************** Triangle4v **************************/

  template<>
  Triangle4v::Type::Type ()
    : PrimitiveType("triangle4v",sizeof(Triangle4v),4,4,false) {}

  template<>
  size_t Triangle4v::Type::size(const char* This) const {
    return ((Triangle4v*)This)->size();
  }

  template<>
  bool Triangle4v::Type::last(const char* This) const {
    return ((Triangle4v*)This)->last();
  }
  
  /********************** Triangle4i **************************/

  template<>
  Triangle4i::Type::Type ()
    : PrimitiveType("triangle4i",sizeof(Triangle4i),4,4,false) {}

  template<>
  size_t Triangle4i::Type::size(const char* This) const {
    return ((Triangle4i*)This)->size();
  }

  template<>
  bool Triangle4i::Type::last(const char* This) const {
    return ((Triangle4i*)This)->last();
  }
  
  /********************** Triangle4vMB **************************/

  template<>
  Triangle4vMB::Type::Type ()
    : PrimitiveType("triangle4vmb",sizeof(Triangle4vMB),4,4,true) {}

  template<>
  size_t Triangle4vMB::Type::size(const char* This) const {
    return ((Triangle4vMB*)This)->size();
  }

  template<>
  bool Triangle4vMB::Type::last(const char* This) const {
    return ((Triangle4vMB*)This)->last();
  }

  /********************** Quad4v **************************/

  template<>
  Quad4v::Type::Type ()
    : PrimitiveType("quad4v",sizeof(Quad4v),4,4,false) {}

  template<>
  size_t Quad4v::Type::size(const char* This) const {
    return ((Quad4v*)This)->size();
  }

  template<>
  bool Quad4v::Type::last(const char* This) const {
    return ((Quad4v*)This)->last();
  }
  
  /********************** Quad4i **************************/

  template<>
  Quad4i::Type::Type ()
    : PrimitiveType("quad4i",sizeof(Quad4i),4,4,false) {}

  template<>
  size_t Quad4i::Type::size(const char* This) const {
    return ((Quad4i*)This)->size();
  }

  template<>
  bool Quad4i::Type::last(const char* This) const {
    return ((Quad4i*)This)->last();
  }

  /********************** SubdivPatch1 **************************/

  SubdivPatch1Cached::Type::Type ()
    : PrimitiveType("subdivpatch1",sizeof(SubdivPatch1Cached),1,1,false) {}

  size_t SubdivPatch1Cached::Type::size(const char* This) const {
    return 1;
  }

  bool SubdivPatch1Cached::Type::last(const char* This) const {
    return true;
  }
  
  SubdivPatch1Cached::Type SubdivPatch1Cached::type;

  /********************** SubdivPatch1Cached **************************/

  SubdivPatch1Cached::TypeCached::TypeCached ()
    : PrimitiveType("subdivpatch1cached",sizeof(SubdivPatch1Cached),1,1,false) {}

  size_t SubdivPatch1Cached::TypeCached::size(const char* This) const {
    return 1;
  }

  bool SubdivPatch1Cached::TypeCached::last(const char* This) const {
    return true;
  }
  
  SubdivPatch1Cached::TypeCached SubdivPatch1Cached::type_cached;

  /********************** Virtual Object **************************/

  Object::Type::Type ()
    : PrimitiveType("object",sizeof(Object),1,1,false) {}

  size_t Object::Type::size(const char* This) const {
    return 1;
  }

  bool Object::Type::last(const char* This) const {
    return ((Object*)This)->last();
  }

  Object::Type Object::type;
}
