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
#include "curveNv.h"
#include "curveNi.h"
#include "curveNi_mb.h"
#include "linei.h"
#include "triangle.h"
#include "trianglev.h"
#include "trianglev_mb.h"
#include "trianglei.h"
#include "quadv.h"
#include "quadi.h"
#include "subdivpatch1.h"
#include "object.h"
#include "instance.h"
#include "subgrid.h"

namespace embree
{
  /********************** Curve4v **************************/

  template<>
  Curve4v::Type::Type ()
    : PrimitiveType("curve4v") {}

  template<>
  size_t Curve4v::Type::sizeActive(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return ((Line4i*)This)->size();
    else
      return ((Curve4v*)This)->N;
  }

  template<>
  size_t Curve4v::Type::sizeTotal(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return 4;
    else
      return ((Curve4v*)This)->N;
  }

  template<>
  size_t Curve4v::Type::getBytes(const char* This) const
  {
     if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
       return Line4i::bytes(sizeActive(This));
     else
       return Curve4v::bytes(sizeActive(This));
  }

  /********************** Curve8v **************************/

  template<>
  Curve8v::Type::Type ()
    : PrimitiveType("curve8v") {}

  template<>
  size_t Curve8v::Type::sizeActive(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return ((Line8i*)This)->size();
    else
      return ((Curve8v*)This)->N;
  }

  template<>
  size_t Curve8v::Type::sizeTotal(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return 8;
    else
      return ((Curve8v*)This)->N;
  }

  template<>
  size_t Curve8v::Type::getBytes(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
       return Line8i::bytes(sizeActive(This));
     else
       return Curve8v::bytes(sizeActive(This));
  }

  /********************** Curve4i **************************/

  template<>
  Curve4i::Type::Type ()
    : PrimitiveType("curve4i") {}

  template<>
  size_t Curve4i::Type::sizeActive(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return ((Line4i*)This)->size();
    else
      return ((Curve4i*)This)->N;
  }

  template<>
  size_t Curve4i::Type::sizeTotal(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return 4;
    else
      return ((Curve4i*)This)->N;
  }

  template<>
  size_t Curve4i::Type::getBytes(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
       return Line4i::bytes(sizeActive(This));
     else
       return Curve4i::bytes(sizeActive(This));
  }

  /********************** Curve8i **************************/

  template<>
  Curve8i::Type::Type ()
    : PrimitiveType("curve8i") {}

  template<>
  size_t Curve8i::Type::sizeActive(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return ((Line8i*)This)->size();
    else
      return ((Curve8i*)This)->N;
  }

  template<>
  size_t Curve8i::Type::sizeTotal(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return 8;
    else
      return ((Curve8i*)This)->N;
  }

  template<>
  size_t Curve8i::Type::getBytes(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
       return Line8i::bytes(sizeActive(This));
     else
       return Curve8i::bytes(sizeActive(This));
  }

  /********************** Curve4iMB **************************/

  template<>
  Curve4iMB::Type::Type ()
    : PrimitiveType("curve4imb") {}

  template<>
  size_t Curve4iMB::Type::sizeActive(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return ((Line4i*)This)->size();
    else
      return ((Curve4iMB*)This)->N;
  }

  template<>
  size_t Curve4iMB::Type::sizeTotal(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return 4;
    else
      return ((Curve4iMB*)This)->N;
  }

  template<>
  size_t Curve4iMB::Type::getBytes(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
       return Line4i::bytes(sizeActive(This));
     else
       return Curve4iMB::bytes(sizeActive(This));
  }

  /********************** Curve8iMB **************************/

  template<>
  Curve8iMB::Type::Type ()
    : PrimitiveType("curve8imb") {}

  template<>
  size_t Curve8iMB::Type::sizeActive(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return ((Line8i*)This)->size();
    else
      return ((Curve8iMB*)This)->N;
  }

  template<>
  size_t Curve8iMB::Type::sizeTotal(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
      return 8;
    else
      return ((Curve8iMB*)This)->N;
  }

  template<>
  size_t Curve8iMB::Type::getBytes(const char* This) const
  {
    if ((*This & Geometry::GType::GTY_BASIS_MASK) == Geometry::GType::GTY_BASIS_LINEAR)
       return Line8i::bytes(sizeActive(This));
     else
       return Curve8iMB::bytes(sizeActive(This));
  }
  
  /********************** Line4i **************************/

  template<>
  Line4i::Type::Type ()
    : PrimitiveType("line4i") {}

  template<>
  size_t Line4i::Type::sizeActive(const char* This) const {
    return ((Line4i*)This)->size();
  }

  template<>
  size_t Line4i::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Line4i::Type::getBytes(const char* This) const {
    return sizeof(Line4i);
  }

  /********************** Triangle4 **************************/

  template<>
  Triangle4::Type::Type ()
    : PrimitiveType("triangle4") {}

  template<>
  size_t Triangle4::Type::sizeActive(const char* This) const {
    return ((Triangle4*)This)->size();
  }

  template<>
  size_t Triangle4::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4::Type::getBytes(const char* This) const {
    return sizeof(Triangle4);
  }

  /********************** Triangle4v **************************/

  template<>
  Triangle4v::Type::Type ()
  : PrimitiveType("triangle4v") {}

  template<>
  size_t Triangle4v::Type::sizeActive(const char* This) const {
    return ((Triangle4v*)This)->size();
  }

  template<>
  size_t Triangle4v::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4v::Type::getBytes(const char* This) const {
    return sizeof(Triangle4v);
  }

  /********************** Triangle4i **************************/

  template<>
  Triangle4i::Type::Type ()
    : PrimitiveType("triangle4i") {}

  template<>
  size_t Triangle4i::Type::sizeActive(const char* This) const {
    return ((Triangle4i*)This)->size();
  }

  template<>
  size_t Triangle4i::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4i::Type::getBytes(const char* This) const {
    return sizeof(Triangle4i);
  }

  /********************** Triangle4vMB **************************/

  template<>
  Triangle4vMB::Type::Type ()
  : PrimitiveType("triangle4vmb") {}

  template<>
  size_t Triangle4vMB::Type::sizeActive(const char* This) const {
    return ((Triangle4vMB*)This)->size();
  }

  template<>
  size_t Triangle4vMB::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4vMB::Type::getBytes(const char* This) const {
    return sizeof(Triangle4vMB);
  }

  /********************** Quad4v **************************/

  template<>
  Quad4v::Type::Type ()
    : PrimitiveType("quad4v") {}

  template<>
  size_t Quad4v::Type::sizeActive(const char* This) const {
    return ((Quad4v*)This)->size();
  }

  template<>
  size_t Quad4v::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Quad4v::Type::getBytes(const char* This) const {
    return sizeof(Quad4v);
  }

  /********************** Quad4i **************************/

  template<>
  Quad4i::Type::Type ()
    : PrimitiveType("quad4i") {}

  template<>
  size_t Quad4i::Type::sizeActive(const char* This) const {
    return ((Quad4i*)This)->size();
  }

  template<>
  size_t Quad4i::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Quad4i::Type::getBytes(const char* This) const {
    return sizeof(Quad4i);
  }

  /********************** SubdivPatch1 **************************/

  SubdivPatch1::Type::Type ()
    : PrimitiveType("subdivpatch1") {}

  size_t SubdivPatch1::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t SubdivPatch1::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t SubdivPatch1::Type::getBytes(const char* This) const {
    return sizeof(SubdivPatch1);
  }

  SubdivPatch1::Type SubdivPatch1::type;

  /********************** Virtual Object **************************/

  Object::Type::Type ()
    : PrimitiveType("object") {}

  size_t Object::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t Object::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t Object::Type::getBytes(const char* This) const {
    return sizeof(Object);
  }

  Object::Type Object::type;

  /********************** Instance **************************/

  InstancePrimitive::Type::Type ()
    : PrimitiveType("instance") {}

  size_t InstancePrimitive::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t InstancePrimitive::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t InstancePrimitive::Type::getBytes(const char* This) const {
    return sizeof(InstancePrimitive);
  }

  InstancePrimitive::Type InstancePrimitive::type;

  /********************** SubGrid **************************/

  SubGrid::Type::Type ()
    : PrimitiveType("subgrid") {}

  size_t SubGrid::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t SubGrid::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t SubGrid::Type::getBytes(const char* This) const {
    return sizeof(SubGrid);
  }

  SubGrid::Type SubGrid::type;
  
  /********************** SubGridQBVH4 **************************/

  template<>
  SubGridQBVH4::Type::Type ()
    : PrimitiveType("SubGridQBVH4") {}

  template<>
  size_t SubGridQBVH4::Type::sizeActive(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH4::Type::sizeTotal(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH4::Type::getBytes(const char* This) const {
    return sizeof(SubGridQBVH4);
  }

  /********************** SubGridQBVH8 **************************/

  template<>
  SubGridQBVH8::Type::Type ()
    : PrimitiveType("SubGridQBVH8") {}

  template<>
  size_t SubGridQBVH8::Type::sizeActive(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH8::Type::sizeTotal(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH8::Type::getBytes(const char* This) const {
    return sizeof(SubGridQBVH8);
  }
}
