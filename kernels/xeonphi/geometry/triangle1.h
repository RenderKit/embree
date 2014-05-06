// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#pragma once

#include "primitive.h"

namespace embree
{
  struct __aligned(64) Triangle1
  {
  public:

    /*! Default constructor. */
    __forceinline Triangle1 () {}

    /*! Construction from vertices and IDs. */
    __forceinline Triangle1 (const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const unsigned int geomID, const unsigned int primID, const unsigned int mask)
      : v0(v0,primID), v1(v1,geomID), v2(v2,mask), Ng(cross(v0-v1,v2-v0)) {}

    /*! calculate the bounds of the triangle */
    __forceinline BBox3fa bounds() const {
      return merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2));
    }

    /*! access hidden members */
    __forceinline unsigned int primID() const { return v0.a; }
    __forceinline unsigned int geomID() const { return v1.a; }
    __forceinline unsigned int mask  () const { return v2.a; }
    
  public:
    Vec3fa v0;          //!< first vertex and primitive ID
    Vec3fa v1;          //!< second vertex and geometry ID
    Vec3fa v2;          //!< third vertex and geometry mask
    Vec3fa Ng;          //!< Geometry normal of the triangles.
  };

  struct Triangle1Type : public PrimitiveType {
    Triangle1Type ();
    size_t blocks(size_t x) const;
    size_t size(const char* This) const;
  };

  struct SceneTriangle1 : public Triangle1Type
  {
    static SceneTriangle1 type;
  };

  struct TriangleMeshTriangle1 : public Triangle1Type
  {
    static TriangleMeshTriangle1 type;
  };


  __forceinline std::ostream &operator<<(std::ostream &o, const embree::Triangle1 &v)
  {
    o << "v0 " << v.v0 << " v1 " << v.v1 << " v2 " << v.v2 << " Ng " << v.Ng;
    return o;
  } 


  struct __aligned(32) Triangle1mc
  {
  public:
    Vec3fa *__restrict__ v0;
    Vec3fa *__restrict__ v1;
    Vec3fa *__restrict__ v2;
    int geometryID;
    int primitiveID;    

    __forceinline unsigned int primID() const { return primitiveID; }
    __forceinline unsigned int geomID() const { return geometryID; }

  };

  __forceinline std::ostream &operator<<(std::ostream &o, const embree::Triangle1mc &v)
  {
    o << "v0 " << v.v0 << " v1 " << v.v1 << " v2 " << v.v2 << " geomID " << v.geomID() << " primID " << v.primID();
    return o;
  } 

}
