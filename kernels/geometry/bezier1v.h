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

#pragma once

#include "primitive.h"
#include "../subdiv/bezier_curve.h"
#include "../common/primref.h"

namespace embree
{
  struct Bezier1v
  {
    struct Type : public PrimitiveType 
    {
      Type ();
      size_t size(const char* This) const;
    };
    static Type type;

  public:

    /* Returns maximal number of stored primitives */
    static __forceinline size_t max_size() { return 1; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

  public:

    /*! Default constructor. */
    __forceinline Bezier1v () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1v (const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const Vec3fa& p3, const unsigned int geomID, const unsigned int primID)
      : p0(p0), p1(p1), p2(p2), p3(p3), geom(geomID), prim(primID) {}

    /*! return primitive ID */
    __forceinline unsigned int primID() const { 
      return prim;
    }

    /*! return geometry ID */
    __forceinline unsigned int geomID() const { 
      return geom; 
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene)
    {
      const PrimRef& prim = prims[i];
      i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const NativeCurves* curves = scene->get<NativeCurves>(geomID);
      const unsigned id = curves->curve(primID);
      const Vec3fa& p0 = curves->vertex(id+0);
      const Vec3fa& p1 = curves->vertex(id+1);
      const Vec3fa& p2 = curves->vertex(id+2);
      const Vec3fa& p3 = curves->vertex(id+3);
      new (this) Bezier1v(p0,p1,p2,p3,geomID,primID);
    }

    friend std::ostream& operator<<(std::ostream& cout, const Bezier1v& b) 
    {
      return std::cout << "Bezier1v { " << std::endl << 
        " p0 = " << b.p0 << ", " << std::endl <<
        " p1 = " << b.p1 << ", " << std::endl <<
        " p2 = " << b.p2 << ", " << std::endl <<
        " p3 = " << b.p3 << ",  " << std::endl <<
        " geomID = " << b.geomID() << ", primID = " << b.primID() << std::endl << 
      "}";
    }
    
  public:
    Vec3fa p0;            //!< 1st control point (x,y,z,r)
    Vec3fa p1;            //!< 2nd control point (x,y,z,r)
    Vec3fa p2;            //!< 3rd control point (x,y,z,r)
    Vec3fa p3;            //!< 4th control point (x,y,z,r)
  private:
    unsigned geom;      //!< geometry ID
    unsigned prim;      //!< primitive ID
  };
}
