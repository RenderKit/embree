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

#pragma once

#include "primitive.h"
#include "bezier1v.h"

namespace embree
{
  struct Bezier1i
  {
    struct Type : public PrimitiveType {
      Type ();
      size_t size(const char* This) const;
    };
    static Type type;

  public:

    /*! Default constructor. */
    __forceinline Bezier1i () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1i (const unsigned vertexID, const unsigned geomID, const unsigned primID)
      : vertexID(vertexID), geom(geomID), prim(primID) {}

    /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

    /*! returns geometry ID */
    __forceinline unsigned geomID() const { return geom; }

    /*! returns primitive ID */
    __forceinline unsigned primID() const { return prim; }

    /*! fill from list */
    __forceinline void fill(atomic_set<PrimRefBlockT<BezierPrim> >::block_iterator_unsafe& iter, Scene* scene, const bool list)
    {
      const BezierPrim& curve = *iter; iter++;
      const unsigned geomID = curve.geomID();
      const unsigned primID = curve.primID();
      const BezierCurves* in = (BezierCurves*) scene->get(geomID);
      const unsigned vertexID = in->curve(primID);
      new (this) Bezier1i(vertexID,geomID,primID);
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, const bool list)
    {
      const PrimRef& prim = prims[i];
      i++;
      const size_t geomID = prim.geomID();
      const size_t primID = prim.primID();
      const BezierCurves* curves = scene->getBezierCurves(geomID);
      const size_t vertexID = curves->curve(primID);
      new (this) Bezier1i(vertexID,geomID,primID);
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(const Bezier1v* prims, size_t& i, size_t end, Scene* scene, const bool list)
    {
      const Bezier1v& curve = prims[i]; i++;
      const unsigned geomID = curve.geomID();
      const unsigned primID = curve.primID();
      const BezierCurves* curves = scene->getBezierCurves(geomID);
      const size_t vertexID = curves->curve(primID);
      new (this) Bezier1i(vertexID,geomID,primID);
    }

  public:
    unsigned vertexID; //!< index of start vertex
    unsigned geom;     //!< geometry ID
    unsigned prim;     //!< primitive ID
  };
}
