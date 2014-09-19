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

#pragma once

#include "primitive.h"

namespace embree
{
  struct SubdivPatch1
  {
    struct Type : public PrimitiveType 
    {
      Type ();
      size_t blocks(size_t x) const; 
      size_t size(const char* This) const;
    };

    static Type type;

  public:

    /*! Default constructor. */
    __forceinline SubdivPatch1 () {}

    /*! Construction from vertices and IDs. */
    __forceinline SubdivPatch1 (const SubdivMesh::HalfEdge* edge, const Vec3fa* vertices, 
                                const unsigned int geomID, const unsigned int primID, const bool last)
      : edge(edge), vertices(vertices), geom(geomID), prim(primID | (last << 31)) { }

    /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

    /*! return geometry ID */
    template<bool list>
    __forceinline unsigned int geomID() const { 
      return geom; 
    }

    /*! return primitive ID */
    template<bool list>
    __forceinline unsigned int primID() const { 
      if (list) return prim & 0x7FFFFFFF; 
      else      return prim; 
    }

    /*! checks if this is the last primitive in list leaf mode */
    __forceinline int last() const { 
      return prim & 0x80000000; 
    }

    /*! builder interface to fill primitive */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      const PrimRef& prim = *prims;
      prims++;

      const unsigned last   = list && !prims;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const SubdivMesh* const mesh = scene->getSubdivMesh(geomID);
      new (this) SubdivPatch1(NULL,&mesh->getVertexPosition(0),geomID,primID,last); // FIXME: how to get edge from SubdivMesh?
    }

    /*! builder interface to fill primitive */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, const bool list)
    {
      const PrimRef& prim = prims[i];
      i++;

      const unsigned last = list && i >= end;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const SubdivMesh* const mesh = scene->getSubdivMesh(geomID);
      new (this) SubdivPatch1(NULL,&mesh->getVertexPosition(0),geomID,primID,last); // FIXME: how to get edge from SubdivMesh?
    }
    
  public:
    const SubdivMesh::HalfEdge* edge;       //!< pointer to first half edge of this patch
    const Vec3fa* vertices;                 //!< pointer to vertex array
    unsigned geom;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned prim;                          //!< primitive ID of this subdivision patch
  };
}
