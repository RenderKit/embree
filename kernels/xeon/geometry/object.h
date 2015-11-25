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

namespace embree
{
  struct Object
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

    /*! constructs a virtual object */
    Object (AccelSet* accel, unsigned item) 
    : accel(accel), item(item) {}

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, const bool list)
    {
      const PrimRef& prim = prims[i]; i++;
      new (this) Object((AccelSet*) scene->get(prim.geomID()), prim.primID());
    }

    /*! fill triangle from triangle list */
    __forceinline std::pair<BBox3fa,BBox3fa> fill_mblur(const PrimRef* prims, size_t& i, size_t end, Scene* scene, const bool list)
    {
      const PrimRef& prim = prims[i]; i++;
      const size_t geomID = prim.geomID();
      const size_t primID = prim.primID();
      AccelSet* obj = (AccelSet*) scene->get(geomID);
      new (this) Object(obj, primID);
      return obj->bounds_mblur(primID);
    }

  public:
    AccelSet* accel; //!< array of acceleration structure
    unsigned item;   //!< the nth acceleration structure referenced
  };
}
