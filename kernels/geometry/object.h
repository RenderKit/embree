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
      bool last(const char* This) const;
    };
    static Type type;
    static const Leaf::Type leaf_type = Leaf::TY_NULL; 

  public:

    /* primitive supports multiple time segments */
    static const bool singleTimeSegment = false;

    /* Returns maximal number of stored primitives */
    static __forceinline size_t max_size() { return 1; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

  public:

    /*! constructs a virtual object */
    Object (unsigned geomID, unsigned primID, bool last) 
    : _geomID(Leaf::encode(Leaf::TY_OBJECT,geomID,last)), _primID(primID) {}

    /*! checks if this is the last primitive */
    __forceinline unsigned last() const { return Leaf::decodeLast(_geomID); }
    
    /*! returns geomID */
    __forceinline unsigned geomID() const { return Leaf::decodeID(_geomID); }

    /*! returns primID */
    __forceinline unsigned primID() const { return _primID; }

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, bool last)
    {
      const PrimRef& prim = prims[i]; i++;
      new (this) Object(prim.geomID(), prim.primID(),last);
    }

    /*! fill triangle from triangle list */
    __forceinline LBBox3fa fillMB(const PrimRef* prims, size_t& i, size_t end, Scene* scene, size_t itime, bool last)
    {
      const PrimRef& prim = prims[i]; i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      new (this) Object(geomID, primID,last);
      AccelSet* accel = (AccelSet*) scene->get(geomID);
      return accel->linearBounds(primID,itime);
    }

    /*! fill triangle from triangle list */
    __forceinline LBBox3fa fillMB(const PrimRefMB* prims, size_t& i, size_t end, Scene* scene, const BBox1f time_range, bool last)
    {
      const PrimRefMB& prim = prims[i]; i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      new (this) Object(geomID, primID,last);
      AccelSet* accel = (AccelSet*) scene->get(geomID);
      return accel->linearBounds(primID,time_range);
    }

    /* Updates the primitive */
    __forceinline BBox3fa update(AccelSet* mesh) {
      return mesh->bounds(primID());
    }

  private:
    unsigned _geomID;  //!< geometry ID
    unsigned _primID;  //!< primitive ID
  };
}
