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
#include "bezier1v.h"

namespace embree
{
  struct Bezier1i
  {
    struct Type : public PrimitiveType {
      Type ();
      size_t size(const char* This) const;
      bool last(const char* This) const;
    };
    static Type type;
    static const Leaf::Type leaf_type = Leaf::TY_HAIR;

  public:

    /* Returns maximal number of stored primitives */
    static __forceinline size_t max_size() { return 1; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

  public:

    /*! Default constructor. */
    __forceinline Bezier1i () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1i (const unsigned vertexID, const unsigned geomID, const unsigned primID, const Leaf::Type ty, const bool last)
      : vertexID(vertexID), geom(Leaf::encode(ty,geomID,last)), prim(primID) {}

     /*! checks if this is the last primitive */
    __forceinline unsigned last() const { return Leaf::decodeLast(geom); }

    /*! returns geometry ID */
    __forceinline unsigned geomID() const { return Leaf::decodeID(geom); }

    /*! returns primitive ID */
    __forceinline unsigned primID() const { return prim; }

    /*! fill curve from curve list */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, bool last)
    {
      const PrimRef& prim = prims[i];
      i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const NativeCurves* curves = scene->get<NativeCurves>(geomID);
      const unsigned vertexID = curves->curve(primID);
      new (this) Bezier1i(vertexID,geomID,primID,Leaf::TY_HAIR,last);
    }

    /*! fill curve from curve list */
    __forceinline LBBox3fa fillMB(const PrimRefMB* prims, size_t& i, size_t end, Scene* scene, const BBox1f time_range, bool last)
    {
      const PrimRefMB& prim = prims[i];
      i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const NativeCurves* curves = scene->get<NativeCurves>(geomID);
      const unsigned vertexID = curves->curve(primID);
      new (this) Bezier1i(vertexID,geomID,primID,Leaf::TY_HAIR_MB,last);
      return curves->linearBounds(primID,time_range);
    }

  public:
    unsigned vertexID; //!< index of start vertex
  private:
    unsigned geom;     //!< geometry ID
    unsigned prim;     //!< primitive ID

  };

  struct Bezier1iMB : public Bezier1i
  {
    static const Leaf::Type leaf_type = Leaf::TY_HAIR_MB;
    
    template<typename BVH>
    __forceinline static typename BVH::NodeRef createLeaf(const FastAllocator::CachedAllocator& alloc, PrimRef* prims, const range<size_t>& range, BVH* bvh)
    {
      assert(false);
      return BVH::emptyNode;
    }

    template<typename BVH>
      __forceinline static const typename BVH::NodeRecordMB4D createLeafMB (const SetMB& set, const FastAllocator::CachedAllocator& alloc, BVH* bvh)
    {
      size_t items = Bezier1i::blocks(set.object_range.size());
      size_t start = set.object_range.begin();
      Bezier1i* accel = (Bezier1i*) alloc.malloc1(items*sizeof(Bezier1i),BVH::byteAlignment);
      typename BVH::NodeRef node = bvh->encodeLeaf((char*)accel,Leaf::TY_HAIR_MB);
      LBBox3fa allBounds = empty;
      for (size_t i=0; i<items; i++)
        allBounds.extend(accel[i].fillMB(set.prims->data(), start, set.object_range.end(), bvh->scene, set.time_range, i==(items-1)));
      return typename BVH::NodeRecordMB4D(node,allBounds,set.time_range,0.0f,items);
    }
  };
}
