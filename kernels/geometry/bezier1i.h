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

    /* Returns maximum number of stored primitives */
    static __forceinline size_t max_size() { return 1; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

    static __forceinline size_t bytes(size_t N) { return N*sizeof(Bezier1i); }

  public:

    /*! Default constructor. */
    __forceinline Bezier1i () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1i (const unsigned vertexID, const unsigned geomID, const unsigned primID)
      : vertexID(vertexID), geom(geomID), prim(primID) {}

    /* Returns the geometry ID */
    template<class T>
    static __forceinline T unmask(T &index) { return index & 0x3fffffff; }

    /*! returns geometry ID */
    __forceinline unsigned geomID() const { return unmask(geom); }

    /*! returns primitive ID */
    __forceinline unsigned primID() const { return prim; }

    /*! fill curve from curve list */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene)
    {
      const PrimRef& prim = prims[i];
      i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const NativeCurves* curves = scene->get<NativeCurves>(geomID);
      const unsigned vertexID = curves->curve(primID);
      /* encode the RTCCurveFlags into the two most significant bits */
      const unsigned int mask = curves->getStartEndBitMask(primID);
      new (this) Bezier1i(vertexID,geomID | mask,primID);
    }

    template<typename BVH, typename Allocator>
      __forceinline static typename BVH::NodeRef createLeaf (BVH* bvh, const PrimRef* prims, const range<size_t>& set, const Allocator& alloc)
    {
      size_t start = set.begin();
      size_t items = Bezier1i::blocks(set.size());
      Bezier1i* accel = (Bezier1i*) alloc.malloc1(items*sizeof(Bezier1i),BVH::byteAlignment);
      for (size_t i=0; i<items; i++) {
        accel[i].fill(prims,start,set.end(),bvh->scene);
      }
      return bvh->encodeLeaf((char*)accel,items);
    };
    
    /*! fill curve from curve list */
    __forceinline LBBox3fa fillMB(const PrimRefMB* prims, size_t& i, size_t end, Scene* scene, const BBox1f time_range)
    {
      const PrimRefMB& prim = prims[i];
      i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const NativeCurves* curves = scene->get<NativeCurves>(geomID);
      const unsigned vertexID = curves->curve(primID);
      /* encode the RTCCurveFlags into the two most significant bits */
      const unsigned int mask = curves->getStartEndBitMask(primID);
      new (this) Bezier1i(vertexID,geomID | mask,primID);
      return curves->linearBounds(primID,time_range);
    }

    template<typename BVH, typename SetMB, typename Allocator>
    __forceinline static typename BVH::NodeRecordMB4D createLeafMB(BVH* bvh, const SetMB& prims, const Allocator& alloc)
    {
      size_t start = prims.object_range.begin();
      size_t end   = prims.object_range.end();
      size_t items = prims.object_range.size();
      Bezier1i* accel = (Bezier1i*) alloc.malloc1(items*sizeof(Bezier1i));
      const typename BVH::NodeRef node = bvh->encodeLeaf((char*)accel,items);
      
      LBBox3fa bounds = empty;
      for (size_t i=0; i<items; i++)
        bounds.extend(accel[i].fillMB(prims.prims->data(),start,end,bvh->scene,prims.time_range));
      
      return typename BVH::NodeRecordMB4D(node,bounds,prims.time_range);
    };

  public:
    unsigned vertexID; //!< index of start vertex
  private:
    unsigned geom;     //!< geometry ID
    unsigned prim;     //!< primitive ID
  };
}
