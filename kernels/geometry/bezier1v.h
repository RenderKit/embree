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
    __forceinline Bezier1v () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1v (const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const Vec3fa& p3, const unsigned int geomID, const unsigned int primID, const Leaf::Type ty, bool last)
      : p0(p0), p1(p1), p2(p2), p3(p3), geom(Leaf::encode(ty,geomID,last)), prim(primID) {}

    /*! checks if this is the last primitive */
    __forceinline unsigned last() const { return Leaf::decodeLast(geom); }

    /*! returns geometry ID */
    __forceinline unsigned geomID() const { return Leaf::decodeID(geom); }

    /*! returns primitive ID */
    __forceinline unsigned primID() const { return prim; }

    /*! fill triangle from triangle list */
    template<typename PrimRef>
    __forceinline BBox3fa fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, bool last)
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
      new (this) Bezier1v(p0,p1,p2,p3,geomID,primID,Leaf::TY_HAIR,last);
      return curves->bounds(primID);
    }

    template<typename BVH>
    __forceinline static typename BVH::NodeRef createLeaf(const FastAllocator::CachedAllocator& alloc, PrimRef* prims, const range<size_t>& range, BVH* bvh)
    {
      size_t cur = range.begin();
      size_t items = blocks(range.size());
      Bezier1v* accel = (Bezier1v*) alloc.malloc1(items*sizeof(Bezier1v),BVH::byteAlignment);
      for (size_t i=0; i<items; i++) {
        accel[i].fill(prims,cur,range.end(),bvh->scene,i==(items-1));
      }
      return BVH::encodeLeaf((char*)accel,Leaf::TY_HAIR);
    }

    template<typename BVH>
    __forceinline static const typename BVH::NodeRecordMB4D createLeafMB (const SetMB& set, const FastAllocator::CachedAllocator& alloc, BVH* bvh)
    {
      size_t items = blocks(set.object_range.size());
      size_t start = set.object_range.begin();
      Bezier1v* accel = (Bezier1v*) alloc.malloc1(items*sizeof(Bezier1v),BVH::byteAlignment);
      typename BVH::NodeRef node = bvh->encodeLeaf((char*)accel,Leaf::TY_HAIR);
      LBBox3fa allBounds = empty;
      for (size_t i=0; i<items; i++) {
        const BBox3fa b = accel[i].fill(set.prims->data(),start,set.object_range.end(),bvh->scene,i==(items-1));
        allBounds.extend(LBBox3fa(b));
      }
      return typename BVH::NodeRecordMB4D(node,allBounds,set.time_range,0.0f,items);
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
