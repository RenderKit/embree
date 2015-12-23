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
  template <int M>
  struct PointMi
  {
    typedef Vec3<vfloat<M>> Vec3vfM;

    /* Virtual interface to query information about the point type */
    struct Type : public PrimitiveType
    {
      Type();
      size_t size(const char* This) const;
    };
    static Type type;

  public:

    /* Returns maximal number of stored points */
    static __forceinline size_t max_size() { return M; }

    /* Returns required number of primitive blocks for N points */
    static __forceinline size_t blocks(size_t N) { return (N+max_size()-1)/max_size(); }

  public:

    /* Default constructor */
    __forceinline PointMi() {  }

    /* Construction from vertices and IDs */
    __forceinline PointMi(const vint<M>& v0, const vint<M>& geomIDs, const vint<M>& primIDs)
      : v0(v0), geomIDs(geomIDs), primIDs(primIDs) {}

    /* Returns a mask that tells which points are valid */
    __forceinline vbool<M> valid() const { return primIDs != vint<M>(-1); }

    /* Returns if the specified point is valid */
    __forceinline bool valid(const size_t i) const { assert(i<M); return primIDs[i] != -1; }

    /* Returns the number of stored points */
    __forceinline size_t size() const { return __bsf(~movemask(valid())); }

    /* Returns the geometry IDs */
    __forceinline vint<M> geomID() const { return geomIDs; }
    __forceinline int geomID(const size_t i) const { assert(i<M); return geomIDs[i]; }

    /* Returns the primitive IDs */
    __forceinline vint<M> primID() const { return primIDs; }
    __forceinline int primID(const size_t i) const { assert(i<M); return primIDs[i]; }

    /* gather the points */
    __forceinline void gather(Vec4<vfloat<M>>& p0, const Scene* scene, size_t j = 0) const;
    __forceinline void gather(Vec4<vfloat<M>>& p0, const Scene* scene, float t) const;

    /* Calculate the bounds of the points */
    __forceinline const BBox3fa bounds(const Scene* scene, size_t j = 0) const
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<M && valid(i); i++)
      {
        const Points* geom = scene->getPoints(geomID(i));
        const Vec3fa& p0 = geom->vertex(v0[i],j);
        BBox3fa b = BBox3fa(p0);
        b = enlarge(b,Vec3fa(p0.w));
        bounds.extend(b);
      }
      return bounds;
    }

    /* Calculate the bounds of the points at t0 */
    __forceinline BBox3fa bounds0(const Scene* scene) const
    {
      return bounds(scene,0);
    }

    /* Calculate the bounds of the points at t1 */
    __forceinline BBox3fa bounds1(const Scene* scene) const
    {
      return bounds(scene,1);
    }

    /* Calculate primitive bounds */
    __forceinline std::pair<BBox3fa,BBox3fa> bounds(const Scene* scene)
    {
      return std::make_pair(bounds0(scene), bounds1(scene));
    }

    /* Fill point from point list */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      vint<M> geomID, primID;
      vint<M> v0;
      PrimRef& prim = *prims;

      for (size_t i=0; i<M; i++)
      {
        const Points* geom = scene->getPoints(prim.geomID());
        if (prims) {
          geomID[i] = prim.geomID();
          primID[i] = prim.primID();
          v0[i] = geom->point(primID);
          prims++;
        } else {
          assert(i);
          geomID[i] = geomID[i-1];
          primID[i] = -1;
          v0[i] = v0[i-1];
        }
        if (prims) prim = *prims;
      }

      new (this) PointMi(v0,geomID,primID); // FIXME: use non temporal store
    }

    /* Fill point from point list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      vint<M> geomID, primID;
      vint<M> v0;
      const PrimRef* prim = &prims[begin];

      for (size_t i=0; i<M; i++)
      {
        const Points* geom = scene->getPoints(prim->geomID());
        if (begin<end) {
          geomID[i] = prim->geomID();
          primID[i] = prim->primID();
          v0[i] = geom->point(prim->primID());
          begin++;
        } else {
          assert(i);
          geomID[i] = geomID[i-1];
          primID[i] = -1;
          v0[i] = v0[i-1];
        }
        if (begin<end) prim = &prims[begin];
      }

      new (this) PointMi(v0,geomID,primID); // FIXME: use non temporal store
    }

    /* Fill point from point list */
    __forceinline std::pair<BBox3fa,BBox3fa> fill_mblur(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      fill(prims,begin,end,scene,list);
      return bounds(scene);
    }

    /* Updates the primitive */
    __forceinline BBox3fa update(Points* geom)
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<M && valid(i); i++)
      {
        const Vec3fa& p0 = geom->vertex(v0[i]);
        BBox3fa b = BBox3fa(p0);
        b = enlarge(b,Vec3fa(p0.w));
        bounds.extend(b);
      }
      return bounds;
    }
  public:
    vint<M> v0;      // index of point vertex
    vint<M> geomIDs; // geometry ID
    vint<M> primIDs; // primitive ID
  };

  template<>
  __forceinline void PointMi<4>::gather(Vec4vf4& p0, const Scene* scene, size_t j) const
  {
    const Points* geom0 = scene->getPoints(geomIDs[0]);
    const Points* geom1 = scene->getPoints(geomIDs[1]);
    const Points* geom2 = scene->getPoints(geomIDs[2]);
    const Points* geom3 = scene->getPoints(geomIDs[3]);

    const vfloat4 a0 = vfloat4::loadu(geom0->vertexPtr(v0[0],j));
    const vfloat4 a1 = vfloat4::loadu(geom1->vertexPtr(v0[1],j));
    const vfloat4 a2 = vfloat4::loadu(geom2->vertexPtr(v0[2],j));
    const vfloat4 a3 = vfloat4::loadu(geom3->vertexPtr(v0[3],j));

    transpose(a0,a1,a2,a3,p0.x,p0.y,p0.z,p0.w);
  }

  template<>
  __forceinline void PointMi<4>::gather(Vec4vf4& p0, const Scene* scene, float t) const
  {
    const vfloat4 t0 = 1.0f - t;
    const vfloat4 t1 = t;
    Vec4vf4 a0;
    gather(a0,scene,(size_t)0);
    Vec4vf4 b0;
    gather(b0,scene,(size_t)1);
    p0 = t0 * a0 + t1 * b0;
  }

  template<int M>
  typename PointMi<M>::Type PointMi<M>::type;

  typedef PointMi<4> Point4i;
}
