// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
#include "../common/scene.h"

namespace embree
{
  /* Stores M motion blur triangles from an indexed face set */
  template <int M>
  struct TriangleMiMB
  {
    typedef Vec3<vfloat<M>> Vec3vfM;

    /* Virtual interface to query information about the triangle type */
    struct Type : public PrimitiveType
    {
      Type();
      size_t size(const char* This) const;
    };
    static Type type;

  public:

    /* Returns maximal number of stored triangles */
    static __forceinline size_t max_size() { return M; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return (N+max_size()-1)/max_size(); }

  public:

    /* Default constructor */
    __forceinline TriangleMiMB() {  }

    /* Construction from vertices and IDs */
    __forceinline TriangleMiMB(const vint<M>& v0,
                               const vint<M>& v1,
                               const vint<M>& v2,
                               const vint<M>& geomIDs,
                               const vint<M>& primIDs)
      : v0(v0), v1(v1), v2(v2), geomIDs(geomIDs), primIDs(primIDs)
    {
    }

    /* Returns a mask that tells which triangles are valid */
    __forceinline vbool<M> valid() const { return primIDs != vint<M>(-1); }

    /* Returns if the specified triangle is valid */
    __forceinline bool valid(const size_t i) const { assert(i<M); return primIDs[i] != -1; }

    /* Returns the number of stored triangles */
    __forceinline size_t size() const { return __bsf(~movemask(valid())); }

    /* Returns the geometry IDs */
    __forceinline vint<M> geomID() const { return geomIDs; }
    __forceinline int geomID(const size_t i) const { assert(i<M); assert(geomIDs[i] != -1); return geomIDs[i]; }

    /* Returns the primitive IDs */
    __forceinline vint<M> primID() const { return primIDs; }
    __forceinline int primID(const size_t i) const { assert(i<M); return primIDs[i]; }

     __forceinline Vec3fa& getVertex(const vint<M> &v, const size_t index, const Scene *const scene) const
    {
      const TriangleMesh* mesh = scene->getTriangleMesh(geomID(index));
      return *(Vec3fa*)mesh->vertexPtr(v[index]);
    }

     template<typename T>
     __forceinline Vec3<T> getVertex(const vint<M> &v, const size_t index, const Scene *const scene, const T& time) const
    {
      const TriangleMesh* mesh = scene->getTriangleMesh(geomID(index));
      const Vec3fa v0 = *(Vec3fa*)mesh->vertexPtr(v[index],0);
      const Vec3fa v1 = *(Vec3fa*)mesh->vertexPtr(v[index],1);
      const Vec3<T> p0(v0.x,v0.y,v0.z);
      const Vec3<T> p1(v1.x,v1.y,v1.z);
      return (T(one)-time)*p0 + time*p1;
    }

    /* gather the triangles */
    __forceinline void gather(Vec3<vfloat<M>>& p0,
                              Vec3<vfloat<M>>& p1,
                              Vec3<vfloat<M>>& p2,
                              const Scene *const scene,
                              const size_t j) const;

    __forceinline void gather(Vec3<vfloat<M>>& p0,
                              Vec3<vfloat<M>>& p1,
                              Vec3<vfloat<M>>& p2,
                              const Scene *const scene,
                              const float ftime,
                              const int itime) const;

    /* Calculate the bounds of the triangles */
    __forceinline const BBox3fa bounds(const Scene *const scene, const size_t j=0) const
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<M && valid(i); i++)
      {
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID(i));
        const Vec3fa &p0 = mesh->vertex(v0[i],j);
        const Vec3fa &p1 = mesh->vertex(v1[i],j);
        const Vec3fa &p2 = mesh->vertex(v2[i],j);
        bounds.extend(p0);
        bounds.extend(p1);
        bounds.extend(p2);
      }
      return bounds;
    }

    /* Calculate the bounds of the triangles at t0 */
    __forceinline BBox3fa bounds0(const Scene *const scene, size_t time) const
    {
      return bounds(scene,time+0);
    }

    /* Calculate the bounds of the triangles at t1 */
    __forceinline BBox3fa bounds1(const Scene *const scene, size_t time) const
    {
      return bounds(scene,time+1);
    }

    /* Calculate primitive bounds */
    __forceinline std::pair<BBox3fa,BBox3fa> bounds(const Scene *const scene, size_t time) {
      return std::make_pair(bounds0(scene,time),bounds1(scene,time));
    }


    /* Fill triangle from triangle list */
    __forceinline std::pair<BBox3fa,BBox3fa> fill_mblur(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list, size_t time)
    {
      vint<M> geomID = -1, primID = -1;
      vint<M> v0 = zero, v1 = zero, v2 = zero;
      const PrimRef* prim = &prims[begin];

      for (size_t i=0; i<M; i++)
      {
        const TriangleMesh* mesh = scene->getTriangleMesh(prim->geomID());
        const TriangleMesh::Triangle& tri = mesh->triangle(prim->primID());
        if (begin<end) {
          geomID[i] = prim->geomID();
          primID[i] = prim->primID();
          v0[i] = tri.v[0];
          v1[i] = tri.v[1];
          v2[i] = tri.v[2];
          begin++;
        } else {
          assert(i);
          geomID[i] = geomID[0]; // always valid geomIDs
          primID[i] = -1;        // indicates invalid data
          v0[i] = 0;
          v1[i] = 0;
          v2[i] = 0;
        }
        if (begin<end) prim = &prims[begin];
      }

      new (this) TriangleMiMB(v0,v1,v2,geomID,primID); // FIXME: use non temporal store
      return bounds(scene,time);
    }

    /* Updates the primitive */
    __forceinline BBox3fa update(TriangleMesh* mesh)
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<M; i++)
      {
        if (!valid(i)) break;
        const unsigned primId = primID(i);
        const TriangleMesh::Triangle& tri = mesh->triangle(primId);
        const Vec3fa p0 = mesh->vertex(tri.v[0]);
        const Vec3fa p1 = mesh->vertex(tri.v[1]);
        const Vec3fa p2 = mesh->vertex(tri.v[2]);
        bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2)));
      }
      return bounds;
    }

  public:
    vint<M> v0;         // index of 1st vertex
    vint<M> v1;         // index of 2nd vertex
    vint<M> v2;         // index of 3rd vertex
    vint<M> geomIDs;    // geometry ID of mesh
    vint<M> primIDs;    // primitive ID of primitive inside mesh
  };

  template<>
    __forceinline void TriangleMiMB<4>::gather(Vec3vf4& p0,
                                               Vec3vf4& p1,
                                               Vec3vf4& p2,
                                               const Scene *const scene,
                                               const size_t j) const
  {
    const TriangleMesh* mesh0 = scene->getTriangleMesh(geomIDs[0]);
    const TriangleMesh* mesh1 = scene->getTriangleMesh(geomIDs[1]);
    const TriangleMesh* mesh2 = scene->getTriangleMesh(geomIDs[2]);
    const TriangleMesh* mesh3 = scene->getTriangleMesh(geomIDs[3]);

    const vfloat4 a0 = vfloat4::loadu(mesh0->vertexPtr(v0[0],j));
    const vfloat4 a1 = vfloat4::loadu(mesh1->vertexPtr(v0[1],j));
    const vfloat4 a2 = vfloat4::loadu(mesh2->vertexPtr(v0[2],j));
    const vfloat4 a3 = vfloat4::loadu(mesh3->vertexPtr(v0[3],j));

    transpose(a0,a1,a2,a3,p0.x,p0.y,p0.z);

    const vfloat4 b0 = vfloat4::loadu(mesh0->vertexPtr(v1[0],j));
    const vfloat4 b1 = vfloat4::loadu(mesh1->vertexPtr(v1[1],j));
    const vfloat4 b2 = vfloat4::loadu(mesh2->vertexPtr(v1[2],j));
    const vfloat4 b3 = vfloat4::loadu(mesh3->vertexPtr(v1[3],j));

    transpose(b0,b1,b2,b3,p1.x,p1.y,p1.z);

    const vfloat4 c0 = vfloat4::loadu(mesh0->vertexPtr(v2[0],j));
    const vfloat4 c1 = vfloat4::loadu(mesh1->vertexPtr(v2[1],j));
    const vfloat4 c2 = vfloat4::loadu(mesh2->vertexPtr(v2[2],j));
    const vfloat4 c3 = vfloat4::loadu(mesh3->vertexPtr(v2[3],j));

    transpose(c0,c1,c2,c3,p2.x,p2.y,p2.z);
  }



  template<>
    __forceinline void TriangleMiMB<4>::gather(Vec3vf4& p0,
                                               Vec3vf4& p1,
                                               Vec3vf4& p2,
                                               const Scene *const scene,
                                               const float ftime,
                                               const int itime) const
  {
    const vfloat4 t0 = 1.0f - ftime;
    const vfloat4 t1 = ftime;
    Vec3vf4 a0,a1,a2;
    gather(a0,a1,a2,scene,(size_t)itime+0);
    Vec3vf4 b0,b1,b2;
    gather(b0,b1,b2,scene,(size_t)itime+1);
    p0 = t0 * a0 + t1 * b0;
    p1 = t0 * a1 + t1 * b1;
    p2 = t0 * a2 + t1 * b2;
  }

  template<int M>
  typename TriangleMiMB<M>::Type TriangleMiMB<M>::type;

  typedef TriangleMiMB<4> Triangle4iMB;
}
