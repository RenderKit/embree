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
#include "../../common/scene.h"

namespace embree
{
  /* Stores M quads from an indexed face set */
  template <int M>
  struct QuadMiMB
  {
    typedef Vec3<vfloat<M>> Vec3vfM;

    /* Virtual interface to query information about the quad type */
    struct Type : public PrimitiveType
    {
      Type();
      size_t size(const char* This) const;
    };
    static Type type;

  public:

    /* Returns maximal number of stored quads */
    static __forceinline size_t max_size() { return M; }
    
    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return (N+max_size()-1)/max_size(); }
  
  public:

    /* Default constructor */
    __forceinline QuadMiMB() {  }

    /* Construction from vertices and IDs */
    __forceinline QuadMiMB(const vint<M>& v0, 
                           const vint<M>& v1, 
                           const vint<M>& v2, 
                           const vint<M>& v3, 
                           const vint<M>& geomIDs, 
                           const vint<M>& primIDs)
      : v0(v0),v1(v1), v2(v2), v3(v3), geomIDs(geomIDs), primIDs(primIDs) 
    {
    }

    /* Returns a mask that tells which quads are valid */
    __forceinline vbool<M> valid() const { return primIDs != vint<M>(-1); }
    
    /* Returns if the specified quad is valid */
    __forceinline bool valid(const size_t i) const { assert(i<M); return primIDs[i] != -1; }
    
    /* Returns the number of stored quads */
    __forceinline size_t size() const { return __bsf(~movemask(valid())); }
    
    /* Returns the geometry IDs */
    __forceinline vint<M> geomID() const { return geomIDs; }
    __forceinline int geomID(const size_t i) const { assert(i<M); assert(geomIDs[i] != -1); return geomIDs[i]; }
    
    /* Returns the primitive IDs */
    __forceinline vint<M> primID() const { return primIDs; }
    __forceinline int primID(const size_t i) const { assert(i<M); return primIDs[i]; }

     __forceinline Vec3fa& getVertex(const vint<M> &v, const size_t index, const Scene *const scene) const
    {
      const QuadMesh* mesh = scene->getQuadMesh(geomID(index));
      return *(Vec3fa*)mesh->vertexPtr(v[index]);
    }

     template<typename T>
     __forceinline Vec3<T> getVertex(const vint<M> &v, const size_t index, const Scene *const scene, const T& time) const
    {
      const QuadMesh* mesh = scene->getQuadMesh(geomID(index));
      const Vec3fa v0  = *(Vec3fa*)mesh->vertexPtr(v[index],0);
      const Vec3fa v1  = *(Vec3fa*)mesh->vertexPtr(v[index],1);
      const Vec3<T> p0(v0.x,v0.y,v0.z);
      const Vec3<T> p1(v1.x,v1.y,v1.z);
      return (T(one)-time)*p0 + time*p1;
    }

    /* gather the quads */
    __forceinline void gather(Vec3<vfloat<M>>& p0, 
                              Vec3<vfloat<M>>& p1, 
                              Vec3<vfloat<M>>& p2, 
                              Vec3<vfloat<M>>& p3,
                              const Scene *const scene,
                              const size_t j) const;

    __forceinline void gather(Vec3<vfloat<M>>& p0, 
                              Vec3<vfloat<M>>& p1, 
                              Vec3<vfloat<M>>& p2, 
                              Vec3<vfloat<M>>& p3,
                              const Scene *const scene,
                              const float time) const;
    
    /* Calculate the bounds of the quads */
    __forceinline const BBox3fa bounds(const Scene *const scene, const size_t j=0) const 
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<M && valid(i); i++)
      {
	const QuadMesh* mesh = scene->getQuadMesh(geomID(i));
	const Vec3fa &p0 = mesh->vertex(v0[i],j);
	const Vec3fa &p1 = mesh->vertex(v1[i],j);
	const Vec3fa &p2 = mesh->vertex(v2[i],j);
	const Vec3fa &p3 = mesh->vertex(v3[i],j);
	bounds.extend(p0);
	bounds.extend(p1);
	bounds.extend(p2);
	bounds.extend(p3);
      }
      return bounds;
    }
    

    /* Calculate the bounds of the triangles at t0 */
    __forceinline BBox3fa bounds0(const Scene *const scene) const 
    {
      return bounds(scene,0);
    }

    /* Calculate the bounds of the triangles at t1 */
    __forceinline BBox3fa bounds1(const Scene *const scene) const 
    {
      return bounds(scene,1);
    }

    /* Calculate primitive bounds */
    __forceinline std::pair<BBox3fa,BBox3fa> bounds(const Scene *const scene) {
      return std::make_pair(bounds0(scene),bounds1(scene));
    }

    
    /* Fill quad from quad list */
    __forceinline std::pair<BBox3fa,BBox3fa> fill_mblur(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      vint<M> geomID = -1, primID = -1;
      vint<M> v0 = zero, v1 = zero, v2 = zero, v3 = zero;
      const PrimRef* prim = &prims[begin];
      
      for (size_t i=0; i<M; i++)
      {
	const QuadMesh* mesh = scene->getQuadMesh(prim->geomID());
	const QuadMesh::Quad& q = mesh->quad(prim->primID());
	if (begin<end) {
	  geomID[i] = prim->geomID();
	  primID[i] = prim->primID();
	  v0[i] = q.v[0]; 
	  v1[i] = q.v[1]; 
	  v2[i] = q.v[2]; 
	  v3[i] = q.v[3]; 
	  begin++;
	} else {
	  assert(i);
	  geomID[i] = geomID[0]; // always valid geomIDs
	  primID[i] = -1;        // indicates invalid data
	  v0[i] = 0;
	  v1[i] = 0; 
	  v2[i] = 0;
	  v3[i] = 0;
	}
	if (begin<end) prim = &prims[begin];
      }
      
      new (this) QuadMiMB(v0,v1,v2,v3,geomID,primID); // FIXME: use non temporal store
      return bounds(scene);
    }
    
    /* Updates the primitive */
    __forceinline BBox3fa update(QuadMesh* mesh)
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<M; i++)
      {
        if (!valid(i)) break;
        const unsigned geomId = geomID(i);
        const unsigned primId = primID(i);
        const QuadMesh::Quad& q = mesh->quad(primId);
        const Vec3fa p0 = mesh->vertex(q.v[0]);
        const Vec3fa p1 = mesh->vertex(q.v[1]);
        const Vec3fa p2 = mesh->vertex(q.v[2]);
        const Vec3fa p3 = mesh->vertex(q.v[3]);
        bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2),BBox3fa(p3)));
      }
      return bounds;
    }
    
  public:
    vint<M> v0;         // offset to 2nd vertex
    vint<M> v1;         // offset to 2nd vertex
    vint<M> v2;         // offset to 3rd vertex
    vint<M> v3;         // offset to 4rd vertex
    vint<M> geomIDs;    // geometry ID of mesh
    vint<M> primIDs;    // primitive ID of primitive inside mesh
  };

  template<>
    __forceinline void QuadMiMB<4>::gather(Vec3vf4& p0, 
                                           Vec3vf4& p1, 
                                           Vec3vf4& p2, 
                                           Vec3vf4& p3,
                                           const Scene *const scene,
                                           const size_t j) const
  {
    const QuadMesh* mesh0 = scene->getQuadMesh(geomIDs[0]);
    const QuadMesh* mesh1 = scene->getQuadMesh(geomIDs[1]);
    const QuadMesh* mesh2 = scene->getQuadMesh(geomIDs[2]);
    const QuadMesh* mesh3 = scene->getQuadMesh(geomIDs[3]);

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

    const vfloat4 d0 = vfloat4::loadu(mesh0->vertexPtr(v3[0],j));
    const vfloat4 d1 = vfloat4::loadu(mesh1->vertexPtr(v3[1],j));
    const vfloat4 d2 = vfloat4::loadu(mesh2->vertexPtr(v3[2],j));
    const vfloat4 d3 = vfloat4::loadu(mesh3->vertexPtr(v3[3],j));

    transpose(d0,d1,d2,d3,p3.x,p3.y,p3.z);
  }



  template<>
    __forceinline void QuadMiMB<4>::gather(Vec3vf4& p0, 
                                           Vec3vf4& p1, 
                                           Vec3vf4& p2, 
                                           Vec3vf4& p3,
                                           const Scene *const scene,
                                           const float t) const
  {
    const vfloat4 t0 = 1.0f - t;
    const vfloat4 t1 = t;
    Vec3vf4 a0,a1,a2,a3;
    gather(a0,a1,a2,a3,scene,(size_t)0);
    Vec3vf4 b0,b1,b2,b3;
    gather(b0,b1,b2,b3,scene,(size_t)1);
    p0 = t0 * a0 + t1 * b0;
    p1 = t0 * a1 + t1 * b1;
    p2 = t0 * a2 + t1 * b2;
    p3 = t0 * a3 + t1 * b3;
  }

  template<int M>
  typename QuadMiMB<M>::Type QuadMiMB<M>::type;

  typedef QuadMiMB<4> Quad4iMB;
}
