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
  /* Stores M triangles from an indexed face set */
  template <int MM>
  struct TriangleMi
  {
    enum { M = MM };
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
    __forceinline TriangleMi() {}

    /* Construction from vertices and IDs */
    __forceinline TriangleMi(Vec3f* base[M], const vint<M>& v1, const vint<M>& v2, const vint<M>& geomIDs, const vint<M>& primIDs)
      : v1(v1), v2(v2), geomIDs(geomIDs), primIDs(primIDs) 
    {
      for (size_t i=0; i<M; i++)
        v0[i] = base[i];
    }

    /* Returns a mask that tells which triangles are valid */
    __forceinline vbool<M> valid() const { return primIDs != vint<M>(-1); }
    
    /* Returns if the specified triangle is valid */
    __forceinline bool valid(const size_t i) const { assert(i<M); return geomIDs[i] != -1; }
    
    /* Returns the number of stored triangles */
    __forceinline size_t size() const { return __bsf(~movemask(valid())); }
    
    /* Returns the geometry IDs */
    __forceinline vint<M> geomID() const { return geomIDs; }
    __forceinline int geomID(const size_t i) const { assert(i<M); return geomIDs[i]; }
    
    /* Returns the primitive IDs */
    __forceinline vint<M> primID() const { return primIDs; }
    __forceinline int primID(const size_t i) const { assert(i<M); return primIDs[i]; }
    
    /* Calculate the bounds of the triangles */
    __forceinline const BBox3fa bounds() const 
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<M && geomIDs[i] != -1; i++)
      {
	const int* base = (int*) v0[i];
	const Vec3fa p0 = *(Vec3fa*)(base);
	const Vec3fa p1 = *(Vec3fa*)(base+v1[i]);
	const Vec3fa p2 = *(Vec3fa*)(base+v2[i]);
	bounds.extend(p0);
	bounds.extend(p1);
	bounds.extend(p2);
      }
      return bounds;
    }
    
    /* Fill triangle from triangle list */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      vint<M> geomID = -1, primID = -1;
      Vec3f* v0[M];
      vint<M> v1 = zero, v2 = zero;
      PrimRef& prim = *prims;
      
      for (size_t i=0; i<M; i++)
      {
	const TriangleMesh* mesh = scene->getTriangleMesh(prim.geomID());
	const TriangleMesh::Triangle& tri = mesh->triangle(prim.primID());
	if (prims) {
	  geomID[i] = prim.geomID();
	  primID[i] = prim.primID();
	  v0[i] = (Vec3f*) mesh->vertexPtr(tri.v[0]); 
	  v1[i] = (int*)   mesh->vertexPtr(tri.v[1])-(int*)v0[i]; 
	  v2[i] = (int*)   mesh->vertexPtr(tri.v[2])-(int*)v0[i]; 
	  prims++;
	} else {
	  assert(i);
	  geomID[i] = -1;
	  primID[i] = -1;
	  v0[i] = v0[i-1];
	  v1[i] = 0; 
	  v2[i] = 0;
	}
	if (prims) prim = *prims; 
      }
      
      new (this) TriangleMi(v0,v1,v2,geomID,primID); // FIXME: use non temporal store
    }
    
    /* Fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      vint<M> geomID = -1, primID = -1;
      Vec3f* v0[M];
      vint<M> v1 = zero, v2 = zero;
      const PrimRef* prim = &prims[begin];
      
      for (size_t i=0; i<M; i++)
      {
	const TriangleMesh* mesh = scene->getTriangleMesh(prim->geomID());
	const TriangleMesh::Triangle& tri = mesh->triangle(prim->primID());
	if (begin<end) {
	  geomID[i] = prim->geomID();
	  primID[i] = prim->primID();
	  v0[i] = (Vec3f*) mesh->vertexPtr(tri.v[0]); 
	  v1[i] = (int*)   mesh->vertexPtr(tri.v[1])-(int*)v0[i]; 
	  v2[i] = (int*)   mesh->vertexPtr(tri.v[2])-(int*)v0[i]; 
	  begin++;
	} else {
	  assert(i);
	  geomID[i] = -1;
	  primID[i] = -1;
	  v0[i] = v0[i-1];
	  v1[i] = 0; 
	  v2[i] = 0;
	}
	if (begin<end) prim = &prims[begin];
      }
      
      new (this) TriangleMi(v0,v1,v2,geomID,primID); // FIXME: use non temporal store
    }
    
    /* Updates the primitive */
    __forceinline BBox3fa update(TriangleMesh* mesh)
    {
      BBox3fa bounds = empty;
      vint<M> vgeomID = -1, vprimID = -1;
      Vec3vfM v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<M; i++)
      {
        if (primID(i) == -1) break;
        const unsigned geomId = geomID(i);
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
    const Vec3f* v0[M]; // pointer to 1st vertex
    vint<M> v1;         // offset to 2nd vertex
    vint<M> v2;         // offset to 3rd vertex
    vint<M> geomIDs;    // geometry ID of mesh
    vint<M> primIDs;    // primitive ID of primitive inside mesh
  };

  template<int MM>
  typename TriangleMi<MM>::Type TriangleMi<MM>::type;

  typedef TriangleMi<4> Triangle4i;
}
