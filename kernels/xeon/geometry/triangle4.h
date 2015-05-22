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
  /*! Precalculated representation for 4 triangles. Stores for each
      triangle a base vertex, two edges, and the geometry normal to
      speed up intersection calculations. */
  struct Triangle4
  {
    typedef bool4 simdb;
    typedef float4 simdf;
    typedef int4 simdi;

  public:
    struct Type : public PrimitiveType 
    {
      Type ();
      size_t size(const char* This) const;
    };
    static Type type;
    
  public:

    /*! returns maximal number of stored triangles */
    static __forceinline size_t max_size() { return 4; }
    
     /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return (N+max_size()-1)/max_size(); }

  public:

    /*! Default constructor. */
    __forceinline Triangle4 () {}

    /*! Construction from vertices and IDs. */
    __forceinline Triangle4 (const Vec3f4& v0, const Vec3f4& v1, const Vec3f4& v2, const int4& geomIDs, const int4& primIDs)
      : v0(v0), e1(v0-v1), e2(v2-v0), Ng(cross(e1,e2)), geomIDs(geomIDs), primIDs(primIDs) {}

    /*! Returns a mask that tells which triangles are valid. */
    __forceinline bool4 valid() const { return geomIDs != int4(-1); }

    /*! Returns true if the specified triangle is valid. */
    __forceinline bool valid(const size_t i) const { assert(i<4); return geomIDs[i] != -1; }
    
    /*! Returns the number of stored triangles. */
    __forceinline size_t size() const { return __bsf(~movemask(valid()));  }

    /*! returns the geometry IDs */
    __forceinline int4 geomID() const { return geomIDs;  }
    __forceinline int geomID(const size_t i) const { assert(i<4); return geomIDs[i]; }

    /*! returns the primitive IDs */
    __forceinline int4 primID() const { return primIDs; }
    __forceinline int  primID(const size_t i) const { assert(i<4); return primIDs[i]; }

    /*! calculate the bounds of the triangle */
    __forceinline BBox3fa bounds() const 
    {
      Vec3f4 p0 = v0;
      Vec3f4 p1 = v0-e1;
      Vec3f4 p2 = v0+e2;
      Vec3f4 lower = min(p0,p1,p2);
      Vec3f4 upper = max(p0,p1,p2);
      bool4 mask = valid();
      lower.x = select(mask,lower.x,float4(pos_inf));
      lower.y = select(mask,lower.y,float4(pos_inf));
      lower.z = select(mask,lower.z,float4(pos_inf));
      upper.x = select(mask,upper.x,float4(neg_inf));
      upper.y = select(mask,upper.y,float4(neg_inf));
      upper.z = select(mask,upper.z,float4(neg_inf));
      return BBox3fa(Vec3fa(reduce_min(lower.x),reduce_min(lower.y),reduce_min(lower.z)),
                     Vec3fa(reduce_max(upper.x),reduce_max(upper.y),reduce_max(upper.z)));
    }

    /*! non temporal store */
    __forceinline static void store_nt(Triangle4* dst, const Triangle4& src)
    {
      store4f_nt(&dst->v0.x,src.v0.x);
      store4f_nt(&dst->v0.y,src.v0.y);
      store4f_nt(&dst->v0.z,src.v0.z);
      store4f_nt(&dst->e1.x,src.e1.x);
      store4f_nt(&dst->e1.y,src.e1.y);
      store4f_nt(&dst->e1.z,src.e1.z);
      store4f_nt(&dst->e2.x,src.e2.x);
      store4f_nt(&dst->e2.y,src.e2.y);
      store4f_nt(&dst->e2.z,src.e2.z);
      store4f_nt(&dst->Ng.x,src.Ng.x);
      store4f_nt(&dst->Ng.y,src.Ng.y);
      store4f_nt(&dst->Ng.z,src.Ng.z);
      store4i_nt(&dst->geomIDs,src.geomIDs);
      store4i_nt(&dst->primIDs,src.primIDs);
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      int4 vgeomID = -1, vprimID = -1;
      Vec3f4 v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<4 && prims; i++, prims++)
      {
	const PrimRef& prim = *prims;
	const size_t geomID = prim.geomID();
        const size_t primID = prim.primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(this,Triangle4(v0,v1,v2,vgeomID,vprimID));
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      int4 vgeomID = -1, vprimID = -1;
      Vec3f4 v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<4 && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
        const size_t geomID = prim.geomID();
        const size_t primID = prim.primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(this,Triangle4(v0,v1,v2,vgeomID,vprimID));
    }

    /*! updates the primitive */
    __forceinline BBox3fa update(TriangleMesh* mesh)
    {
      BBox3fa bounds = empty;
      int4 vgeomID = -1, vprimID = -1;
      Vec3f4 v0 = zero, v1 = zero, v2 = zero;
	
      for (size_t i=0; i<4; i++)
      {
        if (geomID(i) == -1) break;
        const unsigned geomId = geomID(i);
        const unsigned primId = primID(i);
        const TriangleMesh::Triangle& tri = mesh->triangle(primId);
        const Vec3fa p0 = mesh->vertex(tri.v[0]);
        const Vec3fa p1 = mesh->vertex(tri.v[1]);
        const Vec3fa p2 = mesh->vertex(tri.v[2]);
        bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2)));
        vgeomID [i] = geomId;
        vprimID [i] = primId;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(this,Triangle4(v0,v1,v2,vgeomID,vprimID));
      return bounds;
    }

  public:
    Vec3f4 v0;      //!< Base vertex of the triangles
    Vec3f4 e1;      //!< 1st edge of the triangles (v0-v1)
    Vec3f4 e2;      //!< 2nd edge of the triangles (v2-v0)
    Vec3f4 Ng;      //!< Geometry normal of the triangles (cross(e1,e2))
    int4 geomIDs;  //!< geometry IDs
    int4 primIDs;  //!< primitive IDs
  };
}
