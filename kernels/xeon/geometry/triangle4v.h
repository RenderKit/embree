// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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
  /*! Stores the vertices of 4 triangles in struct of array layout. */
  struct Triangle4v
  {
  public:

    /*! Default constructor. */
    __forceinline Triangle4v () {}

    /*! Construction from vertices and IDs. */
    __forceinline Triangle4v (const sse3f& v0, const sse3f& v1, const sse3f& v2, const ssei& geomID, const ssei& primID, const ssei& mask)
      : v0(v0), v1(v1), v2(v2), geomID(geomID), primID(primID)
    {
#if defined(__USE_RAY_MASK__)
      this->mask = mask;
#endif
    }

    /*! Returns if the specified triangle is valid. */
    __forceinline bool valid(const size_t i) const { 
      assert(i<4); 
      return geomID[i] != -1; 
    }

    /*! Returns a mask that tells which triangles are valid. */
    __forceinline sseb valid() const { return geomID != ssei(-1); }

    /*! Returns the number of stored triangles. */
    __forceinline size_t size() const {
      return bitscan(~movemask(valid()));
    }

    /*! calculate the bounds of the triangle */
    __forceinline BBox3fa bounds() const 
    {
      sse3f lower = min(v0,v1,v2);
      sse3f upper = max(v0,v1,v2);
      sseb mask = valid();
      lower.x = select(mask,lower.x,ssef(pos_inf));
      lower.y = select(mask,lower.y,ssef(pos_inf));
      lower.z = select(mask,lower.z,ssef(pos_inf));
      upper.x = select(mask,upper.x,ssef(neg_inf));
      upper.y = select(mask,upper.y,ssef(neg_inf));
      upper.z = select(mask,upper.z,ssef(neg_inf));
      return BBox3fa(Vec3fa(reduce_min(lower.x),reduce_min(lower.y),reduce_min(lower.z)),
                    Vec3fa(reduce_max(upper.x),reduce_max(upper.y),reduce_max(upper.z)));
    }

    /*! non temporal store */
    __forceinline static void store_nt(Triangle4v* dst, const Triangle4v& src)
    {
      store4f_nt(&dst->v0.x,src.v0.x);
      store4f_nt(&dst->v0.y,src.v0.y);
      store4f_nt(&dst->v0.z,src.v0.z);
      store4f_nt(&dst->v1.x,src.v1.x);
      store4f_nt(&dst->v1.y,src.v1.y);
      store4f_nt(&dst->v1.z,src.v1.z);
      store4f_nt(&dst->v2.x,src.v2.x);
      store4f_nt(&dst->v2.y,src.v2.y);
      store4f_nt(&dst->v2.z,src.v2.z);
      store4i_nt(&dst->geomID,src.geomID);
      store4i_nt(&dst->primID,src.primID);
#if defined(__USE_RAY_MASK__)
      store4i_nt(&dst->mask,src.mask);
#endif
    }

    /*! fill triangle from triangle list */
    static __forceinline void fill(Triangle4v* This, atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene)
    {
      ssei geomID = -1, primID = -1, mask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<4 && prims; i++, prims++)
      {
	const PrimRef& prim = *prims;
	const TriangleMesh* mesh = scene->getTriangleMesh(prim.geomID());
	const TriangleMesh::Triangle& tri = mesh->triangle(prim.primID());
	const Vec3fa& p0 = mesh->vertex(tri.v[0]);
	const Vec3fa& p1 = mesh->vertex(tri.v[1]);
	const Vec3fa& p2 = mesh->vertex(tri.v[2]);
	geomID [i] = prim.geomID();
	primID [i] = prim.primID();
	mask   [i] = mesh->mask;
	v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
	v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
	v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      new (This) Triangle4v(v0,v1,v2,geomID,primID,mask);
    }
   
  public:
    sse3f v0;      //!< 1st vertex of the triangles.
    sse3f v1;      //!< 2nd vertex of the triangles.
    sse3f v2;      //!< 3rd vertex of the triangles.
    ssei geomID;   //!< user geometry ID
    ssei primID;   //!< primitive ID
#if defined(__USE_RAY_MASK__)
    ssei mask;     //!< geometry mask
#endif
  };

  struct Triangle4vType : public PrimitiveType {
    Triangle4vType ();
    size_t blocks(size_t x) const;
    size_t size(const char* This) const;
  };

  struct SceneTriangle4v : public Triangle4vType
  {
    static SceneTriangle4v type;
    void pack(char* This, atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, void* geom) const;
    void pack(char* dst, const PrimRef* prims, size_t num, void* geom) const;
    BBox3fa update(char* prim, size_t num, void* geom) const;
  };

  struct TriangleMeshTriangle4v : public Triangle4vType
  {
    static TriangleMeshTriangle4v type;
    void pack(char* This, atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, void* geom) const;
    BBox3fa update(char* prim, size_t num, void* geom) const;
  };
}
