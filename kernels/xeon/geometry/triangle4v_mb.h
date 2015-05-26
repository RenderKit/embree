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
  /*! Stores the vertices of 4 triangles in struct of array layout. */
  struct Triangle4vMB
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
    __forceinline Triangle4vMB () {}

    /*! Construction from vertices and IDs. */
    __forceinline Triangle4vMB (const Vec3f4& a0, const Vec3f4& a1, 
				const Vec3f4& b0, const Vec3f4& b1,
				const Vec3f4& c0, const Vec3f4& c1, 
				const int4& geomIDs, const int4& primIDs)
      : v0(a0), v1(b0), v2(c0), dv0(a1-a0), dv1(b1-b0), dv2(c1-c0), geomIDs(geomIDs), primIDs(primIDs) {}

     /*! Returns a mask that tells which triangles are valid. */
    __forceinline bool4 valid() const { return geomIDs != int4(-1); }

    /*! Returns if the specified triangle is valid. */
    __forceinline bool valid(const size_t i) const { assert(i<4); return geomIDs[i] != -1; }

    /*! Returns the number of stored triangles. */
    __forceinline size_t size() const { return __bsf(~movemask(valid())); }

    /*! returns the geometry IDs */
    __forceinline int4 geomID() const { return geomIDs; }
    __forceinline int geomID(const size_t i) const { assert(i<4); return geomIDs[i]; }

    /*! returns the primitive IDs */
    __forceinline int4 primID() const { return primIDs; }
    __forceinline int  primID(const size_t i) const { assert(i<4); return primIDs[i]; }

    /*! calculate the bounds of the triangles at t0 */
    __forceinline BBox3fa bounds0() const 
    {
      Vec3f4 lower = min(v0,v1,v2);
      Vec3f4 upper = max(v0,v1,v2);
      const bool4 mask = valid();
      lower.x = select(mask,lower.x,float4(pos_inf));
      lower.y = select(mask,lower.y,float4(pos_inf));
      lower.z = select(mask,lower.z,float4(pos_inf));
      upper.x = select(mask,upper.x,float4(neg_inf));
      upper.y = select(mask,upper.y,float4(neg_inf));
      upper.z = select(mask,upper.z,float4(neg_inf));
      return BBox3fa(Vec3fa(reduce_min(lower.x),reduce_min(lower.y),reduce_min(lower.z)),
		     Vec3fa(reduce_max(upper.x),reduce_max(upper.y),reduce_max(upper.z)));
    }

    /*! calculate the bounds of the triangles at t1 */
    __forceinline BBox3fa bounds1() const 
    {
      const Vec3f4 p0 = v0+dv0;
      const Vec3f4 p1 = v1+dv1;
      const Vec3f4 p2 = v2+dv2;
      Vec3f4 lower = min(p0,p1,p2);
      Vec3f4 upper = max(p0,p1,p2);
      const bool4 mask = valid();
      lower.x = select(mask,lower.x,float4(pos_inf));
      lower.y = select(mask,lower.y,float4(pos_inf));
      lower.z = select(mask,lower.z,float4(pos_inf));
      upper.x = select(mask,upper.x,float4(neg_inf));
      upper.y = select(mask,upper.y,float4(neg_inf));
      upper.z = select(mask,upper.z,float4(neg_inf));
      return BBox3fa(Vec3fa(reduce_min(lower.x),reduce_min(lower.y),reduce_min(lower.z)),
		     Vec3fa(reduce_max(upper.x),reduce_max(upper.y),reduce_max(upper.z)));
    }

    /*! calculate primitive bounds */
    __forceinline std::pair<BBox3fa,BBox3fa> bounds() {
      return std::make_pair(bounds0(),bounds1());
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      int4 vgeomID = -1, vprimID = -1;
      Vec3f4 va0 = zero, vb0 = zero, vc0 = zero;
      Vec3f4 va1 = zero, vb1 = zero, vc1 = zero;
      
      for (size_t i=0; i<4 && prims; i++, prims++)
      {
	const PrimRef& prim = *prims;
	const size_t geomID = prim.geomID();
        const size_t primID = prim.primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& a0 = mesh->vertex(tri.v[0],0);
	const Vec3fa& a1 = mesh->vertex(tri.v[0],1);
        const Vec3fa& b0 = mesh->vertex(tri.v[1],0);
	const Vec3fa& b1 = mesh->vertex(tri.v[1],1);
        const Vec3fa& c0 = mesh->vertex(tri.v[2],0);
	const Vec3fa& c1 = mesh->vertex(tri.v[2],1);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        va0.x[i] = a0.x; va0.y[i] = a0.y; va0.z[i] = a0.z;
	va1.x[i] = a1.x; va1.y[i] = a1.y; va1.z[i] = a1.z;
	vb0.x[i] = b0.x; vb0.y[i] = b0.y; vb0.z[i] = b0.z;
	vb1.x[i] = b1.x; vb1.y[i] = b1.y; vb1.z[i] = b1.z;
	vc0.x[i] = c0.x; vc0.y[i] = c0.y; vc0.z[i] = c0.z;
	vc1.x[i] = c1.x; vc1.y[i] = c1.y; vc1.z[i] = c1.z;
      }
      new (this) Triangle4vMB(va0,va1,vb0,vb1,vc0,vc1,vgeomID,vprimID); // FIXME: store_nt
    }
    
    /*! fill triangle from triangle list */
    __forceinline std::pair<BBox3fa,BBox3fa> fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      int4 vgeomID = -1, vprimID = -1;
      Vec3f4 va0 = zero, vb0 = zero, vc0 = zero;
      Vec3f4 va1 = zero, vb1 = zero, vc1 = zero;

      BBox3fa bounds0 = empty;
      BBox3fa bounds1 = empty;
      
      for (size_t i=0; i<4 && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
        const size_t geomID = prim.geomID();
        const size_t primID = prim.primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
	const Vec3fa& a0 = mesh->vertex(tri.v[0],0); bounds0.extend(a0);
	const Vec3fa& a1 = mesh->vertex(tri.v[0],1); bounds1.extend(a1);
        const Vec3fa& b0 = mesh->vertex(tri.v[1],0); bounds0.extend(b0);
	const Vec3fa& b1 = mesh->vertex(tri.v[1],1); bounds1.extend(b1);
        const Vec3fa& c0 = mesh->vertex(tri.v[2],0); bounds0.extend(c0);
	const Vec3fa& c1 = mesh->vertex(tri.v[2],1); bounds1.extend(c1);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        va0.x[i] = a0.x; va0.y[i] = a0.y; va0.z[i] = a0.z;
	va1.x[i] = a1.x; va1.y[i] = a1.y; va1.z[i] = a1.z;
	vb0.x[i] = b0.x; vb0.y[i] = b0.y; vb0.z[i] = b0.z;
	vb1.x[i] = b1.x; vb1.y[i] = b1.y; vb1.z[i] = b1.z;
	vc0.x[i] = c0.x; vc0.y[i] = c0.y; vc0.z[i] = c0.z;
	vc1.x[i] = c1.x; vc1.y[i] = c1.y; vc1.z[i] = c1.z;
      }
      new (this) Triangle4vMB(va0,va1,vb0,vb1,vc0,vc1,vgeomID,vprimID);
      return std::make_pair(bounds0,bounds1);
    }
   
  public:
    Vec3f4 v0;      //!< 1st vertex of the triangles.
    Vec3f4 v1;      //!< 2nd vertex of the triangles.
    Vec3f4 v2;      //!< 3rd vertex of the triangles.
    Vec3f4 dv0;      //!< difference vector between time steps t0 and t1 for first vertex
    Vec3f4 dv1;      //!< difference vector between time steps t0 and t1 for second vertex
    Vec3f4 dv2;      //!< difference vector between time steps t0 and t1 for third vertex
    int4 geomIDs;  //!< geometry ID
    int4 primIDs;  //!< primitive ID
  };
}
