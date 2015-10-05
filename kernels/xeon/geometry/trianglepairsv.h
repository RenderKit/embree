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
  /*! Stores the vertices of M triangles in struct of array layout. */

  template <int MM>
  struct TrianglePairsMv
  { 
    enum { M = MM };
    typedef vbool<M> simdb;
    typedef vfloat<M> simdf;
    typedef vint<M> simdi;
    typedef Vec3<vfloat<M>> Vec3vfM;

  public:
    struct Type : public PrimitiveType 
    {
      Type ();
      size_t size(const char* This) const;
    };
    static Type type;

  public:

    /*! returns maximal number of stored triangles */
    static __forceinline size_t max_size() { return M; }
    
     /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return (N+max_size()-1)/max_size(); }

    static __forceinline unsigned int encode_order(const unsigned int index0,
                                                   const unsigned int index1,
                                                   const unsigned int index2)
    {
      return (index0  <<  0) | (index1  <<  8) | (index2  << 16);
    }

    /* need just rotation for triangle0 */
    static __forceinline unsigned int encode_rotation_order_tri0(const unsigned int index, const bool flip = false)
    {
      const unsigned int rot[3][3] = { {1,0,2}, {0,2,1}, {2,1,0}   };
      return (rot[index][0]  <<  0) | (rot[index][1]  <<  8) | (rot[index][2]  << 16);
    }

    /* need rotation and flip for triangle1 */
    static __forceinline unsigned int encode_rotation_flip_order_tri1(const unsigned int index, const bool flip = false)
    {
      const unsigned int rot_flip[3][3]   = { {2,1,0}, {1,0,2}, {0,2,1}   };
      const unsigned int rot_noflip[3][3] = { {0,1,2}, {1,2,0}, {2,0,1}   };
      if (flip)
        return (rot_flip[index][0]  <<  0) | (rot_flip[index][1]  <<  8) | (rot_flip[index][2]  << 16);
      else
        return (rot_noflip[index][0]  <<  0) | (rot_noflip[index][1]  <<  8) | (rot_noflip[index][2]  << 16);
        
    }
   
  public:

    /*! Default constructor. */
    __forceinline TrianglePairsMv () {}

    /*! Construction from vertices and IDs. */
    __forceinline TrianglePairsMv (const Vec3vfM& v0, 
                                   const Vec3vfM& v1, 
                                   const Vec3vfM& v2, 
                                   const Vec3vfM& v3, 
                                   const vint<M>& geomIDs, 
                                   const vint<M>& primIDs,
                                   const vint<2*M>& flags)
      : v0(v0), v1(v1), v2(v2), v3(v3), geomIDs(geomIDs), primIDs(primIDs), flags(flags) {}
    
    /*! Returns a mask that tells which triangles are valid. */
    __forceinline vbool<M> valid() const { return geomIDs !=  vint<M>(-1); }

    /*! Returns true if the specified triangle is valid. */
    __forceinline bool valid(const size_t i) const { assert(i<M); return geomIDs[i] != -1; }

    /*! Returns the number of stored triangles. */
    __forceinline size_t size() const { return __bsf(~movemask(valid())); }

    /*! returns the geometry IDs */
    __forceinline vint<M> geomID() const { return geomIDs; }
    __forceinline int geomID(const size_t i) const { assert(i<M); return geomIDs[i]; }

    /*! returns the primitive IDs */
    __forceinline vint<M> primID() const { return primIDs; }
    __forceinline int  primID(const size_t i) const { assert(i<M); return primIDs[i]; }

    /*! returns the flags */
    __forceinline vint<2*M> flag() const { return flags; }
    __forceinline int  flag(const size_t i) const { assert(i<2*M); return flags[i]; }

    /*! calculate the bounds of the triangles */
    __forceinline BBox3fa bounds() const 
    {
      Vec3vfM lower = min(v0,v1,v2,v3);
      Vec3vfM upper = max(v0,v1,v2,v3);
      vbool<M> mask = valid();
      lower.x = select(mask,lower.x,vfloat<M>(pos_inf));
      lower.y = select(mask,lower.y,vfloat<M>(pos_inf));
      lower.z = select(mask,lower.z,vfloat<M>(pos_inf));
      upper.x = select(mask,upper.x,vfloat<M>(neg_inf));
      upper.y = select(mask,upper.y,vfloat<M>(neg_inf));
      upper.z = select(mask,upper.z,vfloat<M>(neg_inf));
      return BBox3fa(Vec3fa(reduce_min(lower.x),reduce_min(lower.y),reduce_min(lower.z)),
                     Vec3fa(reduce_max(upper.x),reduce_max(upper.y),reduce_max(upper.z)));
    }
    
    /*! non temporal store */
    __forceinline static void store_nt(TrianglePairsMv* dst, const TrianglePairsMv& src)
    {
      vfloat<M>::store_nt(&dst->v0.x,src.v0.x);
      vfloat<M>::store_nt(&dst->v0.y,src.v0.y);
      vfloat<M>::store_nt(&dst->v0.z,src.v0.z);
      vfloat<M>::store_nt(&dst->v1.x,src.v1.x);
      vfloat<M>::store_nt(&dst->v1.y,src.v1.y);
      vfloat<M>::store_nt(&dst->v1.z,src.v1.z);
      vfloat<M>::store_nt(&dst->v2.x,src.v2.x);
      vfloat<M>::store_nt(&dst->v2.y,src.v2.y);
      vfloat<M>::store_nt(&dst->v2.z,src.v2.z);
      vfloat<M>::store_nt(&dst->v3.x,src.v3.x);
      vfloat<M>::store_nt(&dst->v3.y,src.v3.y);
      vfloat<M>::store_nt(&dst->v3.z,src.v3.z);
      vint<M>::store_nt(&dst->geomIDs,src.geomIDs);
      vint<M>::store_nt(&dst->primIDs,src.primIDs);
      vint<2*M>::store_nt(&dst->flags,src.flags);
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      vint<M> vgeomID = -1, vprimID = -1;
      vint<2*M> vflags = 0;
      Vec3vfM v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<M && prims; i++, prims++)
      {
	const PrimRef& prim = *prims;
	const unsigned int geomId = prim.geomID() & ~((unsigned int)1 << 31); /* remove single triangle flag */
        const unsigned int primId = prim.primID();

        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomId); 

        vgeomID[i] = geomId;
        vprimID[i] = primId;
        /* single triangle, degenerate second triangle */
        if (prim.geomID() & ((unsigned int)1 << 31))
        {
          vflags[i] = (unsigned int)(1 << 31) | encode_order(0,1,2);          
          const TriangleMesh::Triangle& tri = mesh->triangle(primId);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
          v3.x[i] = p2.x; v3.y[i] = p2.y; v3.z[i] = p2.z;
        }
        else
        {
          const TriangleMesh::Triangle& tri0 = mesh->triangle(primId);
          assert(primId + 1 < mesh->size());
          const TriangleMesh::Triangle& tri1 = mesh->triangle(primId+1);
          const unsigned int order = TriangleMesh::sharedEdge(tri0,tri1);
          const unsigned int i0  = (order >>  0) & 0xff;
          const unsigned int i1  = (order >>  8) & 0xff;
          const unsigned int i2  = (order >> 16) & 0xff;
          const unsigned int opp = (order >> 24) & 0xff;
          const Vec3fa& p0 = mesh->vertex(tri0.v[i0]);
          const Vec3fa& p1 = mesh->vertex(tri0.v[i1]);
          const Vec3fa& p2 = mesh->vertex(tri0.v[i2]);
          const Vec3fa& p3 = mesh->vertex(tri1.v[opp]);
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
          v3.x[i] = p3.x; v3.y[i] = p3.y; v3.z[i] = p3.z;          
          vflags[0+i] = encode_rotation_order_tri0(i0);
          const unsigned int tri1_shared0 = (opp+1) % 3;
          const unsigned int tri1_shared1 = (opp+2) % 3;
          const bool flip = tri0.v[i0] != tri1.v[tri1_shared0];
          vflags[M+i] = encode_rotation_flip_order_tri1(opp,flip);
        }
      }
    TrianglePairsMv::store_nt(this,TrianglePairsMv(v0,v1,v2,v3,vgeomID,vprimID,vflags));
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      vint<M> vgeomID = -1, vprimID = -1;
      vint<2*M> vflags(zero);
      Vec3vfM v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
	const unsigned int geomId = prim.geomID() & ~((unsigned int)1 << 31); /* remove single triangle flag */
        const unsigned int primId = prim.primID();

        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomId); 

        vgeomID[i] = geomId;
        vprimID[i] = primId;
        /* single triangle, degenerate second triangle */
        if (prim.geomID() & ((unsigned int)1 << 31))
        {
          vflags[i] = (unsigned int)(1 << 31) | encode_order(0,1,2);
          const TriangleMesh::Triangle& tri = mesh->triangle(primId);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
          v3.x[i] = p2.x; v3.y[i] = p2.y; v3.z[i] = p2.z;
        }
        else
        {
          const TriangleMesh::Triangle& tri0 = mesh->triangle(primId);
          assert(primId + 1 < mesh->size());
          const TriangleMesh::Triangle& tri1 = mesh->triangle(primId+1);
          const unsigned int order = TriangleMesh::sharedEdge(tri0,tri1);
          const unsigned int i0  = (order >>  0) & 0xff;
          const unsigned int i1  = (order >>  8) & 0xff;
          const unsigned int i2  = (order >> 16) & 0xff;
          const unsigned int opp = (order >> 24) & 0xff;
          const Vec3fa& p0 = mesh->vertex(tri0.v[i0]);
          const Vec3fa& p1 = mesh->vertex(tri0.v[i1]);
          const Vec3fa& p2 = mesh->vertex(tri0.v[i2]);
          const Vec3fa& p3 = mesh->vertex(tri1.v[opp]);
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
          v3.x[i] = p3.x; v3.y[i] = p3.y; v3.z[i] = p3.z;          
          vflags[0+i] = encode_rotation_order_tri0(i0);
          const unsigned int tri1_shared0 = (opp+1) % 3;
          const unsigned int tri1_shared1 = (opp+2) % 3;
          assert( (tri0.v[i0] == tri1.v[tri1_shared0]) || (tri0.v[i0] == tri1.v[tri1_shared1] ));
          assert( (tri0.v[i2] == tri1.v[tri1_shared0]) || (tri0.v[i2] == tri1.v[tri1_shared1] ));
          const bool flip = tri0.v[i0] != tri1.v[tri1_shared0];
          vflags[M+i] = encode_rotation_flip_order_tri1(opp,flip);
        }
      }
    TrianglePairsMv::store_nt(this,TrianglePairsMv(v0,v1,v2,v3,vgeomID,vprimID,vflags));
    }

    /*! updates the primitive */
    __forceinline BBox3fa update(TriangleMesh* mesh)
    {
      BBox3fa bounds = empty;
      vint<M> vgeomID = -1, vprimID = -1;
      vint<2*M> vflags = 0;
      Vec3vfM v0 = zero, v1 = zero, v2 = zero;
	
      for (size_t i=0; i<M; i++)
      {
        if (primID(i) == -1) break;

        const unsigned int geomId = geomID(i);
        const unsigned int primId = primID(i);
        const unsigned int gflag  = flag(i);

        vgeomID[i] = geomId;
        vprimID[i] = primId;
        vflags[i]  = gflag;

        /* single triangle, degenerate second triangle */
        if (flag(i) & ((unsigned int)1 << 31))
        {
          const TriangleMesh::Triangle& tri = mesh->triangle(primId);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2)));
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
          v3.x[i] = p2.x; v3.y[i] = p2.y; v3.z[i] = p2.z;
        }
        else
        {
          const TriangleMesh::Triangle& tri0 = mesh->triangle(primId);
          assert(primId + 1 < mesh->size());
          const TriangleMesh::Triangle& tri1 = mesh->triangle(primId+1);

          const unsigned int order = TriangleMesh::sharedEdge(tri0,tri1); // FIXME:: could use encoded indices
          const unsigned int i0  = (order >>  0) & 0xff;
          const unsigned int i1  = (order >>  8) & 0xff;
          const unsigned int i2  = (order >> 16) & 0xff;
          const unsigned int opp = (order >> 24) & 0xff;

          const Vec3fa& p0 = mesh->vertex(tri0.v[i0]);
          const Vec3fa& p1 = mesh->vertex(tri0.v[i1]);
          const Vec3fa& p2 = mesh->vertex(tri0.v[i2]);
          const Vec3fa& p3 = mesh->vertex(tri1.v[opp]);
          bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2),BBox3fa(p3)));
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
          v3.x[i] = p3.x; v3.y[i] = p3.y; v3.z[i] = p3.z;
        }
      }
      new (this) TrianglePairsMv(v0,v1,v2,v3,vgeomID,vprimID);
      return bounds;
    }
   
  public:
    Vec3vfM v0;        //!< 1st vertex of the triangles
    Vec3vfM v1;        //!< 2nd vertex of the triangles
    Vec3vfM v2;        //!< 3rd vertex of the triangles.
    Vec3vfM v3;        //!< 4rd vertex of the triangles.
    vint<M> geomIDs;   //!< geometry ID
    vint<M> primIDs;   //!< primitive ID
    vint<2*M> flags;   //!< flags
  };

  template<int MM>
  typename TrianglePairsMv<MM>::Type TrianglePairsMv<MM>::type;

  typedef TrianglePairsMv<4> TrianglePairs4v;
}
