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
#include "bezierNi.h"

namespace embree
{
  struct BezierNv : public BezierNi
  {
    enum { M = 8 };
    
    struct Type : public PrimitiveType {
      Type ();
      size_t size(const char* This) const;
      size_t getBytes(const char* This) const;
    };
    static Type type;

  public:

    /* Returns maximum number of stored primitives */
    static __forceinline size_t max_size() { return M; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return (N+M-1)/M; }

    static __forceinline size_t bytes(size_t N)
    {
#if EMBREE_HAIR_LEAF_MODE == 0
      const size_t f = N/M, r = N%M;
      static_assert(sizeof(BezierNv) == 5+13*4*M+4*16*M, "internal data layout issue");
      return f*sizeof(BezierNv) + (r!=0)*(5 + 13*4*r + 4*16*r);
#elif EMBREE_HAIR_LEAF_MODE == 1
      const size_t f = N/M, r = N%M;
      static_assert(sizeof(BezierNv) == 56+10*M+4*16*M, "internal data layout issue");
      return f*sizeof(BezierNv) + (r!=0)*(56 + 10*r + 4*16*r);
#elif EMBREE_HAIR_LEAF_MODE == 2
      const size_t f = N/M, r = N%M;
      static_assert(sizeof(BezierNv) == 21+25*M+4*16*M, "internal data layout issue");
      return f*sizeof(BezierNv) + (r!=0)*(21 + 25*r + 4*16*r);
#endif
    }

  public:

    /*! Default constructor. */
    __forceinline BezierNv () {}

    /*! fill curve from curve list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t _end, Scene* scene)
    {  
#if EMBREE_HAIR_LEAF_MODE == 0
      
      size_t end = min(begin+M,_end);
      N = end-begin;
      const unsigned int geomID0 = prims[begin].geomID();
      this->geomID(N) = geomID0;

      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
        const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID(); assert(geomID == geomID0);
        const unsigned int primID = prim.primID();
        AffineSpace3fa space = computeAlignedSpace(scene,prims,range<size_t>(begin));
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(space,primID);
                
        space.p -= bounds.lower;
        space = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19f),bounds.upper-bounds.lower))*space;

        vx_x(N)[i] = space.l.vx.x;
        vx_y(N)[i] = space.l.vx.y;
        vx_z(N)[i] = space.l.vx.z;

        vy_x(N)[i] = space.l.vy.x;
        vy_y(N)[i] = space.l.vy.y;
        vy_z(N)[i] = space.l.vy.z;

        vz_x(N)[i] = space.l.vz.x;
        vz_y(N)[i] = space.l.vz.y;
        vz_z(N)[i] = space.l.vz.z;

        p_x(N)[i] = space.p.x;
        p_y(N)[i] = space.p.y;
        p_z(N)[i] = space.p.z;

        this->primID(N)[i] = primID;

        NativeCurves* mesh = (NativeCurves*) scene->get(geomID);
        const unsigned vtxID = mesh->curve(primID);
        Vec3fa::storeu(&this->vertices(i,N)[0],mesh->vertex(vtxID+0));
        Vec3fa::storeu(&this->vertices(i,N)[1],mesh->vertex(vtxID+1));
        Vec3fa::storeu(&this->vertices(i,N)[2],mesh->vertex(vtxID+2));
        Vec3fa::storeu(&this->vertices(i,N)[3],mesh->vertex(vtxID+3));
      }
        
#endif

#if EMBREE_HAIR_LEAF_MODE == 1 

      /* find aligned space */
      size_t end = min(begin+M,_end);
      LinearSpace3fa s = computeAlignedSpace(scene,prims,range<size_t>(begin,end));

      /* calculate leaf gbounds for this space */
      BBox3fa gbounds = empty;
      for (size_t j=begin; j<end; j++) {
        gbounds.extend(scene->get<NativeCurves>(prims[j].geomID())->bounds(s,prims[j].primID()));
      }

      /* normalize space for encoding */
      const Vec3fa bs = gbounds.size();
      AffineSpace3fa a(255.0f*s.vx/bs,255.0f*s.vy/bs,255.0f*s.vz/bs,-255.0f*gbounds.lower/bs);
      space = AffineSpace3fa(a);
      N = end-begin;
      const unsigned int geomID0 = prims[begin].geomID();
      this->geomID(N) = geomID0;

      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID(); assert(geomID == geomID0);
        const unsigned int primID = prim.primID();
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(s,primID);
        const Vec3fa lower = 255.0f*(bounds.lower-gbounds.lower)/gbounds.size();
        const Vec3fa upper = 255.0f*(bounds.upper-gbounds.lower)/gbounds.size();
        
        this->lower_x(N)[i] = (unsigned char) clamp(floor(lower.x),0.0f,255.0f);
        this->upper_x(N)[i] = (unsigned char) clamp(ceil (upper.x),0.0f,255.0f);
        this->lower_y(N)[i] = (unsigned char) clamp(floor(lower.y),0.0f,255.0f);
        this->upper_y(N)[i] = (unsigned char) clamp(ceil (upper.y),0.0f,255.0f);
        this->lower_z(N)[i] = (unsigned char) clamp(floor(lower.z),0.0f,255.0f);
        this->upper_z(N)[i] = (unsigned char) clamp(ceil (upper.z),0.0f,255.0f);
        this->primID(N)[i] = primID;

        NativeCurves* mesh = (NativeCurves*) scene->get(geomID);
        const unsigned vtxID = mesh->curve(primID);
        Vec3fa::storeu(&this->vertices(i,N)[0],mesh->vertex(vtxID+0));
        Vec3fa::storeu(&this->vertices(i,N)[1],mesh->vertex(vtxID+1));
        Vec3fa::storeu(&this->vertices(i,N)[2],mesh->vertex(vtxID+2));
        Vec3fa::storeu(&this->vertices(i,N)[3],mesh->vertex(vtxID+3));
      }
#endif

#if EMBREE_HAIR_LEAF_MODE == 2
      
      size_t end = min(begin+M,_end);
      N = end-begin;
      const unsigned int geomID0 = prims[begin].geomID();
      this->geomID(N) = geomID0;

      /* encode all primitives */
      BBox3fa bounds = empty;
      for (size_t i=0; i<N; i++)
      {
        const PrimRef& prim = prims[begin+i];
        const unsigned int geomID = prim.geomID(); assert(geomID == geomID0);
        const unsigned int primID = prim.primID();
        bounds.extend(scene->get<NativeCurves>(geomID)->bounds(primID));
      }

      /* calculate offset and scale */
      Vec3fa loffset = bounds.lower;
      float lscale = reduce_min(256.0f/(bounds.size()*sqrt(3.0f)));
      *this->offset(N) = loffset;
      *this->scale(N) = lscale;
      
      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
        const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        const LinearSpace3fa space2 = computeAlignedSpace(scene,prims,range<size_t>(begin),loffset,lscale);
        
        const LinearSpace3fa space3(trunc(126.0f*space2.vx),trunc(126.0f*space2.vy),trunc(126.0f*space2.vz));
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(loffset,lscale,max(length(space3.vx),length(space3.vy),length(space3.vz)),space3.transposed(),primID);
        
        bounds_vx_x(N)[i] = (short) space3.vx.x;
        bounds_vx_y(N)[i] = (short) space3.vx.y;
        bounds_vx_z(N)[i] = (short) space3.vx.z;
        bounds_vx_lower(N)[i] = (short) clamp(floor(bounds.lower.x),-32767.0f,32767.0f);
        bounds_vx_upper(N)[i] = (short) clamp(ceil (bounds.upper.x),-32767.0f,32767.0f);

        bounds_vy_x(N)[i] = (short) space3.vy.x;
        bounds_vy_y(N)[i] = (short) space3.vy.y;
        bounds_vy_z(N)[i] = (short) space3.vy.z;
        bounds_vy_lower(N)[i] = (short) clamp(floor(bounds.lower.y),-32767.0f,32767.0f);
        bounds_vy_upper(N)[i] = (short) clamp(ceil (bounds.upper.y),-32767.0f,32767.0f);

        bounds_vz_x(N)[i] = (short) space3.vz.x;
        bounds_vz_y(N)[i] = (short) space3.vz.y;
        bounds_vz_z(N)[i] = (short) space3.vz.z;
        bounds_vz_lower(N)[i] = (short) clamp(floor(bounds.lower.z),-32767.0f,32767.0f);
        bounds_vz_upper(N)[i] = (short) clamp(ceil (bounds.upper.z),-32767.0f,32767.0f);
               
        this->primID(N)[i] = primID;

        NativeCurves* mesh = (NativeCurves*) scene->get(geomID);
        const unsigned vtxID = mesh->curve(primID);
        Vec3fa::storeu(&this->vertices(i,N)[0],mesh->vertex(vtxID+0));
        Vec3fa::storeu(&this->vertices(i,N)[1],mesh->vertex(vtxID+1));
        Vec3fa::storeu(&this->vertices(i,N)[2],mesh->vertex(vtxID+2));
        Vec3fa::storeu(&this->vertices(i,N)[3],mesh->vertex(vtxID+3));
      }
        
#endif

    }

    template<typename BVH, typename Allocator>
      __forceinline static typename BVH::NodeRef createLeaf (BVH* bvh, const PrimRef* prims, const range<size_t>& set, const Allocator& alloc)
    {
      size_t start = set.begin();
      size_t items = BezierNv::blocks(set.size());
      size_t numbytes = BezierNv::bytes(set.size());
      BezierNv* accel = (BezierNv*) alloc.malloc1(numbytes,BVH::byteAlignment);
      for (size_t i=0; i<items; i++) {
        accel[i].fill(prims,start,set.end(),bvh->scene);
      }
      return bvh->encodeLeaf((char*)accel,items);
    };
    
  public:
    unsigned char data[4*16*M];
    __forceinline       Vec3fa* vertices(size_t i, size_t N)       { return (Vec3fa*)BezierNi::end(N)+4*i; }
    __forceinline const Vec3fa* vertices(size_t i, size_t N) const { return (Vec3fa*)BezierNi::end(N)+4*i; }
  };
}
