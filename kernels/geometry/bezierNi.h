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
#include "bezier1i.h"

namespace embree
{
  struct BezierNi
  {
    enum { M = 8 };
    
    struct Type : public PrimitiveType {
      Type ();
      size_t size(const char* This) const;
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
      return blocks(N)*sizeof(BezierNi);
#elif EMBREE_HAIR_LEAF_MODE == 1
      const size_t f = N/M, r = N%M;
      return f*sizeof(BezierNi) + (r!=0)*(65 + 14*r);
#elif EMBREE_HAIR_LEAF_MODE == 2
      const size_t f = N/M, r = N%M;
      return f*sizeof(BezierNi) + (r!=0)*(25 + 29*r);
#endif
    }

  public:

    /*! Default constructor. */
    __forceinline BezierNi () {}

#if 1
    const LinearSpace3fa computeAlignedSpace(Scene* scene, const PrimRef* prims, const range<size_t>& set)
    {
      Vec3fa axis(0,0,1);
      uint64_t bestGeomPrimID = -1;
      
      /*! find curve with minimum ID that defines valid direction */
      for (size_t i=set.begin(); i<set.end(); i++)
      {
        const unsigned int geomID = prims[i].geomID();
        const unsigned int primID = prims[i].primID();
        const uint64_t geomprimID = prims[i].ID64();
        if (geomprimID >= bestGeomPrimID) continue;
        NativeCurves* mesh = (NativeCurves*) scene->get(geomID);
        const unsigned vtxID = mesh->curve(primID);
        const Vec3fa v0 = mesh->vertex(vtxID+0);
        const Vec3fa v1 = mesh->vertex(vtxID+1);
        const Vec3fa v2 = mesh->vertex(vtxID+2);
        const Vec3fa v3 = mesh->vertex(vtxID+3);
        const Curve3fa curve(v0,v1,v2,v3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa axis1 = normalize(p3 - p0);
        if (sqr_length(p3-p0) > 1E-18f) {
          axis = axis1;
          bestGeomPrimID = geomprimID;
        }
      }
      return frame(axis).transposed();
    }

    const LinearSpace3fa computeAlignedSpace(Scene* scene, const PrimRef* prims, const range<size_t>& set, const Vec3fa& offset, const Vec3fa& scale)
    {
      Vec3fa axis(0,0,1);
      uint64_t bestGeomPrimID = -1;
      
      /*! find curve with minimum ID that defines valid direction */
      for (size_t i=set.begin(); i<set.end(); i++)
      {
        const unsigned int geomID = prims[i].geomID();
        const unsigned int primID = prims[i].primID();
        const uint64_t geomprimID = prims[i].ID64();
        if (geomprimID >= bestGeomPrimID) continue;
        NativeCurves* mesh = (NativeCurves*) scene->get(geomID);
        const unsigned vtxID = mesh->curve(primID);
        const Vec3fa v0 = (mesh->vertex(vtxID+0)-offset)*scale;
        const Vec3fa v1 = (mesh->vertex(vtxID+1)-offset)*scale;
        const Vec3fa v2 = (mesh->vertex(vtxID+2)-offset)*scale;
        const Vec3fa v3 = (mesh->vertex(vtxID+3)-offset)*scale;
        const Curve3fa curve(v0,v1,v2,v3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa axis1 = normalize(p3 - p0);
        if (sqr_length(p3-p0) > 1E-18f) {
          axis = axis1;
          bestGeomPrimID = geomprimID;
        }
      }
      return frame(axis);
    }
#endif

#if 0

    const LinearSpace3fa computeAlignedSpace(Scene* scene, const PrimRef* prims, const range<size_t>& set)
    {
      Vec3fa axis(0,0,0);
      
      /*! find curve with minimum ID that defines valid direction */
      for (size_t i=set.begin(); i<set.end(); i++)
      {
        const unsigned int geomID = prims[i].geomID();
        const unsigned int primID = prims[i].primID();
        NativeCurves* mesh = (NativeCurves*) scene->get(geomID);
        const unsigned vtxID = mesh->curve(primID);
        const Vec3fa v0 = mesh->vertex(vtxID+0);
        const Vec3fa v1 = mesh->vertex(vtxID+1);
        const Vec3fa v2 = mesh->vertex(vtxID+2);
        const Vec3fa v3 = mesh->vertex(vtxID+3);
        const Curve3fa curve(v0,v1,v2,v3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa axis1 = normalize(p3 - p0);
        axis += axis1;
      }
      axis = normalize(axis);
      
      return frame(axis).transposed();
    }
    
#endif
    
    /*! fill curve from curve list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t _end, Scene* scene)
    {
#if EMBREE_HAIR_LEAF_MODE == 0
      
      size_t end = min(begin+M,_end);
      N = end-begin;

      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
        const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        AffineSpace3fa space = computeAlignedSpace(scene,prims,range<size_t>(begin));
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(space,primID);
                
        space.p -= bounds.lower;
        space = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19f),bounds.upper-bounds.lower))*space;

        naabb.l.vx.x[i] = space.l.vx.x;
        naabb.l.vx.y[i] = space.l.vx.y;
        naabb.l.vx.z[i] = space.l.vx.z;

        naabb.l.vy.x[i] = space.l.vy.x;
        naabb.l.vy.y[i] = space.l.vy.y;
        naabb.l.vy.z[i] = space.l.vy.z;

        naabb.l.vz.x[i] = space.l.vz.x;
        naabb.l.vz.y[i] = space.l.vz.y;
        naabb.l.vz.z[i] = space.l.vz.z;

        naabb.p.x[i] = space.p.x;
        naabb.p.y[i] = space.p.y;
        naabb.p.z[i] = space.p.z;

        this->geomID[i] = geomID;
        this->primID[i] = primID;
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

      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID();
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
        this->geomID(N)[i] = geomID;
        this->primID(N)[i] = primID;
      }
#endif

#if EMBREE_HAIR_LEAF_MODE == 2
      
      size_t end = min(begin+M,_end);
      N = end-begin;

      /* encode all primitives */
      BBox3fa bounds = empty;
      for (size_t i=0; i<N; i++)
      {
        const PrimRef& prim = prims[begin+i];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        bounds.extend(scene->get<NativeCurves>(geomID)->bounds(primID));
      }

      /* calculate offset and scale */
      Vec3fa loffset = bounds.lower;
      Vec3fa lscale = 256.0f/(bounds.size()*sqrt(3.0f));
      lscale = Vec3fa(min(lscale.x,lscale.y,lscale.z));
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
               
        this->geomID(N)[i] = geomID;
        this->primID(N)[i] = primID;
      }
        
#endif
    }

    template<typename BVH, typename Allocator>
      __forceinline static typename BVH::NodeRef createLeaf (BVH* bvh, const PrimRef* prims, const range<size_t>& set, const Allocator& alloc)
    {
      size_t start = set.begin();
      size_t items = BezierNi::blocks(set.size());
      size_t numbytes = BezierNi::bytes(set.size());
      BezierNi* accel = (BezierNi*) alloc.malloc1(numbytes,BVH::byteAlignment);
      for (size_t i=0; i<items; i++) {
        accel[i].fill(prims,start,set.end(),bvh->scene);
      }
      return bvh->encodeLeaf((char*)accel,items);
    };
    
  public:
#if EMBREE_HAIR_LEAF_MODE == 0

    // 56 bytes
    AffineSpace3vf<M> naabb;
    unsigned int N;
    unsigned int geomID[M];
    unsigned int primID[M];
#endif

#if EMBREE_HAIR_LEAF_MODE == 1
    // 20 bytes
    AffineSpace3fa space;
    unsigned char N;
    unsigned char _lower_x[M];
    unsigned char _upper_x[M];
    unsigned char _lower_y[M];
    unsigned char _upper_y[M];
    unsigned char _lower_z[M];
    unsigned char _upper_z[M];
    unsigned int _geomID[M];
    unsigned int _primID[M];

    __forceinline       unsigned char* lower_x(size_t N)       { return (unsigned char*)((char*)this+65+0*N); }
    __forceinline const unsigned char* lower_x(size_t N) const { return (unsigned char*)((char*)this+65+0*N); }
    
    __forceinline       unsigned char* upper_x(size_t N)       { return (unsigned char*)((char*)this+65+1*N); }
    __forceinline const unsigned char* upper_x(size_t N) const { return (unsigned char*)((char*)this+65+1*N); }
    
    __forceinline       unsigned char* lower_y(size_t N)       { return (unsigned char*)((char*)this+65+2*N); }
    __forceinline const unsigned char* lower_y(size_t N) const { return (unsigned char*)((char*)this+65+2*N); }
    
    __forceinline       unsigned char* upper_y(size_t N)       { return (unsigned char*)((char*)this+65+3*N); }
    __forceinline const unsigned char* upper_y(size_t N) const { return (unsigned char*)((char*)this+65+3*N); }
    
    __forceinline       unsigned char* lower_z(size_t N)       { return (unsigned char*)((char*)this+65+4*N); }
    __forceinline const unsigned char* lower_z(size_t N) const { return (unsigned char*)((char*)this+65+4*N); }
    
    __forceinline       unsigned char* upper_z(size_t N)       { return (unsigned char*)((char*)this+65+5*N); }
    __forceinline const unsigned char* upper_z(size_t N) const { return (unsigned char*)((char*)this+65+5*N); }
    
    __forceinline       unsigned int* geomID  (size_t N)       { return (unsigned int* )((char*)this+65+6*N); }
    __forceinline const unsigned int* geomID  (size_t N) const { return (unsigned int* )((char*)this+65+6*N); }
    
    __forceinline       unsigned int* primID  (size_t N)       { return (unsigned int* )((char*)this+65+10*N); }
    __forceinline const unsigned int* primID  (size_t N) const { return (unsigned int* )((char*)this+65+10*N); }
    
    
#endif

#if EMBREE_HAIR_LEAF_MODE == 2
    
    // 32.1 bytes per primitive
    char N;
        
    unsigned int _geomID[M];
    unsigned int _primID[M];
    
    char _bounds_vx_x[M];
    char _bounds_vx_y[M];
    char _bounds_vx_z[M];
    short _bounds_vx_lower[M];
    short _bounds_vx_upper[M];
    
    char _bounds_vy_x[M];
    char _bounds_vy_y[M];
    char _bounds_vy_z[M];
    short _bounds_vy_lower[M];
    short _bounds_vy_upper[M];
    
    char _bounds_vz_x[M];
    char _bounds_vz_y[M];
    char _bounds_vz_z[M];
    short _bounds_vz_lower[M];
    short _bounds_vz_upper[M];

    Vec3f _offset;
    Vec3f _scale;
    
    __forceinline       unsigned int* geomID(size_t N)       { return (unsigned int*)((char*)this+1); }
    __forceinline const unsigned int* geomID(size_t N) const { return (unsigned int*)((char*)this+1); }
    
    __forceinline       unsigned int* primID(size_t N)       { return (unsigned int*)((char*)this+1+4*N); }
    __forceinline const unsigned int* primID(size_t N) const { return (unsigned int*)((char*)this+1+4*N); }
    
    __forceinline       char* bounds_vx_x(size_t N)       { return (char*)((char*)this+1+8*N); }
    __forceinline const char* bounds_vx_x(size_t N) const { return (char*)((char*)this+1+8*N); }
    
    __forceinline       char* bounds_vx_y(size_t N)       { return (char*)((char*)this+1+9*N); }
    __forceinline const char* bounds_vx_y(size_t N) const { return (char*)((char*)this+1+9*N); }
    
    __forceinline       char* bounds_vx_z(size_t N)       { return (char*)((char*)this+1+10*N); }
    __forceinline const char* bounds_vx_z(size_t N) const { return (char*)((char*)this+1+10*N); }
    
    __forceinline       short* bounds_vx_lower(size_t N)       { return (short*)((char*)this+1+11*N); }
    __forceinline const short* bounds_vx_lower(size_t N) const { return (short*)((char*)this+1+11*N); }
    
    __forceinline       short* bounds_vx_upper(size_t N)       { return (short*)((char*)this+1+13*N); }
    __forceinline const short* bounds_vx_upper(size_t N) const { return (short*)((char*)this+1+13*N); }
    
    __forceinline       char* bounds_vy_x(size_t N)       { return (char*)((char*)this+1+15*N); }
    __forceinline const char* bounds_vy_x(size_t N) const { return (char*)((char*)this+1+15*N); }
    
    __forceinline       char* bounds_vy_y(size_t N)       { return (char*)((char*)this+1+16*N); }
    __forceinline const char* bounds_vy_y(size_t N) const { return (char*)((char*)this+1+16*N); }
    
    __forceinline       char* bounds_vy_z(size_t N)       { return (char*)((char*)this+1+17*N); }
    __forceinline const char* bounds_vy_z(size_t N) const { return (char*)((char*)this+1+17*N); }
    
    __forceinline       short* bounds_vy_lower(size_t N)       { return (short*)((char*)this+1+18*N); }
    __forceinline const short* bounds_vy_lower(size_t N) const { return (short*)((char*)this+1+18*N); }
    
    __forceinline       short* bounds_vy_upper(size_t N)       { return (short*)((char*)this+1+20*N); }
    __forceinline const short* bounds_vy_upper(size_t N) const { return (short*)((char*)this+1+20*N); }
    
    __forceinline       char* bounds_vz_x(size_t N)       { return (char*)((char*)this+1+22*N); }
    __forceinline const char* bounds_vz_x(size_t N) const { return (char*)((char*)this+1+22*N); }
    
    __forceinline       char* bounds_vz_y(size_t N)       { return (char*)((char*)this+1+23*N); }
    __forceinline const char* bounds_vz_y(size_t N) const { return (char*)((char*)this+1+23*N); }
    
    __forceinline       char* bounds_vz_z(size_t N)       { return (char*)((char*)this+1+24*N); }
    __forceinline const char* bounds_vz_z(size_t N) const { return (char*)((char*)this+1+24*N); }
    
    __forceinline       short* bounds_vz_lower(size_t N)       { return (short*)((char*)this+1+25*N); }
    __forceinline const short* bounds_vz_lower(size_t N) const { return (short*)((char*)this+1+25*N); }
    
    __forceinline       short* bounds_vz_upper(size_t N)       { return (short*)((char*)this+1+27*N); }
    __forceinline const short* bounds_vz_upper(size_t N) const { return (short*)((char*)this+1+27*N); }
    
    __forceinline       Vec3f* offset(size_t N)       { return (Vec3f*)((char*)this+1+29*N); }
    __forceinline const Vec3f* offset(size_t N) const { return (Vec3f*)((char*)this+1+29*N); }
    
    __forceinline       Vec3f* scale(size_t N)       { return (Vec3f*)((char*)this+1+29*N+12); }
    __forceinline const Vec3f* scale(size_t N) const { return (Vec3f*)((char*)this+1+29*N+12); }
    
    
#endif
  };
}
