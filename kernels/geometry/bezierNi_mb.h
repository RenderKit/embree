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
  struct BezierNiMB
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
      static_assert(sizeof(BezierNiMB) == 5+13*4*M, "internal data layout issue");
      return f*sizeof(BezierNiMB) + (r!=0)*(5+13*4*r + 4*max(0,8-3*(int)N));
#elif EMBREE_HAIR_LEAF_MODE == 1
      const size_t f = N/M, r = N%M;
      static_assert(sizeof(BezierNiMB) == 56+10*M, "internal data layout issue");
      return f*sizeof(BezierNiMB) + (r!=0)*(56 + 10*r);
#elif EMBREE_HAIR_LEAF_MODE == 2
      const size_t f = N/M, r = N%M;
      static_assert(sizeof(BezierNiMB) == 21+25*M, "internal data layout issue");
      return f*sizeof(BezierNiMB) + (r!=0)*(21 + 25*r);
#endif
    }

  public:

    /*! Default constructor. */
    __forceinline BezierNiMB () {}

    const LinearSpace3fa computeAlignedSpaceMB(Scene* scene, const PrimRefMB* prims, const range<size_t>& object_range,
                                               const Vec3fa& offset, const float scale, // offset and scale is not required here!?
                                               const BBox1f time_range)
    {
      Vec3fa axis0(0,0,1);
      uint64_t bestGeomPrimID = -1;
      
      /*! find curve with minimum ID that defines valid direction */
      for (size_t i=object_range.begin(); i<object_range.end(); i++)
      {
        const PrimRefMB& prim = prims[i];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        const uint64_t geomprimID = prim.ID64();
        if (geomprimID >= bestGeomPrimID) continue;
        
        const NativeCurves* mesh = scene->get<NativeCurves>(geomID);
        const unsigned num_time_segments = mesh->numTimeSegments();
        const range<int> tbounds = getTimeSegmentRange(time_range, (float)num_time_segments);
        if (tbounds.size() == 0) continue;
        
        const size_t t = (tbounds.begin()+tbounds.end())/2;
        const unsigned int vertexID = mesh->curve(primID);
        const Vec3fa a0 = (mesh->vertex(vertexID+0,t)-ofs)*scale;
        const Vec3fa a1 = (mesh->vertex(vertexID+1,t)-ofs)*scale;
        const Vec3fa a2 = (mesh->vertex(vertexID+2,t)-ofs)*scale;
        const Vec3fa a3 = (mesh->vertex(vertexID+3,t)-ofs)*scale;
        const Curve3fa curve(a0,a1,a2,a3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa axis1 = normalize(p3 - p0);
        
        if (sqr_length(axis1) > 1E-18f) {
          axis0 = normalize(axis1);
          bestGeomPrimID = geomprimID;
        }
      }
      
      return frame(axis0);
    }

    /*! fill curve from curve list */
    __forceinline LBBox3fa fillMB(const PrimRefMB* prims, size_t& i, size_t end, Scene* scene, const BBox1f time_range)
    {
      size_t end = min(begin+M,_end);
      N = end-begin;
      const unsigned int geomID0 = prims[begin].geomID();
      this->geomID(N) = geomID0;

      /* encode all primitives */
      LBBox3fa lbounds = empty;
      for (size_t i=0; i<N; i++)
      {
        const PrimRef& prim = prims[begin+i];
        const unsigned int geomID = prim.geomID(); assert(geomID == geomID0);
        const unsigned int primID = prim.primID();
        lbounds.extend(scene->get<NativeCurves>(geomID)->linearBounds(primID,time_range));
      }
      LBBox3fa bounds = lbounds.bounds();

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
        const LinearSpace3fa space2 = computeAlignedSpace(scene,prims,range<size_t>(begin),loffset,lscale,time_range);
        
        const LinearSpace3fa space3(trunc(126.0f*space2.vx),trunc(126.0f*space2.vy),trunc(126.0f*space2.vz));
        const LBBox3fa bounds = scene->get<NativeCurves>(geomID)->linearBounds(loffset,lscale,max(length(space3.vx),length(space3.vy),length(space3.vz)),space3.transposed(),primID,time_range);
        
        bounds_vx_x(N)[i] = (short) space3.vx.x;
        bounds_vx_y(N)[i] = (short) space3.vx.y;
        bounds_vx_z(N)[i] = (short) space3.vx.z;
        bounds_vx_lower0(N)[i] = (short) clamp(floor(bounds.bounds0.lower.x),-32767.0f,32767.0f);
        bounds_vx_upper0(N)[i] = (short) clamp(ceil (bounds.bounds0.upper.x),-32767.0f,32767.0f);
        bounds_vx_lower1(N)[i] = (short) clamp(floor(bounds.bounds1.lower.x),-32767.0f,32767.0f);
        bounds_vx_upper1(N)[i] = (short) clamp(ceil (bounds.bounds1.upper.x),-32767.0f,32767.0f);

        bounds_vy_x(N)[i] = (short) space3.vy.x;
        bounds_vy_y(N)[i] = (short) space3.vy.y;
        bounds_vy_z(N)[i] = (short) space3.vy.z;
        bounds_vy_lower0(N)[i] = (short) clamp(floor(bounds.bounds0.lower.y),-32767.0f,32767.0f);
        bounds_vy_upper0(N)[i] = (short) clamp(ceil (bounds.bounds0.upper.y),-32767.0f,32767.0f);
        bounds_vy_lower1(N)[i] = (short) clamp(floor(bounds.bounds1.lower.y),-32767.0f,32767.0f);
        bounds_vy_upper1(N)[i] = (short) clamp(ceil (bounds.bounds1.upper.y),-32767.0f,32767.0f);

        bounds_vz_x(N)[i] = (short) space3.vz.x;
        bounds_vz_y(N)[i] = (short) space3.vz.y;
        bounds_vz_z(N)[i] = (short) space3.vz.z;
        bounds_vz_lower0(N)[i] = (short) clamp(floor(bounds.bounds0.lower.z),-32767.0f,32767.0f);
        bounds_vz_upper0(N)[i] = (short) clamp(ceil (bounds.bounds0.upper.z),-32767.0f,32767.0f);
        bounds_vz_lower1(N)[i] = (short) clamp(floor(bounds.bounds1.lower.z),-32767.0f,32767.0f);
        bounds_vz_upper1(N)[i] = (short) clamp(ceil (bounds.bounds1.upper.z),-32767.0f,32767.0f);
               
        this->primID(N)[i] = primID;
      }
      
      const PrimRefMB& prim = prims[i];
      i++;
      const unsigned geomID = prim.geomID();
      const unsigned primID = prim.primID();
      const NativeCurves* curves = scene->get<NativeCurves>(geomID);
      const unsigned vertexID = curves->curve(primID);
      /* encode the RTCCurveFlags into the two most significant bits */
      const unsigned int mask = curves->getStartEndBitMask(primID);
      new (this) Bezier1i(vertexID,geomID | mask,primID);
      return curves->linearBounds(primID,time_range);
    }

    template<typename BVH, typename SetMB, typename Allocator>
    __forceinline static typename BVH::NodeRecordMB4D createLeafMB(BVH* bvh, const SetMB& prims, const Allocator& alloc)
    {
      size_t start = prims.object_range.begin();
      size_t end   = prims.object_range.end();
      size_t items = prims.object_range.size();
      Bezier1i* accel = (Bezier1i*) alloc.malloc1(items*sizeof(Bezier1i));
      const typename BVH::NodeRef node = bvh->encodeLeaf((char*)accel,items);
      
      LBBox3fa bounds = empty;
      for (size_t i=0; i<items; i++)
        bounds.extend(accel[i].fillMB(prims.prims->data(),start,end,bvh->scene,prims.time_range));
      
      return typename BVH::NodeRecordMB4D(node,bounds,prims.time_range);
    };

    
  public:
    
    // 27.6 - 46 bytes per primitive
    unsigned char N;
    unsigned char data[4+25*M+16];

    /*
    struct Layout
    {
      unsigned int geomID;
      unsigned int primID[N];
      
      char bounds_vx_x[N];
      char bounds_vx_y[N];
      char bounds_vx_z[N];
      short bounds_vx_lower[N];
      short bounds_vx_upper[N];
      
      char bounds_vy_x[N];
      char bounds_vy_y[N];
      char bounds_vy_z[N];
      short bounds_vy_lower[N];
      short bounds_vy_upper[N];
      
      char bounds_vz_x[N];
      char bounds_vz_y[N];
      char bounds_vz_z[N];
      short bounds_vz_lower[N];
      short bounds_vz_upper[N];
      
      Vec3f offset;
      float scale;
    };
    */
    
    __forceinline       unsigned int& geomID(size_t N)       { return *(unsigned int*)((char*)this+1); }
    __forceinline const unsigned int& geomID(size_t N) const { return *(unsigned int*)((char*)this+1); }
    
    __forceinline       unsigned int* primID(size_t N)       { return (unsigned int*)((char*)this+5); }
    __forceinline const unsigned int* primID(size_t N) const { return (unsigned int*)((char*)this+5); }
    
    __forceinline       char* bounds_vx_x(size_t N)       { return (char*)((char*)this+5+4*N); }
    __forceinline const char* bounds_vx_x(size_t N) const { return (char*)((char*)this+5+4*N); }
    
    __forceinline       char* bounds_vx_y(size_t N)       { return (char*)((char*)this+5+5*N); }
    __forceinline const char* bounds_vx_y(size_t N) const { return (char*)((char*)this+5+5*N); }
    
    __forceinline       char* bounds_vx_z(size_t N)       { return (char*)((char*)this+5+6*N); }
    __forceinline const char* bounds_vx_z(size_t N) const { return (char*)((char*)this+5+6*N); }
    
    __forceinline       short* bounds_vx_lower(size_t N)       { return (short*)((char*)this+5+7*N); }
    __forceinline const short* bounds_vx_lower(size_t N) const { return (short*)((char*)this+5+7*N); }
    
    __forceinline       short* bounds_vx_upper(size_t N)       { return (short*)((char*)this+5+9*N); }
    __forceinline const short* bounds_vx_upper(size_t N) const { return (short*)((char*)this+5+9*N); }
    
    __forceinline       char* bounds_vy_x(size_t N)       { return (char*)((char*)this+5+11*N); }
    __forceinline const char* bounds_vy_x(size_t N) const { return (char*)((char*)this+5+11*N); }
    
    __forceinline       char* bounds_vy_y(size_t N)       { return (char*)((char*)this+5+12*N); }
    __forceinline const char* bounds_vy_y(size_t N) const { return (char*)((char*)this+5+12*N); }
    
    __forceinline       char* bounds_vy_z(size_t N)       { return (char*)((char*)this+5+13*N); }
    __forceinline const char* bounds_vy_z(size_t N) const { return (char*)((char*)this+5+13*N); }
    
    __forceinline       short* bounds_vy_lower(size_t N)       { return (short*)((char*)this+5+14*N); }
    __forceinline const short* bounds_vy_lower(size_t N) const { return (short*)((char*)this+5+14*N); }
    
    __forceinline       short* bounds_vy_upper(size_t N)       { return (short*)((char*)this+5+16*N); }
    __forceinline const short* bounds_vy_upper(size_t N) const { return (short*)((char*)this+5+16*N); }
    
    __forceinline       char* bounds_vz_x(size_t N)       { return (char*)((char*)this+5+18*N); }
    __forceinline const char* bounds_vz_x(size_t N) const { return (char*)((char*)this+5+18*N); }
    
    __forceinline       char* bounds_vz_y(size_t N)       { return (char*)((char*)this+5+19*N); }
    __forceinline const char* bounds_vz_y(size_t N) const { return (char*)((char*)this+5+19*N); }
    
    __forceinline       char* bounds_vz_z(size_t N)       { return (char*)((char*)this+5+20*N); }
    __forceinline const char* bounds_vz_z(size_t N) const { return (char*)((char*)this+5+20*N); }
    
    __forceinline       short* bounds_vz_lower(size_t N)       { return (short*)((char*)this+5+21*N); }
    __forceinline const short* bounds_vz_lower(size_t N) const { return (short*)((char*)this+5+21*N); }
    
    __forceinline       short* bounds_vz_upper(size_t N)       { return (short*)((char*)this+5+23*N); }
    __forceinline const short* bounds_vz_upper(size_t N) const { return (short*)((char*)this+5+23*N); }
    
    __forceinline       Vec3f* offset(size_t N)       { return (Vec3f*)((char*)this+5+25*N); }
    __forceinline const Vec3f* offset(size_t N) const { return (Vec3f*)((char*)this+5+25*N); }
    
    __forceinline       float* scale(size_t N)       { return (float*)((char*)this+5+25*N+12); }
    __forceinline const float* scale(size_t N) const { return (float*)((char*)this+5+25*N+12); }

    __forceinline       char* end(size_t N)       { return (char*)this+5+25*N+16; }
    __forceinline const char* end(size_t N) const { return (char*)this+5+25*N+16; }
  };
}
