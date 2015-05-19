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

#include <cmath>
#include "common/subdiv/bspline_patch.h"
#include "common/subdiv/bezier_patch.h"
#include "common/subdiv/gregory_patch.h"
#include "common/subdiv/gregory_triangle_patch.h"
#include "common/subdiv/tessellation.h"

#define FORCE_TESSELLATION_BOUNDS 1
#define USE_DISPLACEMENT_FOR_TESSELLATION_BOUNDS 1
#define DISCRITIZED_UV 1

namespace embree
{

  template<class T> 
    __forceinline T bilinear_interpolate(const float x0, const float x1, const float x2, const float x3,const T &u, const T &v)
    {
      return (1.0f-u) * (1.0f-v) * x0 + u * (1.0f-v) * x1 + u * v * x2 + (1.0f-u) * v * x3; 
    }


  template<class T>
    __forceinline T *aligned_alloca(size_t elements, const size_t alignment = 64)
    {
      void *ptr = alloca(elements * sizeof(T) + alignment);
      return (T*)ALIGN_PTR(ptr,alignment);
    }
  
  /* adjust discret tessellation level for feature-adaptive pre-subdivision */
    __forceinline float adjustDiscreteTessellationLevel(float l, const int sublevel = 0)
    {
      for (size_t i=0; i<sublevel; i++) l *= 0.5f;
      float r = ceilf(l);      
      for (size_t i=0; i<sublevel; i++) r *= 2.0f;
      return r;
    }

#if !defined(__MIC__)

  /* 3x3 point grid => 2x2 quad grid */


  struct __aligned(64) Quad2x2
  {

    /*  v00 - v01 - v02 */
    /*  v10 - v11 - v12 */
    /*  v20 - v21 - v22 */
  
    /* v00 - v10 - v01 - v11 - v02 - v12 */
    /* v10 - v20 - v11 - v21 - v12 - v22 */
   
    Quad2x2() 
      {
      }


    float vtx_x[12];
    float vtx_y[12];
    float vtx_z[12];
#if DISCRITIZED_UV == 1
    unsigned short vtx_u[12];
    unsigned short vtx_v[12];
#else
    float vtx_u[12];
    float vtx_v[12];
#endif

    static __forceinline ssei u16_to_ssei(const unsigned short *const source) // FIXME: move to ssei header
    {
#if defined (__SSE4_1__)
      return _mm_cvtepu16_epi32(loadu4i(source));
#else
      return ssei(source[0],source[1],source[2],source[3]);
#endif
    } 

    static __forceinline ssef u16_to_ssef(const unsigned short *const source)
    {
      const ssei t = u16_to_ssei(source);
      return ssef(t) * 1.0f/65535.0f;
    } 

    static __forceinline float u16_to_float(const unsigned short source)
    {
      return (float)source * 1.0f/65535.0f;
    } 


    static __forceinline unsigned short float_to_u16(const float f) { return (unsigned short)(f*65535.0f); }
    static __forceinline ssei float_to_u16(const ssef &f) { return (ssei)(f*65535.0f); }
 


    __forceinline void initFrom3x3Grid( const float *const source,
                                        float *const dest,
                                        const size_t offset_line0,
                                        const size_t offset_line1,
                                        const size_t offset_line2)
    {
      const float v00 = source[offset_line0 + 0];
      const float v01 = source[offset_line0 + 1];
      const float v02 = source[offset_line0 + 2];
      const float v10 = source[offset_line1 + 0];
      const float v11 = source[offset_line1 + 1];
      const float v12 = source[offset_line1 + 2];
      const float v20 = source[offset_line2 + 0];
      const float v21 = source[offset_line2 + 1];
      const float v22 = source[offset_line2 + 2];

      /* v00 - v10 - v01 - v11 - v02 - v12 */
      dest[ 0] = v00;
      dest[ 1] = v10;
      dest[ 2] = v01;
      dest[ 3] = v11;
      dest[ 4] = v02;
      dest[ 5] = v12;
      /* v10 - v20 - v11 - v21 - v12 - v22 */
      dest[ 6] = v10;
      dest[ 7] = v20;
      dest[ 8] = v11;
      dest[ 9] = v21;
      dest[10] = v12;
      dest[11] = v22;
    }

    __forceinline void initFrom3x3Grid( const ssef source[3],
                                        float *const dest)
    {
      const ssef row0_a = unpacklo(source[0],source[1]); 
      const ssef row0_b = shuffle<0,1,0,1>(unpackhi(source[0],source[1]));
      const ssef row1_a = unpacklo(source[1],source[2]);
      const ssef row1_b = shuffle<0,1,0,1>(unpackhi(source[1],source[2]));

      storeu4f(&dest[2], row0_b);
      storeu4f(&dest[8], row1_b);
      storeu4f(&dest[0], row0_a);
      storeu4f(&dest[6], row1_a);
    }

    __forceinline void initFrom3x3Grid_discritized( const ssef source[3],
                                                    unsigned short *const dest)
    {
      const ssef row0_a = unpacklo(source[0],source[1]); 
      const ssef row0_b = shuffle<0,1,0,1>(unpackhi(source[0],source[1]));
      const ssef row1_a = unpacklo(source[1],source[2]);
      const ssef row1_b = shuffle<0,1,0,1>(unpackhi(source[1],source[2]));

      //FIXME: use intrinsics for conversion
      for (size_t i=0;i<4;i++)
        dest[2+i] = (unsigned short)(row0_b[i]*65535.0f);
      for (size_t i=0;i<4;i++)
        dest[8+i] = (unsigned short)(row1_b[i]*65535.0f);
      for (size_t i=0;i<4;i++)
        dest[0+i] = (unsigned short)(row0_a[i]*65535.0f);
      for (size_t i=0;i<4;i++)
        dest[6+i] = (unsigned short)(row1_a[i]*65535.0f);

    }

    __forceinline void initFrom3x3Grid_discritized( const float *const source,
                                                    unsigned short *const dest,
                                                    const size_t offset_line0,
                                                    const size_t offset_line1,
                                                    const size_t offset_line2)
    {
      const float v00 = source[offset_line0 + 0];
      const float v01 = source[offset_line0 + 1];
      const float v02 = source[offset_line0 + 2];
      const float v10 = source[offset_line1 + 0];
      const float v11 = source[offset_line1 + 1];
      const float v12 = source[offset_line1 + 2];
      const float v20 = source[offset_line2 + 0];
      const float v21 = source[offset_line2 + 1];
      const float v22 = source[offset_line2 + 2];

      /* v00 - v10 - v01 - v11 - v02 - v12 */
      dest[ 0] = float_to_u16(v00);
      dest[ 1] = float_to_u16(v10);
      dest[ 2] = float_to_u16(v01);
      dest[ 3] = float_to_u16(v11);
      dest[ 4] = float_to_u16(v02);
      dest[ 5] = float_to_u16(v12);
      /* v10 - v20 - v11 - v21 - v12 - v22 */
      dest[ 6] = float_to_u16(v10);
      dest[ 7] = float_to_u16(v20);
      dest[ 8] = float_to_u16(v11);
      dest[ 9] = float_to_u16(v21);
      dest[10] = float_to_u16(v12);
      dest[11] = float_to_u16(v22);
    }

    /* init from 3x3 point grid */
    void init( const float * const grid_x,
               const float * const grid_y,
               const float * const grid_z,
               const float * const grid_u,
               const float * const grid_v,
               const size_t offset_line0,
               const size_t offset_line1,
               const size_t offset_line2)
    {
      initFrom3x3Grid( grid_x, vtx_x, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_y, vtx_y, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_z, vtx_z, offset_line0, offset_line1, offset_line2 );
#if DISCRITIZED_UV == 1
      initFrom3x3Grid_discritized( grid_u, vtx_u, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid_discritized( grid_v, vtx_v, offset_line0, offset_line1, offset_line2 );
#else
      initFrom3x3Grid( grid_u, vtx_u, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_v, vtx_v, offset_line0, offset_line1, offset_line2 );
#endif
    }

    /* init from 3x3 point grid */
    void init( const ssef grid_x[3],
               const ssef grid_y[3],
               const ssef grid_z[3],
               const ssef grid_u[3],
               const ssef grid_v[3])
    {
      initFrom3x3Grid( grid_x, vtx_x);
      initFrom3x3Grid( grid_y, vtx_y);
      initFrom3x3Grid( grid_z, vtx_z);
#if DISCRITIZED_UV == 1
      
      initFrom3x3Grid_discritized( grid_u, vtx_u);
      initFrom3x3Grid_discritized( grid_v, vtx_v);
#else
      initFrom3x3Grid( grid_u, vtx_u );
      initFrom3x3Grid( grid_v, vtx_v );
#endif
    }

        /* init from 3x3 point grid */
    void init_xyz( const ssef grid_x[3],
		   const ssef grid_y[3],
		   const ssef grid_z[3])
    {
      initFrom3x3Grid( grid_x, vtx_x);
      initFrom3x3Grid( grid_y, vtx_y);
      initFrom3x3Grid( grid_z, vtx_z);
    }


#if defined(__AVX__)

    __forceinline avxf combine( const float *const source, const size_t offset) const {
      return avxf( ssef::loadu(&source[0+offset]), ssef::loadu(&source[6+offset]) ); // FIXME: unaligned loads
    }

    __forceinline avxi combine_discritized( const unsigned short *const source, const size_t offset) const {
      
      return avxi( u16_to_ssei(&source[0+offset]), u16_to_ssei(&source[6+offset]) );            
    }

    __forceinline avx3f getVtx( const size_t offset, const size_t delta = 0 ) const {
      return avx3f(  combine(vtx_x,offset), combine(vtx_y,offset), combine(vtx_z,offset) );
    }

    __forceinline avx2f getUV( const size_t offset, const size_t delta = 0 ) const {
#if DISCRITIZED_UV == 1
      return avx2f(  avxf(combine_discritized(vtx_u,offset)) * 1.0f/65535.0f, avxf(combine_discritized(vtx_v,offset)) * 1.0f/65535.0f )  ;
#else
      return avx2f(  combine(vtx_u,offset), combine(vtx_v,offset) );
#endif
    }

#else

    __forceinline sse3f getVtx( const size_t offset, const size_t delta = 0 ) const {
      return sse3f( loadu4f(&vtx_x[offset+delta]), loadu4f(&vtx_y[offset+delta]), loadu4f(&vtx_z[offset+delta]) );
    }

    __forceinline sse2f getUV( const size_t offset, const size_t delta = 0 ) const {
#if DISCRITIZED_UV == 1
      return sse2f( u16_to_ssef(&vtx_u[offset+delta]), u16_to_ssef(&vtx_v[offset+delta]) );
#else
      return sse2f(  loadu4f(&vtx_u[offset+delta]), loadu4f(&vtx_v[offset+delta])  );
#endif
    }
    
#endif

    
    __forceinline BBox3fa bounds() const 
    {
      BBox3fa b( empty );
      for (size_t i=0;i<12;i++)
        b.extend( Vec3fa(vtx_x[i],vtx_y[i],vtx_z[i]) );
      return b;
    }
    
    __forceinline Vec3fa getVec3fa_xyz(const size_t i) const {
      return Vec3fa( vtx_x[i], vtx_y[i], vtx_z[i] );
    }

    __forceinline Vec2f getVec2f_uv(const size_t i) const {
      return Vec2f( vtx_u[i], vtx_v[i] );
    }

  };

  /*! Outputs ray to stream. */
  inline std::ostream& operator<<(std::ostream& cout, const Quad2x2& qquad) {
    for (size_t i=0;i<12;i++)
      cout << "i = " << i << " -> xyz = " << qquad.getVec3fa_xyz(i) << " uv = " << qquad.getVec2f_uv(i) << std::endl;
    return cout;
  }

#endif

#if defined(__AVX__)
  __forceinline BBox3fa getBBox3fa(const avx3f &v)
  {
    const Vec3fa b_min( reduce_min(v.x), reduce_min(v.y), reduce_min(v.z) );
    const Vec3fa b_max( reduce_max(v.x), reduce_max(v.y), reduce_max(v.z) );
    return BBox3fa( b_min, b_max );
  }
#endif

#if defined(__SSE__)
  __forceinline BBox3fa getBBox3fa(const sse3f &v)
  {
    const Vec3fa b_min( reduce_min(v.x), reduce_min(v.y), reduce_min(v.z) );
    const Vec3fa b_max( reduce_max(v.x), reduce_max(v.y), reduce_max(v.z) );
    return BBox3fa( b_min, b_max );
  }
#endif

#if defined(__MIC__)
  __forceinline BBox3fa getBBox3fa(const mic3f &v, const mic_m m_valid = 0xffff)
  {
    const mic_f x_min = select(m_valid,v.x,mic_f::inf());
    const mic_f y_min = select(m_valid,v.y,mic_f::inf());
    const mic_f z_min = select(m_valid,v.z,mic_f::inf());

    const mic_f x_max = select(m_valid,v.x,mic_f::minus_inf());
    const mic_f y_max = select(m_valid,v.y,mic_f::minus_inf());
    const mic_f z_max = select(m_valid,v.z,mic_f::minus_inf());
    
    const Vec3fa b_min( reduce_min(x_min), reduce_min(y_min), reduce_min(z_min) );
    const Vec3fa b_max( reduce_max(x_max), reduce_max(y_max), reduce_max(z_max) );
    return BBox3fa( b_min, b_max );
  }
#endif

  struct __aligned(16) GridRange
  {
    unsigned int u_start;
    unsigned int u_end;
    unsigned int v_start;
    unsigned int v_end;

    __forceinline GridRange() {}

#if defined(__MIC__)    
    __forceinline void operator=(const GridRange& v) {
      store4f((float*)this,broadcast4to16f((float*)&v));
    };
#endif

    __forceinline GridRange(unsigned int u_start, unsigned int u_end, unsigned int v_start, unsigned int v_end) : u_start(u_start), u_end(u_end), v_start(v_start), v_end(v_end) {}

    __forceinline bool hasLeafSize() const
    {

#if defined(__MIC__)
      return u_end-u_start <= 1 && v_end-v_start <= 1;
#else
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      assert(u_size >= 1);
      assert(v_size >= 1);

      return u_size <= 3 && v_size <= 3;
#endif
    }


    static __forceinline unsigned int split(unsigned int start,unsigned int end)
    {
      const unsigned int size = end-start+1;
#if defined(__MIC__)
      //assert( size > 4 );
      //const unsigned int blocks4 = (end-start+1+4-1)/4;
      //const unsigned int center  = (start + (blocks4/2)*4)-1; 
      //assert ((center-start+1) % 4 == 0);
      const unsigned int center = (start+end)/2;
      assert(center<end);
      return center;
#else
      // FIXME: for xeon the divide is 3!
      const unsigned int center = (start+end)/2;
#endif

      assert (center > start);
      assert (center < end);
      return center;
    }

    __forceinline void split(GridRange &r0, GridRange &r1) const
    {
      //if (hasLeafSize()) return false;
      assert( hasLeafSize() == false );
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      r0 = *this;
      r1 = *this;

      if (u_size >= v_size)
        {
          //assert(u_size >= 3);
          const unsigned int u_mid = split(u_start,u_end);
          r0.u_end   = u_mid;
          r1.u_start = u_mid;
        }
      else
        {
          //assert(v_size >= 3);
          const unsigned int v_mid = split(v_start,v_end);
          r0.v_end   = v_mid;
          r1.v_start = v_mid;
        }

    }

    __forceinline unsigned int splitIntoSubRanges(GridRange r[4]) const
    {
      assert( !hasLeafSize() );
      size_t children = 0;
      GridRange first,second;
      split(first,second);
      if (first.hasLeafSize())
	{
	  children = 1;
	  r[0] = first;
	}
      else
	{
	  first.split(r[0],r[1]);
	  children = 2;
	}

      if (second.hasLeafSize())
	{
	  r[children] = second;
	  children++;
	}
      else
	{
	  second.split(r[children+0],r[children+1]);
	  children += 2;
	}
      return children;      
    }

  };

  inline std::ostream& operator<<(std::ostream& cout, const GridRange& r) {
    cout << "range: u_start " << r.u_start << " u_end " << r.u_end << " v_start " << r.v_start << " v_end " << r.v_end << std::endl;
    return cout;
  }

  struct __aligned(64) SubdivPatch1Base
  {
  public:

    enum {
      BSPLINE_PATCH     = 1,  
      BEZIER_PATCH      = 2,  
      GREGORY_PATCH     = 4,
      TRANSITION_PATCH  = 8,  // needs stiching?
      HAS_DISPLACEMENT  = 16   // 0 => no displacments
    };

    /*! Default constructor. */
    __forceinline SubdivPatch1Base () {}

    /*! Construction from vertices and IDs. */
    SubdivPatch1Base (const CatmullClarkPatch3fa& ipatch,
                      const unsigned int gID,
                      const unsigned int pID,
                      const SubdivMesh *const mesh,
                      const Vec2f uv[4],
                      const float edge_level[4]);

    SubdivPatch1Base (const unsigned int gID,
                      const unsigned int pID,
                      const SubdivMesh *const mesh);

    __forceinline bool needsStitching() const
    {
      return (flags & TRANSITION_PATCH) == TRANSITION_PATCH;      
    }

    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      if (likely(isBezierPatch()))
        return BezierPatch::eval_bezier( patch.v, uu, vv );
      else if (likely(isBSplinePatch()))
        return patch.eval(uu,vv);
      else if (likely(isGregoryPatch()))
	return GregoryPatch::eval( patch.v, uu, vv );
      return Vec3fa( zero );
    }

    __forceinline Vec3fa normal(const float &uu,
				const float &vv) const
    {
      if (likely(isBezierPatch()))
        return BezierPatch::normal_bezier( patch.v, uu, vv );
      else if (likely(isBSplinePatch()))
        return patch.normal(uu,vv);
      else if (likely(isGregoryPatch()))
	return GregoryPatch::normal( patch.v, uu, vv );
      return Vec3fa( zero );
    }

#if defined(__SSE__)
    __forceinline sse3f eval4(const ssef &uu,
			      const ssef &vv) const
    {
      if (likely(isBezierPatch()))
        return BezierPatch::eval4_bezier( patch.v, uu, vv );
      else if (likely(isBSplinePatch()))
        return patch.eval(uu,vv);
      else if (likely(isGregoryPatch()))
	return GregoryPatch::eval_t<sseb>( patch.v, uu, vv );
      return sse3f( zero );
    }

    __forceinline sse3f normal4(const ssef &uu,
                                const ssef &vv) const
    {
      if (likely(isBezierPatch()))
        return BezierPatch::normal4_bezier( patch.v, uu, vv );
      else if (likely(isBSplinePatch()))
        return patch.normal(uu,vv);
      else if (likely(isGregoryPatch()))
	return GregoryPatch::normal_t<sseb>( patch.v, uu, vv );
      return sse3f( zero );
    }

#endif

#if defined(__AVX__)
    __forceinline avx3f eval8(const avxf &uu,
			      const avxf &vv) const
    {
      if (likely(isBezierPatch()))
        return BezierPatch::eval8_bezier( patch.v, uu, vv );
      else if (likely(isBSplinePatch()))
        return patch.eval(uu,vv);
      else if (likely(isGregoryPatch()))
	return GregoryPatch::eval_t<avxb>( patch.v, uu, vv );
      return avx3f( zero );
    }
    __forceinline avx3f normal8(const avxf &uu,
                                const avxf &vv) const
    {
      if (likely(isBezierPatch()))
        return BezierPatch::normal8_bezier( patch.v, uu, vv );
      else if (likely(isBSplinePatch()))
        return patch.normal(uu,vv);
      else if (likely(isGregoryPatch()))
	return GregoryPatch::normal_t<avxb>( patch.v, uu, vv );
      return avx3f( zero );
    }
#endif

#if defined(__MIC__)
    __forceinline mic_f eval4(const mic_f &uu,
			      const mic_f &vv) const
    {
      if (likely(isBSplinePatch()))
	{
	  return patch.eval4(uu,vv);
	}
      else 
	{	  
	  return GregoryPatch::eval4( patch.v, uu, vv );
	}     
    }

    __forceinline mic3f eval16(const mic_f &uu,
			       const mic_f &vv) const
    {
      if (likely(isBSplinePatch()))
        return patch.eval(uu,vv);
      else 
        return GregoryPatch::eval16( patch.v, uu, vv );
    }

    __forceinline mic3f normal16(const mic_f &uu,
				 const mic_f &vv) const
    {
      if (likely(isBSplinePatch()))
	return patch.normal(uu,vv);
      else
        return GregoryPatch::normal16( patch.v, uu, vv );
    }
#endif


    __forceinline bool isBSplinePatch() const
    {
      return (flags & BSPLINE_PATCH) == BSPLINE_PATCH;
    }

    __forceinline bool isBezierPatch() const
    {
      return (flags & BEZIER_PATCH) == BEZIER_PATCH;
    }

    __forceinline bool isGregoryPatch() const
    {
      return (flags & GREGORY_PATCH) == GREGORY_PATCH;
    }

    __forceinline bool hasDisplacement() const
    {
      return (flags & HAS_DISPLACEMENT) == HAS_DISPLACEMENT;
    }


    __forceinline void prefetchData() const
    {
      const char *const t = (char*)this;
      prefetchL1(t + 0*64);
      prefetchL1(t + 1*64);
      prefetchL1(t + 2*64);
      prefetchL1(t + 3*64);
      prefetchL1(t + 4*64);
    }

    __forceinline Vec2f getUV(const size_t i) const
    {
      return Vec2f((float)u[i],(float)v[i]) * 1.0f/65535.0f;
    }


#if defined(__MIC__)
    __forceinline void store(void *mem)
    {
      const mic_f *const src = (mic_f*)this;
      assert(sizeof(SubdivPatch1Base) % 64 == 0);
      mic_f *const dst = (mic_f*)mem;
#pragma unroll
      for (size_t i=0;i<sizeof(SubdivPatch1Base) / 64;i++)
	store16f_ngo(&dst[i],src[i]);
    }
#endif

    void updateEdgeLevels(const float edge_level[4],const SubdivMesh *const mesh);

    __forceinline size_t gridOffset(const size_t y, const size_t x) const
    {
      return grid_u_res*y+x;
    }

    void evalToOBJ(Scene *scene, size_t &vertex_index,size_t &numTotalTriangles);
    
  private:

    size_t get64BytesBlocksForGridSubTree(const GridRange &range,
                                          const unsigned int leafBlocks)
    {
      if (range.hasLeafSize()) return leafBlocks;

      __aligned(64) GridRange r[4];

      const unsigned int children = range.splitIntoSubRanges(r);

      size_t blocks = 2; /* 128 bytes bvh4 node layout */

      for (unsigned int i=0;i<children;i++)
	blocks += get64BytesBlocksForGridSubTree(r[i],
						 leafBlocks);
      return blocks;    
    }




  public:
    __forceinline unsigned int getSubTreeSize64bBlocks(const unsigned int leafBlocks = 2)
    {
#if defined(__MIC__)
      const unsigned int U_BLOCK_SIZE = 5;
      const unsigned int V_BLOCK_SIZE = 3;

      const unsigned int grid_u_blocks = (grid_u_res + U_BLOCK_SIZE-2) / (U_BLOCK_SIZE-1);
      const unsigned int grid_v_blocks = (grid_v_res + V_BLOCK_SIZE-2) / (V_BLOCK_SIZE-1);

      return get64BytesBlocksForGridSubTree(GridRange(0,grid_u_blocks,0,grid_v_blocks),leafBlocks);
#else
      return get64BytesBlocksForGridSubTree(GridRange(0,grid_u_res-1,0,grid_v_res-1),leafBlocks);
#endif
    }

    __forceinline void read_lock()      { mtx.read_lock();    }
    __forceinline void read_unlock()    { mtx.read_unlock();  }
    __forceinline void write_lock()     { mtx.write_lock();   }
    __forceinline void write_unlock()   { mtx.write_unlock(); }
    
    __forceinline bool try_write_lock() { return mtx.try_read_lock(); }
    __forceinline bool try_read_lock()  { return mtx.try_read_lock(); }
    
    __forceinline void upgrade_read_to_write_lock() { mtx.upgrade_read_to_write_lock(); }
    __forceinline void upgrade_write_to_read_lock() { mtx.upgrade_write_to_read_lock(); }

    __forceinline void resetRootRef()
    {
      assert( mtx.hasInitialState() );
      root_ref = 0;
    }


    // 16bit discritized u,v coordinates

    unsigned short u[4]; 
    unsigned short v[4];
    float level[4];

    unsigned short flags;
    unsigned short grid_bvh_size_64b_blocks;
    unsigned int geom;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int prim;                          //!< primitive ID of this subdivision patch
    unsigned short grid_u_res;
    unsigned short grid_v_res;

    unsigned short grid_size_simd_blocks;
    unsigned short grid_subtree_size_64b_blocks;

    RWMutex mtx;
    volatile int64 root_ref;

    __aligned(64) BSplinePatch3fa patch;
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1Base &p)
  {
    o << " flags " << p.flags << " geomID " << p.geom << " primID " << p.prim << " levels: " << p.level[0] << "," << p.level[1] << "," << p.level[2] << "," << p.level[3] << std::endl;
    o << " patch " << p.patch;

    return o;
  } 

  /* eval grid over patch and stich edges when required */      
  static __forceinline void evalGrid(const SubdivPatch1Base &patch,
                                     float *__restrict__ const grid_x,
                                     float *__restrict__ const grid_y,
                                     float *__restrict__ const grid_z,
                                     float *__restrict__ const grid_u,
                                     float *__restrict__ const grid_v,
                                     const SubdivMesh* const geom)
  {
    /* grid_u, grid_v need to be padded as we write with SIMD granularity */
    gridUVTessellator(patch.level,
                      patch.grid_u_res,
                      patch.grid_v_res,
                      grid_u,
                      grid_v);

#if defined(__MIC__)
    const size_t SIMD_WIDTH = 16;
#else
    const size_t SIMD_WIDTH = 8;
#endif

    /* set last elements in u,v array to 1.0f */
    for (size_t i=patch.grid_u_res*patch.grid_v_res;i<patch.grid_size_simd_blocks*SIMD_WIDTH;i++)
      {
	grid_u[i] = 1.0f;
	grid_v[i] = 1.0f;
      }

    /* stitch edges if necessary */
    if (unlikely(patch.needsStitching()))
      stitchUVGrid(patch.level,patch.grid_u_res,patch.grid_v_res,grid_u,grid_v);
        
#if defined(__MIC__)

       for (size_t i = 0; i<patch.grid_size_simd_blocks; i++)
        {
          const mic_f u = load16f(&grid_u[i * 16]);
          const mic_f v = load16f(&grid_v[i * 16]);

	  //prefetch<PFHINT_L2EX>(&grid_x[16*i]);
	  //prefetch<PFHINT_L2EX>(&grid_y[16*i]);
	  //prefetch<PFHINT_L2EX>(&grid_z[16*i]);

          mic3f vtx = patch.eval16(u, v);

          /* eval displacement function */
	  if (unlikely(((SubdivMesh*)geom)->displFunc != nullptr))
            {
              mic3f normal = patch.normal16(u, v);
              normal = normalize(normal);

              const Vec2f uv0 = patch.getUV(0);
              const Vec2f uv1 = patch.getUV(1);
              const Vec2f uv2 = patch.getUV(2);
              const Vec2f uv3 = patch.getUV(3);

              const mic_f patch_uu = bilinear_interpolate(uv0.x, uv1.x, uv2.x, uv3.x, u, v);
              const mic_f patch_vv = bilinear_interpolate(uv0.y, uv1.y, uv2.y, uv3.y, u, v);

              ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
					     patch.geom,
					     patch.prim,
					     (const float*)&patch_uu,
					     (const float*)&patch_vv,
					     (const float*)&normal.x,
					     (const float*)&normal.y,
					     (const float*)&normal.z,
					     (float*)&vtx.x,
					     (float*)&vtx.y,
					     (float*)&vtx.z,
					     16);

            }
	  //prefetch<PFHINT_L1EX>(&grid_x[16*i]);
	  //prefetch<PFHINT_L1EX>(&grid_y[16*i]);
	  //prefetch<PFHINT_L1EX>(&grid_z[16*i]);

	  store16f_ngo(&grid_x[16*i],vtx.x);
	  store16f_ngo(&grid_y[16*i],vtx.y);
	  store16f_ngo(&grid_z[16*i],vtx.z);
        }
   
#else        
#if defined(__AVX__)
    for (size_t i=0;i<patch.grid_size_simd_blocks;i++)
      {
        avxf uu = load8f(&grid_u[8*i]);
        avxf vv = load8f(&grid_v[8*i]);
        avx3f vtx = patch.eval8(uu,vv);
                 
        if (unlikely(((SubdivMesh*)geom)->displFunc != nullptr))
          {
	    const Vec2f uv0 = patch.getUV(0);
	    const Vec2f uv1 = patch.getUV(1);
	    const Vec2f uv2 = patch.getUV(2);
	    const Vec2f uv3 = patch.getUV(3);

            avx3f normal = patch.normal8(uu,vv);
            normal = normalize_safe(normal) ;
            
            const avxf patch_uu = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,uu,vv);
            const avxf patch_vv = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,uu,vv);
            
            ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                           patch.geom,
                                           patch.prim,
                                           (const float*)&patch_uu,
                                           (const float*)&patch_vv,
                                           (const float*)&normal.x,
                                           (const float*)&normal.y,
                                           (const float*)&normal.z,
                                           (float*)&vtx.x,
                                           (float*)&vtx.y,
                                           (float*)&vtx.z,
                                           8);
          }
        *(avxf*)&grid_x[8*i] = vtx.x;
        *(avxf*)&grid_y[8*i] = vtx.y;
        *(avxf*)&grid_z[8*i] = vtx.z;        
      }
#else
    for (size_t i=0;i<patch.grid_size_simd_blocks*2;i++) // 4-wide blocks for SSE
      {
        ssef uu = load4f(&grid_u[4*i]);
        ssef vv = load4f(&grid_v[4*i]);
        sse3f vtx = patch.eval4(uu,vv);
          
          
        if (unlikely(((SubdivMesh*)geom)->displFunc != nullptr))
          {
	    const Vec2f uv0 = patch.getUV(0);
	    const Vec2f uv1 = patch.getUV(1);
	    const Vec2f uv2 = patch.getUV(2);
	    const Vec2f uv3 = patch.getUV(3);

            sse3f normal = patch.normal4(uu,vv);
            normal = normalize_safe(normal);

            const ssef patch_uu = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,uu,vv);
            const ssef patch_vv = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,uu,vv);
            
            ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                           patch.geom,
                                           patch.prim,
                                           (const float*)&patch_uu,
                                           (const float*)&patch_vv,
                                           (const float*)&normal.x,
                                           (const float*)&normal.y,
                                           (const float*)&normal.z,
                                           (float*)&vtx.x,
                                           (float*)&vtx.y,
                                           (float*)&vtx.z,
                                           4);
          }
          
        *(ssef*)&grid_x[4*i] = vtx.x;
        *(ssef*)&grid_y[4*i] = vtx.y;
        *(ssef*)&grid_z[4*i] = vtx.z;        
      }
#endif
#endif        
  }


}
