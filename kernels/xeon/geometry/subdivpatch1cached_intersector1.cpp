// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "subdivpatch1cached_intersector1.h"

#include "common/subdiv/tessellation.h"

namespace embree
{

#define FORCE_TRIANGLE_UV 1

  struct __aligned(64) Quad2x2
  {
    Quad2x2() {}

    float vtx_x[12];
    float vtx_y[12];
    float vtx_z[12];
    float vtx_u[12];
    float vtx_v[12];
    float dummy[3];

    /*  v00 - v01 - v02 */
    /*  v10 - v11 - v12 */
    /*  v20 - v21 - v22 */

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

    /* init from 3x3 point grid */
    Quad2x2( const float * const grid_x,
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
        initFrom3x3Grid( grid_u, vtx_u, offset_line0, offset_line1, offset_line2 );
        initFrom3x3Grid( grid_v, vtx_v, offset_line0, offset_line1, offset_line2 );
      }

#if defined(__AVX__)

    __forceinline avxf combine( const float *const source, const size_t offset ) const {
      return avxf( *(ssef*)&source[0+offset], *(ssef*)&source[6+offset] );            
    }

    __forceinline avx3f getVtx( const size_t offset ) const {
      return avx3f(  combine(vtx_x,offset), combine(vtx_y,offset), combine(vtx_z,offset) );
    }

    __forceinline avx2f getUV( const size_t offset ) const {
      return avx2f(  combine(vtx_u,offset), combine(vtx_v,offset) );
     }

#endif

  };

#if defined(__AVX__)
  static __forceinline void intersect1_tri8_precise(Ray& ray,
                                                    const avx3f &v0_org,
                                                    const avx3f &v1_org,
                                                    const avx3f &v2_org,
                                                    const float *__restrict__ const u_grid,
                                                    const float *__restrict__ const v_grid,
                                                    const size_t offset_v0,
                                                    const size_t offset_v1,
                                                    const size_t offset_v2,
                                                    const avxb &m_active,
                                                    const SubdivPatch1Cached *const sptr,
                                                    const void* geom)
  {
    const avx3f O = ray.org;
    const avx3f D = ray.dir;

    const avx3f v0 = v0_org - O;
    const avx3f v1 = v1_org - O;
    const avx3f v2 = v2_org - O;
   
    const avx3f e0 = v2 - v0;
    const avx3f e1 = v0 - v1;	     
    const avx3f e2 = v1 - v2;	     

    /* calculate geometry normal and denominator */
    const avx3f Ng1 = cross(e1,e0);
    const avx3f Ng = Ng1+Ng1;
    const avxf den = dot(Ng,D);
    const avxf absDen = abs(den);
    const avxf sgnDen = signmsk(den);
      
    avxb valid = m_active;
    /* perform edge tests */
    const avxf U = dot(avx3f(cross(v2+v0,e0)),D) ^ sgnDen;
    valid &= U >= 0.0f;
    if (likely(none(valid))) return;
    const avxf V = dot(avx3f(cross(v0+v1,e1)),D) ^ sgnDen;
    valid &= V >= 0.0f;
    if (likely(none(valid))) return;
    const avxf W = dot(avx3f(cross(v1+v2,e2)),D) ^ sgnDen;
    valid &= W >= 0.0f;
    if (likely(none(valid))) return;
      
    /* perform depth test */
    const avxf T = dot(v0,Ng) ^ sgnDen;
    valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
    if (unlikely(none(valid))) return;
      
    /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
    valid &= den > avxf(zero);
    if (unlikely(none(valid))) return;
#else
    valid &= den != avxf(zero);
    if (unlikely(none(valid))) return;
#endif
            
    /* calculate hit information */
    const avxf rcpAbsDen = rcp(absDen);
    const avxf u = U*rcpAbsDen;
    const avxf v = V*rcpAbsDen;
    const avxf t = T*rcpAbsDen;

#if FORCE_TRIANGLE_UV == 0
    const avxf _u0 = load8f(&u_grid[offset_v0]);
    const avxf _u1 = load8f(&u_grid[offset_v1]);
    const avxf _u2 = load8f(&u_grid[offset_v2]);
    const avxf u_final = u * _u1 + v * _u2 + (1.0f-u-v) * _u0;

    const avxf _v0 = load8f(&v_grid[offset_v0]);
    const avxf _v1 = load8f(&v_grid[offset_v1]);
    const avxf _v2 = load8f(&v_grid[offset_v2]);
    const avxf v_final = u * _v1 + v * _v2 + (1.0f-u-v) * _v0;
#else
    const avxf u_final = u;
    const avxf v_final = v;
#endif


    size_t i = select_min(valid,t);

    /* update hit information */
    ray.u = u_final[i];
    ray.v = v_final[i];
    ray.tfar = t[i];
    ray.Ng.x = Ng.x[i];
    ray.Ng.y = Ng.y[i];
    ray.Ng.z = Ng.z[i];
    ray.geomID = ((size_t)sptr)       & (unsigned int)-1; 
    ray.primID = (((size_t)sptr)>>32) & (unsigned int)-1;
      
  };

  static __forceinline void intersect1_quad8(Ray& ray,
                                             const float *__restrict__ const vtx_x,
                                             const float *__restrict__ const vtx_y,
                                             const float *__restrict__ const vtx_z,
                                             const float *__restrict__ const u,
                                             const float *__restrict__ const v,
                                             const size_t offset_v0,
                                             const size_t offset_v1,
                                             const size_t offset_v2,
                                             const size_t offset_v3,
                                             const avxb &m_active,
                                             const SubdivPatch1Cached *const sptr,
                                             const void* geom)
  {
    const avx3f v0( load8f(&vtx_x[offset_v0]), load8f(&vtx_y[offset_v0]), load8f(&vtx_z[offset_v0]));
    const avx3f v1( load8f(&vtx_x[offset_v1]), load8f(&vtx_y[offset_v1]), load8f(&vtx_z[offset_v1]));
    const avx3f v2( load8f(&vtx_x[offset_v2]), load8f(&vtx_y[offset_v2]), load8f(&vtx_z[offset_v2]));
    const avx3f v3( load8f(&vtx_x[offset_v3]), load8f(&vtx_y[offset_v3]), load8f(&vtx_z[offset_v3]));

    intersect1_tri8_precise(ray,v0,v1,v3,u,v,offset_v0,offset_v1,offset_v3,m_active,sptr,geom);
    intersect1_tri8_precise(ray,v3,v1,v2,u,v,offset_v3,offset_v1,offset_v2,m_active,sptr,geom);

  }


  static __forceinline void intersect1_quad8(Ray& ray,
                                             const Quad2x2 &quad,
                                             const SubdivPatch1Cached *const sptr,
                                             const void* geom)
  {
    const avx3f v0 = quad.getVtx( 0 );
    const avx3f v1 = quad.getVtx( 1 );
    const avx3f v2 = quad.getVtx( 2 );
    //intersect1_tri8_precise(ray,v0,v1,v2,u,v,offset_v0,offset_v1,offset_v3,m_active,sptr,geom);
  }

#endif


  void SubdivPatch1CachedIntersector1::intersect_subdiv_patch(const Precalculations& pre,
                                                              Ray& ray,
                                                              const Primitive& subdiv_patch,
                                                              const void* geom) // geom == mesh or geom == scene?
  {

#if defined(__AVX__)

    const float * const edge_levels = subdiv_patch.level;
    const unsigned int grid_u_res   = subdiv_patch.grid_u_res;
    const unsigned int grid_v_res   = subdiv_patch.grid_v_res;

#if 0
    DBG_PRINT(geom);
    DBG_PRINT(subdiv_patch.grid_size_8wide_blocks);
    DBG_PRINT(grid_u_res);
    DBG_PRINT(grid_v_res);
    DBG_PRINT(edge_levels[0]);
    DBG_PRINT(edge_levels[1]);
    DBG_PRINT(edge_levels[2]);
    DBG_PRINT(edge_levels[3]);
#endif

    __aligned(64) float u_array[(subdiv_patch.grid_size_8wide_blocks+1)*8]; // for unaligned access
    __aligned(64) float v_array[(subdiv_patch.grid_size_8wide_blocks+1)*8];

    __aligned(64) float vtx_x[(subdiv_patch.grid_size_8wide_blocks+1)*8];
    __aligned(64) float vtx_y[(subdiv_patch.grid_size_8wide_blocks+1)*8];
    __aligned(64) float vtx_z[(subdiv_patch.grid_size_8wide_blocks+1)*8];

    gridUVTessellator(edge_levels,grid_u_res,grid_v_res,u_array,v_array);

    if (unlikely(subdiv_patch.needsStiching()))
      stichUVGrid(edge_levels,grid_u_res,grid_v_res,u_array,v_array);

    for (size_t i=0;i<subdiv_patch.grid_size_8wide_blocks;i++)
      {
        const avxf uu = load8f(&u_array[8*i]);
        const avxf vv = load8f(&v_array[8*i]);
        const avx3f vtx = subdiv_patch.eval8(uu,vv);

        if (unlikely(geom != NULL))
          if (unlikely(((SubdivMesh*)geom)->displFunc != NULL))
            {
              avx3f normal = subdiv_patch.normal8(uu,vv);
              normal = normalize(normal);
              
              ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                             subdiv_patch.geom,
                                             subdiv_patch.prim,
                                             (const float*)&uu,
                                             (const float*)&vv,
                                             (const float*)&normal.x,
                                             (const float*)&normal.y,
                                             (const float*)&normal.z,
                                             (float*)&vtx.x,
                                             (float*)&vtx.y,
                                             (float*)&vtx.z,
                                             8);
            }
        
        *(avxf*)&vtx_x[8*i] = vtx.x;
        *(avxf*)&vtx_y[8*i] = vtx.y;
        *(avxf*)&vtx_z[8*i] = vtx.z;        
      }


    size_t offset_line0 = 0;
    size_t offset_line1 = grid_u_res;

    for (unsigned int y=0;y<grid_v_res-1;y++,offset_line0+=grid_u_res,offset_line1+=grid_u_res)
      {
        for (unsigned int x=0;x<grid_u_res-1;x+=8)
          {
            const size_t offset_v0 = offset_line0 + x + 0;
            const size_t offset_v1 = offset_line0 + x + 1;
            const size_t offset_v2 = offset_line1 + x + 1;
            const size_t offset_v3 = offset_line1 + x + 0;
            
            avxb m_active ( true );
            if (unlikely(x + 8 > (grid_u_res-1))) 
              {
                //DBG_PRINT( (grid_u_res-1)%8 );
                //DBG_PRINT( grid_u_res-1 );
              
                for (size_t i=(grid_u_res-1)%8;i<8;i++)
                  m_active[i] = 0;
              }
            //DBG_PRINT( m_active);
            intersect1_quad8(ray,vtx_x,vtx_y,vtx_z,
                             u_array,v_array,
                             offset_v0,offset_v1,offset_v2,offset_v3,
                             m_active,
                             &subdiv_patch,
                             geom );

          }
        //exit(0);
	      
      }	  


    
#endif
  }

  bool SubdivPatch1CachedIntersector1::occluded_subdiv_patch(const Precalculations& pre,
                                                             Ray& ray,
                                                             const Primitive& subdiv_patch,
                                                             const void* geom)
  {
    return false;
  }

};
