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

#pragma once

#include "catmullclark_patch.h"

namespace embree
{
  class CubicBSplineCurve
  {
  public:

    static __forceinline Vec4f eval(const float u)
    {
      const float t  = u;
      const float s  = 1.0f - u;
      const float n0 = s*s*s;
      const float n1 = (4.0f*s*s*s+t*t*t) + (12.0f*s*t*s + 6.0*t*s*t);
      const float n2 = (4.0f*t*t*t+s*s*s) + (12.0f*t*s*t + 6.0*s*t*s);
      const float n3 = t*t*t;
      //const float c  = 1.0f/6.0f; // do this later
      return Vec4f(n0,n1,n2,n3);
    }

    static __forceinline Vec4f derivative(const float u)
    {
      const float t  =  u;
      const float s  =  1.0f - u;
      const float n0 = -s*s;
      const float n1 = -t*t - 4.0f*t*s;
      const float n2 =  s*s + 4.0f*s*t;
      const float n3 =  t*t;
      //const float c  = 1.0f/6.0f; // do this later
      return Vec4f(n0,n1,n2,n3);
    }

#if defined(__MIC__)

    static __forceinline mic4f eval(const mic_f u)
    {
      const mic_f t  = u;
      const mic_f s  = 1.0f - u;
      const mic_f n0 = s*s*s;
      const mic_f n1 = (4.0f*s*s*s+t*t*t) + (12.0f*s*t*s + 6.0*t*s*t);
      const mic_f n2 = (4.0f*t*t*t+s*s*s) + (12.0f*t*s*t + 6.0*s*t*s);
      const mic_f n3 = t*t*t;
      //const mic_f c  = 1.0f/6.0f; // do this later
      return mic4f(n0,n1,n2,n3);
    }


    static __forceinline mic4f derivative(const mic_f u)
    {
      const mic_f t  =  u;
      const mic_f s  =  1.0f - u;
      const mic_f n0 = -s*s;
      const mic_f n1 = -t*t - 4.0f*t*s;
      const mic_f n2 =  s*s + 4.0f*s*t;
      const mic_f n3 =  t*t;
      //const mic_f c  = 1.0f/6.0f; // do this later
      return mic4f(n0,n1,n2,n3);
    }

    static __forceinline mic4f eval_derivative(const mic_f u, const mic_m m_mask)
    {
      const mic4f e = eval(u);
      const mic4f d = derivative(u);
      return mic4f(select(m_mask,e[0],d[0]),select(m_mask,e[1],d[1]),select(m_mask,e[2],d[2]),select(m_mask,e[3],d[3]));
    }    

#endif
  };


  template<typename T>
    class BSplinePatchT
    {
    public:
      T v[4][4];

      __forceinline T computeFaceVertex(const unsigned int y,const unsigned int x) const
      {
	return (v[y][x] + v[y][x+1] + v[y+1][x+1] + v[y+1][x]) * 0.25f;
      }

      __forceinline T computeQuadVertex(const unsigned int y,
					const unsigned int x,
					const T face[3][3]) const
      {
	const T P = v[y][x]; 
	const T Q = face[y-1][x-1] + face[y-1][x] + face[y][x] + face[y][x-1];
	const T R = v[y-1][x] + v[y+1][x] + v[y][x-1] + v[y][x+1];
	const T res = (Q + R) * 0.0625f + P * 0.5f;
	return res;
      }

      __forceinline T computeLimitVertex(const int y,
					 const int x) const
      {
	const T P = v[y][x];
	const T Q = v[y-1][x-1] + v[y-1][x+1] + v[y+1][x-1] + v[y+1][x+1];
	const T R = v[y-1][x] + v[y+1][x] + v[y][x-1] + v[y][x+1];
	const T res = (P * 16.0f + R * 4.0f + Q) * 1.0f / 36.0f;
	return res;
      }

      __forceinline T computeLimitTangentX(const int y,
                                           const int x) const
      {
	/* --- tangent X --- */
	const T Qx = v[y-1][x+1] - v[y-1][x-1] + v[y+1][x+1] - v[y+1][x-1];
	const T Rx = v[y][x-1] - v[y][x+1];
	const T tangentX = (Rx * 4.0f + Qx) * 1.0f / 12.0f;
        return tangentX;
      };

      __forceinline T computeLimitTangentY(const int y,
                                           const int x) const
      {
	const T Qy = v[y-1][x-1] - v[y+1][x-1] + v[y-1][x+1] - v[y+1][x+1];
	const T Ry = v[y-1][x] - v[y+1][x];
	const T tangentY = (Ry * 4.0f + Qy) * 1.0f / 12.0f;
    
	return tangentY;
      }

      __forceinline T computeLimitNormal(const int y,
					 const int x) const
      {
	/* --- tangent X --- */
	const T Qx = v[y-1][x+1] - v[y-1][x-1] + v[y+1][x+1] - v[y+1][x-1];
	const T Rx = v[y][x-1] - v[y][x+1];
	const T tangentX = (Rx * 4.0f + Qx) * 1.0f / 12.0f;

	/* --- tangent Y --- */
	const T Qy = v[y-1][x-1] - v[y+1][x-1] + v[y-1][x+1] - v[y+1][x+1];
	const T Ry = v[y-1][x] - v[y+1][x];
	const T tangentY = (Ry * 4.0f + Qy) * 1.0f / 12.0f;
    
	return cross(tangentX,tangentY);
      }

      __forceinline void initSubPatches(const T edge[12],
					const T face[3][3],
					const T newQuadVertex[2][2],
					BSplinePatchT child[4]) const
      {
	BSplinePatchT &subTL = child[0];
	BSplinePatchT &subTR = child[1];
	BSplinePatchT &subBR = child[2];
	BSplinePatchT &subBL = child[3];

	// top-left
	subTL.v[0][0] = face[0][0];
	subTL.v[0][1] = edge[0];
	subTL.v[0][2] = face[0][1];
	subTL.v[0][3] = edge[1];

	subTL.v[1][0] = edge[2];
	subTL.v[1][1] = newQuadVertex[0][0];
	subTL.v[1][2] = edge[3];
	subTL.v[1][3] = newQuadVertex[0][1];

	subTL.v[2][0] = face[1][0];
	subTL.v[2][1] = edge[5];
	subTL.v[2][2] = face[1][1];
	subTL.v[2][3] = edge[6];

	subTL.v[3][0] = edge[7];
	subTL.v[3][1] = newQuadVertex[1][0];
	subTL.v[3][2] = edge[8];
	subTL.v[3][3] = newQuadVertex[1][1];

	// top-right
	subTR.v[0][0] = edge[0];
	subTR.v[0][1] = face[0][1];
	subTR.v[0][2] = edge[1];
	subTR.v[0][3] = face[0][2];

	subTR.v[1][0] = newQuadVertex[0][0];
	subTR.v[1][1] = edge[3];
	subTR.v[1][2] = newQuadVertex[0][1];
	subTR.v[1][3] = edge[4];

	subTR.v[2][0] = edge[5];
	subTR.v[2][1] = face[1][1];
	subTR.v[2][2] = edge[6];
	subTR.v[2][3] = face[1][2];

	subTR.v[3][0] = newQuadVertex[1][0];
	subTR.v[3][1] = edge[8];
	subTR.v[3][2] = newQuadVertex[1][1];
	subTR.v[3][3] = edge[9];

	// buttom-right
	subBR.v[0][0] = newQuadVertex[0][0];
	subBR.v[0][1] = edge[3];
	subBR.v[0][2] = newQuadVertex[0][1];
	subBR.v[0][3] = edge[4];

	subBR.v[1][0] = edge[5];
	subBR.v[1][1] = face[1][1];
	subBR.v[1][2] = edge[6];
	subBR.v[1][3] = face[1][2];

	subBR.v[2][0] = newQuadVertex[1][0];
	subBR.v[2][1] = edge[8];
	subBR.v[2][2] = newQuadVertex[1][1];
	subBR.v[2][3] = edge[9];

	subBR.v[3][0] = edge[10];
	subBR.v[3][1] = face[2][1];
	subBR.v[3][2] = edge[11];
	subBR.v[3][3] = face[2][2];

	// buttom-left
	subBL.v[0][0] = edge[2];
	subBL.v[0][1] = newQuadVertex[0][0];
	subBL.v[0][2] = edge[3];
	subBL.v[0][3] = newQuadVertex[0][1];

	subBL.v[1][0] = face[1][0];
	subBL.v[1][1] = edge[5];
	subBL.v[1][2] = face[1][1];
	subBL.v[1][3] = edge[6];

	subBL.v[2][0] = edge[7];
	subBL.v[2][1] = newQuadVertex[1][0];
	subBL.v[2][2] = edge[8];
	subBL.v[2][3] = newQuadVertex[1][1];

	subBL.v[3][0] = face[2][0];
	subBL.v[3][1] = edge[10];
	subBL.v[3][2] = face[2][1];
	subBL.v[3][3] = edge[11];
      }

      __forceinline void subdivide(BSplinePatchT child[4]) const
      {
	T face[3][3];
	face[0][0] = computeFaceVertex(0,0);
	face[0][1] = computeFaceVertex(0,1);
	face[0][2] = computeFaceVertex(0,2);
	face[1][0] = computeFaceVertex(1,0);
	face[1][1] = computeFaceVertex(1,1);
	face[1][2] = computeFaceVertex(1,2);
	face[2][0] = computeFaceVertex(2,0);
	face[2][1] = computeFaceVertex(2,1);
	face[2][2] = computeFaceVertex(2,2);

	T edge[12];
	edge[0]  = 0.25f * (v[0][1] + v[1][1] + face[0][0] + face[0][1]);
	edge[1]  = 0.25f * (v[0][2] + v[1][2] + face[0][1] + face[0][2]);
	edge[2]  = 0.25f * (v[1][0] + v[1][1] + face[0][0] + face[1][0]);
	edge[3]  = 0.25f * (v[1][1] + v[1][2] + face[0][1] + face[1][1]);
	edge[4]  = 0.25f * (v[1][2] + v[1][3] + face[0][2] + face[1][2]);
	edge[5]  = 0.25f * (v[1][1] + v[2][1] + face[1][0] + face[1][1]);
	edge[6]  = 0.25f * (v[1][2] + v[2][2] + face[1][1] + face[1][2]);
	edge[7]  = 0.25f * (v[2][0] + v[2][1] + face[1][0] + face[2][0]);
	edge[8]  = 0.25f * (v[2][1] + v[2][2] + face[1][1] + face[2][1]);
	edge[9]  = 0.25f * (v[2][2] + v[2][3] + face[1][2] + face[2][2]);
	edge[10] = 0.25f * (v[2][1] + v[3][1] + face[2][0] + face[2][1]);
	edge[11] = 0.25f * (v[2][2] + v[3][2] + face[2][1] + face[2][2]);

	T newQuadVertex[2][2];
	newQuadVertex[0][0] = computeQuadVertex(1,1,face);
	newQuadVertex[0][1] = computeQuadVertex(1,2,face);
	newQuadVertex[1][1] = computeQuadVertex(2,2,face);
	newQuadVertex[1][0] = computeQuadVertex(2,1,face);

	initSubPatches(edge,face,newQuadVertex,child);
      }
    };


  class __aligned(64) BSplinePatch : public BSplinePatchT<Vec3fa> 
  {
  public:

    BSplinePatch () {}

    __forceinline void init( FinalQuad& quad ) const
    {
      quad.vtx[0] = v[1][1];
      quad.vtx[1] = v[1][2];
      quad.vtx[2] = v[2][2];
      quad.vtx[3] = v[2][1];
    };

    __forceinline Vec3fa limitVtx0() const { return computeLimitVertex(1,1); }
    __forceinline Vec3fa limitVtx1() const { return computeLimitVertex(1,2); }
    __forceinline Vec3fa limitVtx2() const { return computeLimitVertex(2,2); }
    __forceinline Vec3fa limitVtx3() const { return computeLimitVertex(2,1); }

    __forceinline void init_limit( FinalQuad& quad ) const
    {

      const Vec3fa limit_v0 = computeLimitVertex(1,1);
      const Vec3fa limit_v1 = computeLimitVertex(1,2);
      const Vec3fa limit_v2 = computeLimitVertex(2,2);
      const Vec3fa limit_v3 = computeLimitVertex(2,1);
      
      /* const Vec3fa limit_normal0 = computeLimitNormal(1,1); */
      /* const Vec3fa limit_normal1 = computeLimitNormal(1,2); */
      /* const Vec3fa limit_normal2 = computeLimitNormal(2,2); */
      /* const Vec3fa limit_normal3 = computeLimitNormal(2,1); */

      quad.vtx[0] = limit_v0;
      quad.vtx[1] = limit_v1;
      quad.vtx[2] = limit_v2;
      quad.vtx[3] = limit_v3;
    };


    __forceinline void init(const CatmullClarkPatch &irreg_patch)
    {
      assert( irreg_patch.isRegular() );

      v[1][1] = irreg_patch.ring[0].vtx;
      v[0][1] = irreg_patch.ring[0].ring[6];
      v[0][0] = irreg_patch.ring[0].ring[5];
      v[1][0] = irreg_patch.ring[0].ring[4];

      v[1][2] = irreg_patch.ring[1].vtx;
      v[1][3] = irreg_patch.ring[1].ring[6];
      v[0][3] = irreg_patch.ring[1].ring[5];
      v[0][2] = irreg_patch.ring[1].ring[4];

      v[2][2] = irreg_patch.ring[2].vtx;
      v[3][2] = irreg_patch.ring[2].ring[6];
      v[3][3] = irreg_patch.ring[2].ring[5];
      v[2][3] = irreg_patch.ring[2].ring[4];

      v[2][1] = irreg_patch.ring[3].vtx;
      v[2][0] = irreg_patch.ring[3].ring[6];
      v[3][0] = irreg_patch.ring[3].ring[5];      
      v[3][1] = irreg_patch.ring[3].ring[4];
    }

    __forceinline void init(const SubdivMesh::HalfEdge *const first_half_edge,
			    const Vec3fa *const vertices)
    {
      CatmullClarkPatch ipatch( first_half_edge, vertices );
      init( ipatch );
    }

    __forceinline BBox3fa bounds() const
    {
      const Vec3fa *const cv = &v[0][0];
      BBox3fa bounds ( cv[0] );
      for (size_t i = 1; i<16 ; i++)
	bounds.extend( cv[i] );
      return bounds;
    }

#if defined(__MIC__)

    __forceinline mic_f getRow(const size_t i) const
    {
      return load16f(&v[i][0]);
    }

    __forceinline void prefetchData() const
    {
      prefetch<PFHINT_L1>(&v[0][0]);
      prefetch<PFHINT_L1>(&v[1][0]);
      prefetch<PFHINT_L1>(&v[2][0]);
      prefetch<PFHINT_L1>(&v[3][0]);
    }
#endif

    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      const Vec4f v_n = CubicBSplineCurve::eval(vv);

      const Vec3fa_t curve0 = v_n[0] * v[0][0] + v_n[1] * v[1][0] + v_n[2] * v[2][0] + v_n[3] * v[3][0];
      const Vec3fa_t curve1 = v_n[0] * v[0][1] + v_n[1] * v[1][1] + v_n[2] * v[2][1] + v_n[3] * v[3][1];
      const Vec3fa_t curve2 = v_n[0] * v[0][2] + v_n[1] * v[1][2] + v_n[2] * v[2][2] + v_n[3] * v[3][2];
      const Vec3fa_t curve3 = v_n[0] * v[0][3] + v_n[1] * v[1][3] + v_n[2] * v[2][3] + v_n[3] * v[3][3];

      const Vec4f u_n = CubicBSplineCurve::eval(uu);

      return (u_n[0] * curve0 + u_n[1] * curve1 + u_n[2] * curve2 + u_n[3] * curve3) * 1.0f/36.0f;
    }


    __forceinline Vec3fa tangentU(const float uu, const float vv) const
    {
      const Vec4f v_n = CubicBSplineCurve::eval(vv);

      const Vec3fa_t curve0 = v_n[0] * v[0][0] + v_n[1] * v[1][0] + v_n[2] * v[2][0] + v_n[3] * v[3][0];
      const Vec3fa_t curve1 = v_n[0] * v[0][1] + v_n[1] * v[1][1] + v_n[2] * v[2][1] + v_n[3] * v[3][1];
      const Vec3fa_t curve2 = v_n[0] * v[0][2] + v_n[1] * v[1][2] + v_n[2] * v[2][2] + v_n[3] * v[3][2];
      const Vec3fa_t curve3 = v_n[0] * v[0][3] + v_n[1] * v[1][3] + v_n[2] * v[2][3] + v_n[3] * v[3][3];

      const Vec4f u_n = CubicBSplineCurve::derivative(uu);

      return (u_n[0] * curve0 + u_n[1] * curve1 + u_n[2] * curve2 + u_n[3] * curve3); 
    }

    __forceinline Vec3fa tangentV(const float uu, const float vv) const
    {
      const Vec4f v_n = CubicBSplineCurve::derivative(vv);

      const Vec3fa_t curve0 = v_n[0] * v[0][0] + v_n[1] * v[1][0] + v_n[2] * v[2][0] + v_n[3] * v[3][0];
      const Vec3fa_t curve1 = v_n[0] * v[0][1] + v_n[1] * v[1][1] + v_n[2] * v[2][1] + v_n[3] * v[3][1];
      const Vec3fa_t curve2 = v_n[0] * v[0][2] + v_n[1] * v[1][2] + v_n[2] * v[2][2] + v_n[3] * v[3][2];
      const Vec3fa_t curve3 = v_n[0] * v[0][3] + v_n[1] * v[1][3] + v_n[2] * v[2][3] + v_n[3] * v[3][3];

      const Vec4f u_n = CubicBSplineCurve::eval(uu);

      return (u_n[0] * curve0 + u_n[1] * curve1 + u_n[2] * curve2 + u_n[3] * curve3); 
    }



#if defined(__MIC__)


    __forceinline mic_f normal4(const float uu, const float vv) const
    {
      const mic4f v_e_d = CubicBSplineCurve::eval_derivative(mic_f(vv),0x00ff);       // ev,ev,dv,dv

      const mic_f curve0 = v_e_d[0] * broadcast4to16f(&v[0][0]) + v_e_d[1] * broadcast4to16f(&v[1][0]) + v_e_d[2] * broadcast4to16f(&v[2][0]) + v_e_d[3] * broadcast4to16f(&v[3][0]);
      const mic_f curve1 = v_e_d[0] * broadcast4to16f(&v[0][1]) + v_e_d[1] * broadcast4to16f(&v[1][1]) + v_e_d[2] * broadcast4to16f(&v[2][1]) + v_e_d[3] * broadcast4to16f(&v[3][1]);
      const mic_f curve2 = v_e_d[0] * broadcast4to16f(&v[0][2]) + v_e_d[1] * broadcast4to16f(&v[1][2]) + v_e_d[2] * broadcast4to16f(&v[2][2]) + v_e_d[3] * broadcast4to16f(&v[3][2]);
      const mic_f curve3 = v_e_d[0] * broadcast4to16f(&v[0][3]) + v_e_d[1] * broadcast4to16f(&v[1][3]) + v_e_d[2] * broadcast4to16f(&v[2][3]) + v_e_d[3] * broadcast4to16f(&v[3][3]);

      const mic4f u_e_d = CubicBSplineCurve::eval_derivative(mic_f(uu),0xff00);       // du,du,eu,eu

      const mic_f tangentUV = (u_e_d[0] * curve0 + u_e_d[1] * curve1 + u_e_d[2] * curve2 + u_e_d[3] * curve3); // tu, tu, tv, tv
      
      const mic_f tangentU = permute<0,0,0,0>(tangentUV);
      const mic_f tangentV = permute<2,2,2,2>(tangentUV);

      /* DBG_PRINT( tangentUV ); */
      /* DBG_PRINT( tangentU ); */
      /* DBG_PRINT( tangentV ); */

      const mic_f n = lcross_xyz(tangentU,tangentV);
      return n;
    }

    __forceinline mic_f eval4(const mic_f uu, const mic_f vv) const
    {
      const mic4f v_n = CubicBSplineCurve::eval(vv); //FIXME: precompute in table

      const mic_f curve0 = v_n[0] * broadcast4to16f(&v[0][0]) + v_n[1] * broadcast4to16f(&v[1][0]) + v_n[2] * broadcast4to16f(&v[2][0]) + v_n[3] * broadcast4to16f(&v[3][0]);
      const mic_f curve1 = v_n[0] * broadcast4to16f(&v[0][1]) + v_n[1] * broadcast4to16f(&v[1][1]) + v_n[2] * broadcast4to16f(&v[2][1]) + v_n[3] * broadcast4to16f(&v[3][1]);
      const mic_f curve2 = v_n[0] * broadcast4to16f(&v[0][2]) + v_n[1] * broadcast4to16f(&v[1][2]) + v_n[2] * broadcast4to16f(&v[2][2]) + v_n[3] * broadcast4to16f(&v[3][2]);
      const mic_f curve3 = v_n[0] * broadcast4to16f(&v[0][3]) + v_n[1] * broadcast4to16f(&v[1][3]) + v_n[2] * broadcast4to16f(&v[2][3]) + v_n[3] * broadcast4to16f(&v[3][3]);

      const mic4f u_n = CubicBSplineCurve::eval(uu); //FIXME: precompute in table

      return (u_n[0] * curve0 + u_n[1] * curve1 + u_n[2] * curve2 + u_n[3] * curve3) * mic_f(1.0f/36.0f);
    }

    __forceinline mic3f eval16(const mic_f &uu, 
			       const mic_f &vv,
			       const mic4f &u_n,
			       const mic4f &v_n) const
    {
      const mic_f curve0_x = v_n[0] * mic_f(v[0][0].x) + v_n[1] * mic_f(v[1][0].x) + v_n[2] * mic_f(v[2][0].x) + v_n[3] * mic_f(v[3][0].x);
      const mic_f curve1_x = v_n[0] * mic_f(v[0][1].x) + v_n[1] * mic_f(v[1][1].x) + v_n[2] * mic_f(v[2][1].x) + v_n[3] * mic_f(v[3][1].x);
      const mic_f curve2_x = v_n[0] * mic_f(v[0][2].x) + v_n[1] * mic_f(v[1][2].x) + v_n[2] * mic_f(v[2][2].x) + v_n[3] * mic_f(v[3][2].x);
      const mic_f curve3_x = v_n[0] * mic_f(v[0][3].x) + v_n[1] * mic_f(v[1][3].x) + v_n[2] * mic_f(v[2][3].x) + v_n[3] * mic_f(v[3][3].x);
      const mic_f x = (u_n[0] * curve0_x + u_n[1] * curve1_x + u_n[2] * curve2_x + u_n[3] * curve3_x) * mic_f(1.0f/36.0f);


      const mic_f curve0_y = v_n[0] * mic_f(v[0][0].y) + v_n[1] * mic_f(v[1][0].y) + v_n[2] * mic_f(v[2][0].y) + v_n[3] * mic_f(v[3][0].y);
      const mic_f curve1_y = v_n[0] * mic_f(v[0][1].y) + v_n[1] * mic_f(v[1][1].y) + v_n[2] * mic_f(v[2][1].y) + v_n[3] * mic_f(v[3][1].y);
      const mic_f curve2_y = v_n[0] * mic_f(v[0][2].y) + v_n[1] * mic_f(v[1][2].y) + v_n[2] * mic_f(v[2][2].y) + v_n[3] * mic_f(v[3][2].y);
      const mic_f curve3_y = v_n[0] * mic_f(v[0][3].y) + v_n[1] * mic_f(v[1][3].y) + v_n[2] * mic_f(v[2][3].y) + v_n[3] * mic_f(v[3][3].y);
      const mic_f y = (u_n[0] * curve0_y + u_n[1] * curve1_y + u_n[2] * curve2_y + u_n[3] * curve3_y) * mic_f(1.0f/36.0f);
      

      const mic_f curve0_z = v_n[0] * mic_f(v[0][0].z) + v_n[1] * mic_f(v[1][0].z) + v_n[2] * mic_f(v[2][0].z) + v_n[3] * mic_f(v[3][0].z);
      const mic_f curve1_z = v_n[0] * mic_f(v[0][1].z) + v_n[1] * mic_f(v[1][1].z) + v_n[2] * mic_f(v[2][1].z) + v_n[3] * mic_f(v[3][1].z);
      const mic_f curve2_z = v_n[0] * mic_f(v[0][2].z) + v_n[1] * mic_f(v[1][2].z) + v_n[2] * mic_f(v[2][2].z) + v_n[3] * mic_f(v[3][2].z);
      const mic_f curve3_z = v_n[0] * mic_f(v[0][3].z) + v_n[1] * mic_f(v[1][3].z) + v_n[2] * mic_f(v[2][3].z) + v_n[3] * mic_f(v[3][3].z);
      const mic_f z = (u_n[0] * curve0_z + u_n[1] * curve1_z + u_n[2] * curve2_z + u_n[3] * curve3_z) * mic_f(1.0f/36.0f);

      return mic3f(x,y,z);
    }
    
    __forceinline mic3f eval16(const mic_f uu, const mic_f vv) const
    {
      const mic4f v_n = CubicBSplineCurve::eval(vv); //FIXME: precompute in table
      const mic4f u_n = CubicBSplineCurve::eval(uu); //FIXME: precompute in table
      return eval16(uu,vv,u_n,v_n);
    }

    
    __forceinline mic3f tangentU16(const mic_f uu, const mic_f vv) const
    {
      const mic4f v_n = CubicBSplineCurve::derivative(vv); 
      const mic4f u_n = CubicBSplineCurve::eval(uu); 
      return eval16(uu,vv,u_n,v_n);      
    }

    __forceinline mic3f tangentV16(const mic_f uu, const mic_f vv) const
    {
      const mic4f v_n = CubicBSplineCurve::eval(vv); 
      const mic4f u_n = CubicBSplineCurve::derivative(uu); 
      return eval16(uu,vv,u_n,v_n);      
    }

    __forceinline mic3f normal16(const mic_f uu, const mic_f vv) const
    {
      const mic3f tU = tangentU16(uu,vv);
      const mic3f tV = tangentV16(uu,vv);
      return cross(tU,tV);
    }

#endif

    __forceinline Vec3fa normal(const float uu, const float vv) const
    {
      const Vec3fa tu = tangentU(uu,vv);
      const Vec3fa tv = tangentV(uu,vv);
      return cross(tu,tv);
    }   

    friend __forceinline std::ostream &operator<<(std::ostream &o, const BSplinePatch &p)
    {
      for (size_t y=0;y<4;y++)
	for (size_t x=0;x<4;x++)
	  o << "[" << y << "][" << x << "] " << p.v[y][x] << std::endl;
      return o;
    } 
  };
}


