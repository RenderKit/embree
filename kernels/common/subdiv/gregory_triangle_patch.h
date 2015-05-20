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

#include "catmullclark_patch.h"

namespace embree
{  
  class __aligned(64) GregoryTrianglePatch 
  {
  public:

    Vec3fa v[4][4];
        
    GregoryTrianglePatch() {
      memset(this,0,sizeof(GregoryTrianglePatch));
    }
     
    Vec3fa& p0() { return v[0][0]; }
    Vec3fa& p1() { return v[0][3]; }
    Vec3fa& p2() { return v[3][3]; }
    //Vec3fa& p3() { return v[3][0]; }
    
    Vec3fa& e0_p() { return v[0][1]; }
    Vec3fa& e0_m() { return v[1][0]; }
    Vec3fa& e1_p() { return v[1][3]; }
    Vec3fa& e1_m() { return v[0][2]; }
    Vec3fa& e2_p() { return v[3][2]; }
    Vec3fa& e2_m() { return v[2][3]; }
    //Vec3fa& e3_p() { return v[2][0]; }
    //Vec3fa& e3_m() { return v[3][1]; }
    
    Vec3fa& f0_p() { return v[1][1]; }
    Vec3fa& f1_p() { return v[1][2]; }
    Vec3fa& f2_p() { return v[2][2]; }
    //Vec3fa& f3_p() { return v[2][1]; }
    Vec3fa& f0_m() { return v[3][0]; }
    Vec3fa& f1_m() { return v[2][0]; }
    Vec3fa& f2_m() { return v[1][0]; }
    //Vec3fa& f3_m() { return f[1][0]; }
    
    const Vec3fa& p0() const { return v[0][0]; }
    const Vec3fa& p1() const { return v[0][3]; }
    const Vec3fa& p2() const { return v[3][3]; }
    
    const Vec3fa& e0_p() const { return v[0][1]; }
    const Vec3fa& e1_p() const { return v[1][3]; }
    const Vec3fa& e2_p() const { return v[3][2]; }

    const Vec3fa& e0_m() const { return v[1][0]; }
    const Vec3fa& e1_m() const { return v[0][2]; }
    const Vec3fa& e2_m() const { return v[2][3]; }
    
    const Vec3fa& f0_p() const { return v[1][1]; }
    const Vec3fa& f1_p() const { return v[1][2]; }
    const Vec3fa& f2_p() const { return v[2][2]; }

    const Vec3fa& f0_m() const { return v[3][0]; }
    const Vec3fa& f1_m() const { return v[2][0]; }
    const Vec3fa& f2_m() const { return v[1][0]; }
    
    
    Vec3fa initCornerVertex(const CatmullClarkPatch3fa &irreg_patch, const size_t index)
    {
      return irreg_patch.ring[index].getLimitVertex();
    }
    
    
    Vec3fa initPositiveEdgeVertex(const CatmullClarkPatch3fa &irreg_patch, const size_t index, const Vec3fa &p_vtx)
    {
      const Vec3fa tangent = irreg_patch.ring[index].getLimitTangent();
#if 0
      const float n = irreg_patch.ring[index].face_valence;
      const float alpha = 1.0f/16.0f * (5.0f + cosf(2.0f*M_PI/n) + cosf(M_PI/n) * sqrtf(18.0f+2.0f*cosf(2.0f*M_PI/n)));
      return 2.0f/3.0f * alpha * tangent + p_vtx;
#else
      return 1.0f/3.0f * tangent + p_vtx;
#endif
    }
    
    Vec3fa initNegativeEdgeVertex(const CatmullClarkPatch3fa &irreg_patch, const size_t index, const Vec3fa &p_vtx)
    {
      const Vec3fa tangent = irreg_patch.ring[index].getSecondLimitTangent();
#if 0
      const float n = irreg_patch.ring[index].face_valence;
      const float alpha = 1.0f/16.0f * (5.0f + cosf(2.0f*M_PI/n) + cosf(M_PI/n) * sqrtf(18.0f+2.0f*cosf(2.0f*M_PI/n)));
      return 2.0f/3.0f * alpha * tangent + p_vtx;
#else
      return 1.0f/3.0f * tangent + p_vtx;
#endif
    }
    
    
    void initFaceVertex(const CatmullClarkPatch3fa &irreg_patch,
			const size_t index,
			const Vec3fa &p_vtx,
			const Vec3fa &e0_p_vtx,
			const Vec3fa &e1_m_vtx,
			const unsigned int face_valence_p1,
			const Vec3fa &e0_m_vtx,
			const Vec3fa &e3_p_vtx,
			const unsigned int face_valence_p3,
			Vec3fa &f_p_vtx,
			Vec3fa &f_m_vtx)
    {
      const unsigned int face_valence = irreg_patch.ring[index].face_valence;
      const unsigned int edge_valence = irreg_patch.ring[index].edge_valence;
      const unsigned int border_index = irreg_patch.ring[index].border_index;
      
      const Vec3fa &vtx     = irreg_patch.ring[index].vtx;
      const Vec3fa e_i      = irreg_patch.ring[index].getEdgeCenter( 0 );
      const Vec3fa c_i_m_1  = irreg_patch.ring[index].getQuadCenter( 0 );
      const Vec3fa e_i_m_1  = irreg_patch.ring[index].getEdgeCenter( 1 );
      
      Vec3fa c_i, e_i_p_1;
      const bool hasHardEdge = \
        std::isinf(irreg_patch.ring[index].vertex_crease_weight) &&
        std::isinf(irreg_patch.ring[index].crease_weight[0]);
                
      if (unlikely(border_index == edge_valence-2) || hasHardEdge)
      {
        /* mirror quad center and edge mid-point */
        c_i     = c_i_m_1 + 2 * (e_i - c_i_m_1);
        e_i_p_1 = e_i_m_1 + 2 * (vtx - e_i_m_1);
      }
      else
      {
        c_i     = irreg_patch.ring[index].getQuadCenter( face_valence-1 );
        e_i_p_1 = irreg_patch.ring[index].getEdgeCenter( face_valence-1 );
      }
      
      Vec3fa c_i_m_2, e_i_m_2;
      if (unlikely(border_index == 2 || face_valence == 2 || hasHardEdge))
      {
        /* mirror quad center and edge mid-point */
        c_i_m_2  = c_i_m_1 + 2 * (e_i_m_1 - c_i_m_1);
        e_i_m_2  = e_i + 2 * (vtx - e_i);	  
      }
      else
      {
        c_i_m_2  = irreg_patch.ring[index].getQuadCenter( 1 );
        e_i_m_2  = irreg_patch.ring[index].getEdgeCenter( 2 );
      }      
      
      const float d = 4.0f;
      const float c     = cosf(2.0*M_PI/(float)face_valence);
      const float c_e_p = cosf(2.0*M_PI/(float)face_valence_p1);
      const float c_e_m = cosf(2.0*M_PI/(float)face_valence_p3);
      
      const Vec3fa r_e_p = 1.0f/3.0f * (e_i_m_1 - e_i_p_1) + 2.0f/3.0f * (c_i_m_1 - c_i);
      
      f_p_vtx =  1.0f / d * (c_e_p * p_vtx + (d - 2.0f*c - c_e_p) * e0_p_vtx + 2.0f*c* e1_m_vtx + r_e_p);
      
      const Vec3fa r_e_m = 1.0f/3.0f * (e_i - e_i_m_2) + 2.0f/3.0f * (c_i_m_1 - c_i_m_2);
      
      f_m_vtx = 1.0f / d * (c_e_m * p_vtx + (d - 2.0f*c - c_e_m) * e0_m_vtx + 2.0f*c* e3_p_vtx + r_e_m);      
    }


    __noinline void init(const CatmullClarkPatch3fa& patch)
    {
      assert( patch.ring[0].hasValidPositions() );
      assert( patch.ring[1].hasValidPositions() );
      assert( patch.ring[2].hasValidPositions() );

      p0() = initCornerVertex(patch,0);
      p1() = initCornerVertex(patch,1);
      p2() = initCornerVertex(patch,2);

      e0_p() = initPositiveEdgeVertex(patch,0, p0());
      e1_p() = initPositiveEdgeVertex(patch,1, p1());
      e2_p() = initPositiveEdgeVertex(patch,2, p2());

      e0_m() = initNegativeEdgeVertex(patch,0, p0());
      e1_m() = initNegativeEdgeVertex(patch,1, p1());
      e2_m() = initNegativeEdgeVertex(patch,2, p2());

      const unsigned int face_valence_p0 = patch.ring[0].face_valence;
      const unsigned int face_valence_p1 = patch.ring[1].face_valence;
      const unsigned int face_valence_p2 = patch.ring[2].face_valence;
      
      initFaceVertex(patch,0,p0(),e0_p(),e1_m(),face_valence_p1,e0_m(),e2_p(),face_valence_p2,f0_p(),f0_m() );
      initFaceVertex(patch,1,p1(),e1_p(),e2_m(),face_valence_p2,e1_m(),e0_p(),face_valence_p0,f1_p(),f1_m() );
      initFaceVertex(patch,2,p2(),e2_p(),e0_m(),face_valence_p0,e2_m(),e1_p(),face_valence_p1,f2_p(),f2_m() );
    }
    
    __noinline void init(const GeneralCatmullClarkPatch3fa& patch)
    {
      assert(patch.size() == 4);
      CatmullClarkPatch3fa qpatch; patch.init(qpatch);
      init(qpatch);
    }

    
    __forceinline void exportControlPoints( Vec3fa matrix[4][4] ) const
    {
      for (size_t y=0;y<4;y++)
	for (size_t x=0;x<4;x++)
	  matrix[y][x] = (Vec3fa_t)v[y][x];
      
    }
        
    template<class T, class S>
      static __forceinline T deCasteljau_t(const S &uu, const T &v0, const T &v1, const T &v2, const T &v3)
    {
      const S one_minus_uu = 1.0f - uu;      
      const T v0_1 = one_minus_uu * v0   + uu * v1;
      const T v1_1 = one_minus_uu * v1   + uu * v2;
      const T v2_1 = one_minus_uu * v2   + uu * v3;      
      const T v0_2 = one_minus_uu * v0_1 + uu * v1_1;
      const T v1_2 = one_minus_uu * v1_1 + uu * v2_1;      
      const T v0_3 = one_minus_uu * v0_2 + uu * v1_2;
      return v0_3;
    }
    
    template<class T, class S>
      static __forceinline T deCasteljau_tangent_t(const S &uu, const T &v0, const T &v1, const T &v2, const T &v3)
    {
      const S one_minus_uu = 1.0f - uu;      
      const T v0_1         = one_minus_uu * v0   + uu * v1;
      const T v1_1         = one_minus_uu * v1   + uu * v2;
      const T v2_1         = one_minus_uu * v2   + uu * v3;      
      const T v0_2         = one_minus_uu * v0_1 + uu * v1_1;
      const T v1_2         = one_minus_uu * v1_1 + uu * v2_1;      
      return v1_2 - v0_2;      
    }
    
        
    template<class M, class T>
      static __forceinline Vec3<T> eval_t(const GregoryTrianglePatch &tpatch,
				   const T &uu,
				   const T &vv)
    {
      const T ww = T(1.0f) - uu - vv;
      const M m_border = (uu == 0.0f) | (uu == 1.0f) | (vv == 0.0f) | (vv == 1.0f) | (ww == 0.0f) | (ww == 1.0f);
      
      const Vec3<T> f0_p = Vec3<T>(tpatch.f0_p().x,tpatch.f0_p().y,tpatch.f0_p().z);
      const Vec3<T> f1_p = Vec3<T>(tpatch.f1_p().x,tpatch.f1_p().y,tpatch.f1_p().z);
      const Vec3<T> f2_p = Vec3<T>(tpatch.f2_p().x,tpatch.f2_p().y,tpatch.f2_p().z);

      const Vec3<T> f0_m = Vec3<T>(tpatch.f0_m().x,tpatch.f0_m().y,tpatch.f0_m().z);
      const Vec3<T> f1_m = Vec3<T>(tpatch.f1_m().x,tpatch.f1_m().y,tpatch.f1_m().z);
      const Vec3<T> f2_m = Vec3<T>(tpatch.f2_m().x,tpatch.f2_m().y,tpatch.f2_m().z);
      
      
      
      const T inv_uu_vv = rcp(uu+vv);
      const T inv_vv_ww = rcp(vv+ww);
      const T inv_ww_uu = rcp(ww+uu);
      
      const Vec3<T> f0_i = (ww * f0_m + vv * f0_p) * inv_vv_ww;
      const Vec3<T> f1_i = (uu * f1_m + ww * f1_p) * inv_ww_uu;
      const Vec3<T> f2_i = (vv * f2_m + uu * f2_p) * inv_uu_vv;

      
      const Vec3<T> F0( select(m_border,T(zero),f0_i.x), select(m_border,T(zero),f0_i.y), select(m_border,T(zero),f0_i.z) );
      const Vec3<T> F1( select(m_border,T(zero),f1_i.x), select(m_border,T(zero),f1_i.y), select(m_border,T(zero),f1_i.z) );
      const Vec3<T> F2( select(m_border,T(zero),f2_i.x), select(m_border,T(zero),f2_i.y), select(m_border,T(zero),f2_i.z) );

      const T uu3     = uu*uu*uu;
      const T vv3     = vv*vv*vv;
      const T ww3     = ww*ww*ww;
      const T e0e1   = 3.0f * (uu * vv) * (uu + vv);
      const T e1e2   = 3.0f * (vv * ww) * (vv + ww);
      const T e2e0   = 3.0f * (ww * uu) * (ww + uu);
      const T f0f1f2 = 12.0f * (uu * vv * ww);
      
      const T x = 
	(uu3 * tpatch.p0().x + vv3 * tpatch.p1().x + ww3 * tpatch.p2().x) +
	(e0e1 * (uu * tpatch.e0_p().x + vv * tpatch.e1_m().x)) +
	(e1e2 * (vv * tpatch.e1_p().x + ww * tpatch.e2_m().x)) +
	(e2e0 * (ww * tpatch.e2_p().x + uu * tpatch.e0_m().x)) +
	(f0f1f2 * (uu * F0.x + vv * F1.x + ww * F2.x));

      const T y = 
	(uu3 * tpatch.p0().y + vv3 * tpatch.p1().y + ww3 * tpatch.p2().y) +
	(e0e1 * (uu * tpatch.e0_p().y + vv * tpatch.e1_m().y)) +
	(e1e2 * (vv * tpatch.e1_p().y + ww * tpatch.e2_m().y)) +
	(e2e0 * (ww * tpatch.e2_p().y + uu * tpatch.e0_m().y)) +
	(f0f1f2 * (uu * F0.y + vv * F1.y + ww * F2.y));

      const T z = 
	(uu3 * tpatch.p0().z + vv3 * tpatch.p1().z + ww3 * tpatch.p2().z) +
	(e0e1 * (uu * tpatch.e0_p().z + vv * tpatch.e1_m().z)) +
	(e1e2 * (vv * tpatch.e1_p().z + ww * tpatch.e2_m().z)) +
	(e2e0 * (ww * tpatch.e2_p().z + uu * tpatch.e0_m().z)) +
	(f0f1f2 * (uu * F0.z + vv * F1.z + ww * F2.z));

      return Vec3<T>(x,y,z);
    }
    
    


       
#if !defined(__MIC__)
    
#if defined(__AVX__)    
    
    static __forceinline avx3f eval8  (const Vec3fa matrix[4][4], const avxf &uu, const avxf &vv) 
    {
      const GregoryTrianglePatch &tpatch = *(GregoryTrianglePatch*)matrix;
      return eval_t<avxb,avxf>(tpatch,uu,vv); 
    }
    //static __forceinline avx3f normal8(const Vec3fa matrix[4][4], const avxf &uu, const avxf &vv) { return normal_t<avxb,avxf>(matrix,uu,vv); }
    
#endif
    
    static __forceinline sse3f eval4  (const Vec3fa matrix[4][4], const ssef &uu, const ssef &vv) 
    {
      const GregoryTrianglePatch &tpatch = *(GregoryTrianglePatch*)matrix;
      return eval_t<sseb,ssef>(tpatch,uu,vv); 
    }
    //static __forceinline sse3f normal4(const Vec3fa matrix[4][4], const ssef &uu, const ssef &vv) { return normal_t<sseb,ssef>(matrix,uu,vv); }
    
#else    
    
#endif
           
    static __forceinline Vec3fa normal(const Vec3fa matrix[4][4],
				       const float uu,
				       const float vv) 
    {
      return Vec3fa( zero );
    }
    
    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      return Vec3fa( zero );
    }
    
    __forceinline BBox3fa bounds() const
    {
      BBox3fa bounds ( empty );
      bounds.extend(p0());
      bounds.extend(p1());
      bounds.extend(p2());

      bounds.extend(e0_p());
      bounds.extend(e0_m());
      bounds.extend(e1_p());
      bounds.extend(e1_m());
      bounds.extend(e2_p());
      bounds.extend(e2_m());

      bounds.extend(f0_p());
      bounds.extend(f0_m());
      bounds.extend(f1_p());
      bounds.extend(f1_m());
      bounds.extend(f2_p());
      bounds.extend(f2_m());
      return bounds;
    }
    
    friend std::ostream &operator<<(std::ostream &o, const GregoryTrianglePatch&g)
    {
      o << "p0 " << g.p0() << std::endl;
      o << "p1 " << g.p1() << std::endl;
      o << "p2 " << g.p2() << std::endl;

      o << "e0_p " << g.e0_p() << std::endl;
      o << "e0_m " << g.e0_m() << std::endl;

      o << "e1_p " << g.e1_p() << std::endl;
      o << "e1_m " << g.e1_m() << std::endl;

      o << "e2_p " << g.e2_p() << std::endl;
      o << "e2_m " << g.e2_m() << std::endl;

      o << "f0_p " << g.f0_p() << std::endl;
      o << "f0_m " << g.f0_m() << std::endl;

      o << "f1_p " << g.f1_p() << std::endl;
      o << "f1_m " << g.f1_m() << std::endl;

      o << "f2_p " << g.f2_p() << std::endl;
      o << "f2_m " << g.f2_m() << std::endl;

      return o;
    } 
  };
}
