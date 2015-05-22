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
  template<typename Vertex, typename Vertex_t = Vertex>  
  class __aligned(64) GregoryTrianglePatchT 
  {
     typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
   
  public:

    Vertex v[4][4];
        
    GregoryTrianglePatchT() {
      memset(this,0,sizeof(GregoryTrianglePatchT));
    }
     
    Vertex& p0() { return v[0][0]; }
    Vertex& p1() { return v[0][3]; }
    Vertex& p2() { return v[3][3]; }
    //Vertex& p3() { return v[3][0]; }
    
    Vertex& e0_p() { return v[0][1]; }
    Vertex& e0_m() { return v[1][0]; }
    Vertex& e1_p() { return v[1][3]; }
    Vertex& e1_m() { return v[0][2]; }
    Vertex& e2_p() { return v[3][2]; }
    Vertex& e2_m() { return v[2][3]; }
    //Vertex& e3_p() { return v[2][0]; }
    //Vertex& e3_m() { return v[3][1]; }
    
    Vertex& f0_p() { return v[1][1]; }
    Vertex& f1_p() { return v[1][2]; }
    Vertex& f2_p() { return v[2][2]; }
    //Vertex& f3_p() { return v[2][1]; }
    Vertex& f0_m() { return v[3][0]; }
    Vertex& f1_m() { return v[2][0]; }
    Vertex& f2_m() { return v[1][0]; }
    //Vertex& f3_m() { return f[1][0]; }
    
    const Vertex& p0() const { return v[0][0]; }
    const Vertex& p1() const { return v[0][3]; }
    const Vertex& p2() const { return v[3][3]; }
    
    const Vertex& e0_p() const { return v[0][1]; }
    const Vertex& e1_p() const { return v[1][3]; }
    const Vertex& e2_p() const { return v[3][2]; }

    const Vertex& e0_m() const { return v[1][0]; }
    const Vertex& e1_m() const { return v[0][2]; }
    const Vertex& e2_m() const { return v[2][3]; }
    
    const Vertex& f0_p() const { return v[1][1]; }
    const Vertex& f1_p() const { return v[1][2]; }
    const Vertex& f2_p() const { return v[2][2]; }

    const Vertex& f0_m() const { return v[3][0]; }
    const Vertex& f1_m() const { return v[2][0]; }
    const Vertex& f2_m() const { return v[1][0]; }
    
    void computeGregoryPatchFacePoints(const unsigned int face_valence,
				       const Vertex& r_e_p, 
				       const Vertex& r_e_m, 					 
				       const Vertex& p_vtx, 
				       const Vertex& e0_p_vtx, 
				       const Vertex& e1_m_vtx, 
				       const unsigned int face_valence_p1,
				       const Vertex& e0_m_vtx,	
				       const Vertex& e3_p_vtx,	
				       const unsigned int face_valence_p3,
				       Vertex& f_p_vtx, 
				       Vertex& f_m_vtx,
                                       const float d = 3.0f)
    {
      const float c     = cosf(2.0*M_PI/(float)face_valence);
      const float c_e_p = cosf(2.0*M_PI/(float)face_valence_p1);
      const float c_e_m = cosf(2.0*M_PI/(float)face_valence_p3);
      
      f_p_vtx = 1.0f / d * (c_e_p * p_vtx + (d - 2.0f*c - c_e_p) * e0_p_vtx + 2.0f*c* e1_m_vtx + r_e_p);      
      f_m_vtx = 1.0f / d * (c_e_m * p_vtx + (d - 2.0f*c - c_e_m) * e0_m_vtx + 2.0f*c* e3_p_vtx + r_e_m);      
      f_p_vtx = 1.0f / d * (c_e_p * p_vtx + (d - 2.0f*c - c_e_p) * e0_p_vtx + 2.0f*c* e1_m_vtx + r_e_p);      
      f_m_vtx = 1.0f / d * (c_e_m * p_vtx + (d - 2.0f*c - c_e_m) * e0_m_vtx + 2.0f*c* e3_p_vtx + r_e_m);
    }


    __noinline void init(const GeneralCatmullClarkPatch& patch)
    {
      assert(patch.size() == 3);
      const float face_valence_p0 = patch.ring[0].face_valence;
      const float face_valence_p1 = patch.ring[1].face_valence;
      const float face_valence_p2 = patch.ring[2].face_valence;

      Vertex p0_r_p, p0_r_m;
      patch.ring[0].computeGregoryPatchEdgePoints( p0(), e0_p(), e0_m(), p0_r_p, p0_r_m );

      Vertex p1_r_p, p1_r_m;
      patch.ring[1].computeGregoryPatchEdgePoints( p1(), e1_p(), e1_m(), p1_r_p, p1_r_m );
      
      Vertex p2_r_p, p2_r_m;
      patch.ring[2].computeGregoryPatchEdgePoints( p2(), e2_p(), e2_m(), p2_r_p, p2_r_m );

      /* PRINT(p0()); */
      /* PRINT(p1()); */
      /* PRINT(p2()); */

      /* PRINT(e0_p()); */
      /* PRINT(e0_m()); */
      /* PRINT(e1_p()); */
      /* PRINT(e1_m()); */
      /* PRINT(e2_p()); */
      /* PRINT(e2_m()); */

      computeGregoryPatchFacePoints(face_valence_p0, p0_r_p, p0_r_m, p0(), e0_p(), e1_m(), face_valence_p1, e0_m(), e2_p(), face_valence_p2, f0_p(), f0_m() );
      computeGregoryPatchFacePoints(face_valence_p1, p1_r_p, p1_r_m, p1(), e1_p(), e2_m(), face_valence_p2, e1_m(), e0_p(), face_valence_p0, f1_p(), f1_m() );
      computeGregoryPatchFacePoints(face_valence_p2, p2_r_p, p2_r_m, p2(), e2_p(), e0_m(), face_valence_p0, e2_m(), e1_p(), face_valence_p1, f2_p(), f2_m() );

      /* PRINT(f0_p()); */
      /* PRINT(f0_m()); */
      /* PRINT(f1_p()); */
      /* PRINT(f1_m()); */
      /* PRINT(f2_p()); */
      /* PRINT(f2_m());       */
    }
    

    
    __forceinline void exportConrolPoints( Vertex matrix[4][4] ) const
    {
      for (size_t y=0;y<4;y++)
	for (size_t x=0;x<4;x++)
	  matrix[y][x] = (Vertex_t)v[y][x];
      
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
      static __forceinline Vec3<T> eval_t(const GregoryTrianglePatchT &tpatch,
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


    template<class M, class T>
      static __forceinline Vec3<T> normal_t(const GregoryTrianglePatchT &tpatch,
					    const T &uu,
					    const T &vv)
    {
      FATAL("not implemented");
      const T z( zero );
      return Vec3<T>(z);
    }    
    
    template<class M, class T>
    static __forceinline Vec3<T> eval(const Vertex matrix[4][4], const T &uu, const T &vv) 
    {
      const GregoryTrianglePatchT &tpatch = *(GregoryTrianglePatchT*)matrix;
      return eval_t<M,T>(tpatch,uu,vv); 
    }

    template<class M, class T>
    static __forceinline Vec3<T> normal(const Vertex matrix[4][4], const T &uu, const T &vv) 
    {
      const GregoryTrianglePatchT &tpatch = *(GregoryTrianglePatchT*)matrix;
      return normal_t<M,T>(tpatch,uu,vv); 
    }


       
    static __forceinline Vertex normal(const Vertex matrix[4][4],
					 const float uu,
					 const float vv) 
    {
      FATAL("not yet implemented");
      return Vertex( zero );
    }
    
     static __forceinline Vertex eval(const Vertex matrix[4][4],
				      const float uu, 
				      const float vv)
     {
      FATAL("not yet implemented");
      return Vertex( zero );
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
    
    friend std::ostream &operator<<(std::ostream &o, const GregoryTrianglePatchT &g)
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

  typedef GregoryTrianglePatchT<Vec3fa,Vec3fa_t> GregoryTrianglePatch3fa;
  
}
