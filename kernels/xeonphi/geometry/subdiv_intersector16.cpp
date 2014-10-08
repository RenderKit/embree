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

#include "subdivpatch1.h"
#include "common/ray16.h"
#include "geometry/filter.h"
#include "subdiv_intersector16.h"
#include "bicubic_bezier_patch.h"

namespace embree
{

  static __forceinline bool intersect1_quad(const size_t rayIndex, 
					    const mic_f &dir_xyz,
					    const mic_f &org_xyz,
					    Ray16& ray16,
					    const Vec3fa &vtx0,
					    const Vec3fa &vtx1,
					    const Vec3fa &vtx2,
					    const Vec3fa &vtx3,
					    const unsigned int geomID,
					    const unsigned int primID)
  {
    const mic_i and_mask = broadcast4to16i(_zlc4);

    const mic_f v0 = broadcast4to16f(&vtx0);
    const mic_f v1 = select(0x00ff,broadcast4to16f(&vtx1),broadcast4to16f(&vtx2));
    const mic_f v2 = select(0x00ff,broadcast4to16f(&vtx2),broadcast4to16f(&vtx3));
	      
    const mic_f e1 = v1 - v0;
    const mic_f e2 = v0 - v2;	     
    const mic_f normal = lcross_zxy(e1,e2);
    const mic_f org = v0 - org_xyz;
    const mic_f odzxy = msubr231(org * swizzle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, swizzle(org,_MM_SWIZ_REG_DACB));
    const mic_f den = ldot3_zxy(dir_xyz,normal);	      
    const mic_f rcp_den = rcp(den);
    const mic_f uu = ldot3_zxy(e2,odzxy); 
    const mic_f vv = ldot3_zxy(e1,odzxy); 
    const mic_f u = uu * rcp_den;
    const mic_f v = vv * rcp_den;

#if defined(__BACKFACE_CULLING__)
    const mic_m m_init = (mic_m)0x1111 & (den > zero);
#else
    const mic_m m_init = 0x1111;
#endif

    const mic_m valid_u = ge(m_init,u,zero);
    const mic_m valid_v = ge(valid_u,v,zero);
    const mic_m m_aperture = le(valid_v,u+v,mic_f::one()); 

    const mic_f nom = ldot3_zxy(org,normal);

    if (unlikely(none(m_aperture))) return false;
    const mic_f t = rcp_den*nom;
    mic_f max_dist_xyz = mic_f(ray16.tfar[rayIndex]);
    mic_m m_final  = lt(lt(m_aperture,mic_f(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    /* did the ray hit one of the four triangles? */
    if (unlikely(any(m_final)))
      {
	STAT3(normal.trav_prim_hits,1,1,1);
	max_dist_xyz  = select(m_final,t,max_dist_xyz);
	const mic_f min_dist = vreduce_min(max_dist_xyz);
	const mic_m m_dist = eq(min_dist,max_dist_xyz);
	const size_t vecIndex = bitscan(toInt(m_dist));
	const size_t triIndex = vecIndex >> 2;

	const mic_m m_tri = m_dist^(m_dist & (mic_m)((unsigned int)m_dist - 1));

                
	prefetch<PFHINT_L1EX>(&ray16.tfar);  
	prefetch<PFHINT_L1EX>(&ray16.u);
	prefetch<PFHINT_L1EX>(&ray16.v);
	prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
	prefetch<PFHINT_L1EX>(&ray16.geomID);
	prefetch<PFHINT_L1EX>(&ray16.primID);

	max_dist_xyz = min_dist;

	const mic_f gnormalx(normal[triIndex*4+1]);
	const mic_f gnormaly(normal[triIndex*4+2]);
	const mic_f gnormalz(normal[triIndex*4+0]);
		  
	compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
	compactustore16f_low(m_tri,&ray16.u[rayIndex],u); 
	compactustore16f_low(m_tri,&ray16.v[rayIndex],v); 
	compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
	compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
	compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 

	ray16.geomID[rayIndex] = geomID;
	ray16.primID[rayIndex] = primID;
	return true;
      
      }
    return false;      
  };


  __forceinline void subdivide(const RegularCatmullClarkPatch &source,
			       RegularCatmullClarkPatch dest[4])
  {
    const mic_f row0  = load16f((const float *)&source.v[0][0]);
    const mic_f row1  = load16f((const float *)&source.v[1][0]);
    const mic_f row2  = load16f((const float *)&source.v[2][0]);
    const mic_f row3  = load16f((const float *)&source.v[3][0]);

    const mic_m m_edge0 = 0x0ff0;
    const mic_m m_edge1 = 0x0f0f;

    const mic_m m_left  = 0x0f0f;
    const mic_m m_right = 0xf0f0;

    /* ---- face 0 1 1 2 ---- */
    const mic_f row01 = row0 + row1;
    const mic_f row01shuffle = lshuf<2,1,2,1>(row01);
    const mic_f face0112 = (row01 + row01shuffle) * 0.25f;

    /* ---- edge edge0011 ---- */
    const mic_f face1021 = lshuf<2,3,0,1>(face0112);
    const mic_f mid0011 = select(m_edge0,row01,row01shuffle);
    const mic_f edge0011 = (mid0011 + face0112 + face1021) * 0.25f;

    const mic_f c0_v00 = select(m_left,face0112,edge0011);
    const mic_f c1_v00 = select(m_right,face0112,edge0011);

    /* ---- face 3 4 4 5 ---- */
    const mic_f row12 = row1 + row2;
    const mic_f row12shuffle = lshuf<2,1,2,1>(row12);
    const mic_f face3445 = (row12 + row12shuffle) * 0.25f;

    /* ---- edge edge2334 ---- */
    const mic_f row1shuffle = lshuf<2,1,2,1>(row1);
    const mic_f mid2334 = row1shuffle + row1;
    store16f(&dest[0].v[0][0],c0_v00);
    const mic_f edge2334 = (face0112 + face3445 + mid2334) * 0.25f;
    store16f(&dest[1].v[0][0],c1_v00);

    /* ---- edge edge5566 ---- */
    const mic_f face4354 = lshuf<2,3,0,1>(face3445);
    const mic_f mid5566 = select(m_edge0,row12,row12shuffle);
    const mic_f edge5566 = (mid5566 + face3445 + face4354) * 0.25f;
   
    const mic_f c0_v20 = select(m_left,face3445,edge5566);
    const mic_f c1_v20 = select(m_right,face3445,edge5566);
    const mic_f c2_v10 = c1_v20; // select(m_right,face3445,edge5566);
    const mic_f c3_v10 = c0_v20; // select(m_left,face3445,edge5566);

    /* ---- face 6 7 7 8 ---- */
    const mic_f row23 = row2 + row3;
    const mic_f row23shuffle = lshuf<2,1,2,1>(row23);
    const mic_f face6778 = (row23 + row23shuffle) * 0.25f;

    /* ---- edge edge7889 ---- */
    const mic_f row2shuffle = lshuf<2,1,2,1>(row2);
    store16f(&dest[0].v[2][0],c0_v20);
    const mic_f mid7889 = row2shuffle + row2;
    store16f(&dest[1].v[2][0],c1_v20);
    const mic_f edge7889 = (face3445 + face6778 + mid7889) * 0.25f;
    store16f(&dest[2].v[1][0],c2_v10);
    /* ---- edge edge10101111 ---- */
    const mic_f face7687 = lshuf<2,3,0,1>(face6778);
    store16f(&dest[3].v[1][0],c3_v10);
    const mic_f mid10101111 = select(m_edge0,row23,row23shuffle);
    const mic_f edge10101111 = (mid10101111 + face6778 + face7687) * 0.25;

    const mic_f c2_v30 = select(m_right,face6778,edge10101111);
    const mic_f c3_v30 = select(m_left,face6778,edge10101111);


    /* ---- new quad0011 ---- */
    const mic_f p5566 = lshuf<2,2,1,1>(row1);
    const mic_f row1_e = row1 + lshuf<1,0,0,2>(row1);

    const mic_f sum_edges01 = select(0xf00f,row1_e,row0+row2);
    store16f(&dest[2].v[3][0],c2_v30);
    const mic_f qr01_2lanes = face0112 + face3445 + sum_edges01;
    store16f(&dest[3].v[3][0],c3_v30);    

    const mic_f qr01 = qr01_2lanes + lshuf<2,3,0,1>(qr01_2lanes);
    const mic_f quad0011 = qr01 * 1.0f/16.0f + p5566 * 0.5f;


    const mic_f c0_v10 = select(m_left,edge2334,quad0011);
    const mic_f c1_v10 = select(m_right,edge2334,quad0011);
    const mic_f c2_v00 = c1_v10; // select(m_right,edge2334,quad0011);
    const mic_f c3_v00 = c0_v10; // select(m_left,edge2334,quad0011);

    /* ---- new quad2233 ---- */
    const mic_f p991010 = lshuf<2,2,1,1>(row2);
    const mic_f row2_e = row2 + lshuf<1,0,0,2>(row2);
    const mic_f sum_edges23 = select(0xf00f,row2_e,row1+row3);
    const mic_f qr23_2lanes = face3445 + face6778 + sum_edges23;
    const mic_f qr23 = qr23_2lanes + lshuf<2,3,0,1>(qr23_2lanes);
    const mic_f quad2233 = p991010 * 0.5f + qr23 * 1.0f/16.0f;

    store16f(&dest[0].v[1][0],c0_v10);

    const mic_f c0_v30 = select(m_left,edge7889,quad2233);
    const mic_f c1_v30 = select(m_right,edge7889,quad2233);

    const mic_f c2_v20 = c1_v30; // select(m_right,edge7889,quad2233);
    const mic_f c3_v20 = c0_v30; // select(m_left,edge7889,quad2233);
    store16f(&dest[1].v[1][0],c1_v10);
    store16f(&dest[2].v[0][0],c2_v00);
    store16f(&dest[3].v[0][0],c3_v00);

    store16f(&dest[0].v[3][0],c0_v30);
    store16f(&dest[3].v[2][0],c3_v20);
    store16f(&dest[1].v[3][0],c1_v30);
    store16f(&dest[2].v[2][0],c2_v20);


  }

 void subdivide_intersect1(const size_t rayIndex, 
			   const mic_f &dir_xyz,
			   const mic_f &org_xyz,
			   Ray16& ray16,
			   const IrregularCatmullClarkPatch &patch,
			   const unsigned int geomID,
			   const unsigned int primID,
			   const unsigned int subdiv_level)
 {
   if (subdiv_level == 0)
     {
       intersect1_quad(rayIndex,
		       dir_xyz,
		       org_xyz,
		       ray16,
		       patch.ring[0].vtx,
		       patch.ring[1].vtx,
		       patch.ring[2].vtx,
		       patch.ring[3].vtx,
		       geomID,
		       primID);      

     }
   else
     {
       IrregularCatmullClarkPatch subpatches[4];
       patch.subdivide(subpatches);
       for (size_t i=0;i<4;i++)
	 subdivide_intersect1(rayIndex, 
			      dir_xyz,
			      org_xyz,
			      ray16,
			      subpatches[i],
			      geomID,
			      primID,
			      subdiv_level - 1);	    
     }
   
 }


 void subdivide_intersect1(const size_t rayIndex, 
			   const mic_f &dir_xyz,
			   const mic_f &org_xyz,
			   Ray16& ray16,
			   const RegularCatmullClarkPatch &patch,
			   const unsigned int geomID,
			   const unsigned int primID,
			   const unsigned int subdiv_level)
 {
   if (subdiv_level == 0)
     {
       //__aligned(64) FinalQuad finalQuad;
       //patch.init( finalQuad );
       //intersect1_quad(rayIndex,dir_xyz,org_xyz,ray16,finalQuad,geomID,primID);      
       intersect1_quad(rayIndex,
		       dir_xyz,
		       org_xyz,
		       ray16,
		       patch.v[1][1],
		       patch.v[1][2],
		       patch.v[2][2],
		       patch.v[2][1],
		       geomID,
		       primID);      
     }
   else
     {
       RegularCatmullClarkPatch subpatches[4];
       //patch.subdivide(subpatches);
       subdivide(patch,subpatches);
       for (size_t i=0;i<4;i++)
	 subdivide_intersect1(rayIndex, 
			      dir_xyz,
			      org_xyz,
			      ray16,
			      subpatches[i],
			      geomID,
			      primID,
			      subdiv_level - 1);	    
     }
   
 }

};
