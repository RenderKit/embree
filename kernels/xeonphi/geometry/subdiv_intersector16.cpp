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
       __aligned(64) FinalQuad finalQuad;
       patch.init( finalQuad );
       intersect1_quad(rayIndex,dir_xyz,org_xyz,ray16,finalQuad,geomID,primID);      
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
       __aligned(64) FinalQuad finalQuad;
       patch.init( finalQuad );
       intersect1_quad(rayIndex,dir_xyz,org_xyz,ray16,finalQuad,geomID,primID);      
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
