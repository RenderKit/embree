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
  template<typename Tessellator>
  struct FeatureAdaptiveSubdivisionGregory
  {
    Tessellator& tessellator;

    __forceinline FeatureAdaptiveSubdivisionGregory (const SubdivMesh::HalfEdge* h, const Vec3fa* vertices, Tessellator& tessellator)
      : tessellator(tessellator)
    {
#if 0
      const Vec2f uv[4] = { Vec2f(0.0f,0.0f),Vec2f(0.0f,1.0f),Vec2f(1.0f,1.0f),Vec2f(1.0f,0.0f) };
      bool subdiv[4];
      const CatmullClarkPatch patch(h,vertices);
      for (size_t i=0; i<4; i++) {
	subdiv[i] = !h->hasOpposite() || !h->opposite()->isGregoryFace(); h = h->next();
      }
      subdivide(patch,5,uv,subdiv);
#else
      bool subdiv[GeneralCatmullClarkPatch::SIZE];
      const GeneralCatmullClarkPatch patch(h,vertices);
      for (size_t i=0; i<patch.size(); i++) {
	subdiv[i] = !h->hasOpposite() || !h->opposite()->isGregoryFace(); h = h->next();
      }
      subdivide(patch,5,subdiv);
#endif
    }

    void subdivide(const GeneralCatmullClarkPatch& patch, int depth, bool subdiv[GeneralCatmullClarkPatch::SIZE])
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.size() == 4)) 
      {
	const Vec2f uv[4] = { Vec2f(0.0f,0.0f), Vec2f(0.0f,1.0f), Vec2f(1.0f,1.0f), Vec2f(1.0f,0.0f) };
	CatmullClarkPatch qpatch; patch.init(qpatch);
	subdivide(qpatch,depth,uv,subdiv);
	return;
      }

      size_t N;
      CatmullClarkPatch patches[GeneralCatmullClarkPatch::SIZE]; 
      patch.subdivide(patches,N);

      const bool noleaf = depth > 1;
      bool csubdiv[GeneralCatmullClarkPatch::SIZE];
      for (size_t i=0; i<N; i++)
        csubdiv[i] = noleaf && !patches[i].isGregory();

      /* parametrization for triangles */
      if (N == 3) {
	const Vec2f uv_0(0.0f,0.0f);
	const Vec2f uv01(0.5f,0.0f);
	const Vec2f uv_1(1.0f,0.0f);
	const Vec2f uv12(0.5f,0.5f);
	const Vec2f uv_2(0.0f,1.0f);
	const Vec2f uv20(0.0f,0.5f);
	const Vec2f uvcc(1.0f/3.0f);
	const Vec2f uv0[4] = { uv_0,uv01,uvcc,uv20 };
	const Vec2f uv1[4] = { uv_1,uv12,uvcc,uv01 };
	const Vec2f uv2[4] = { uv_2,uv20,uvcc,uv12 };
	const bool subdiv0[4] = { false,csubdiv[1],csubdiv[2],false };
	const bool subdiv1[4] = { false,csubdiv[2],csubdiv[0],false };
	const bool subdiv2[4] = { false,csubdiv[0],csubdiv[1],false };
	subdivide(patches[0],depth-1, uv0, subdiv0);
	subdivide(patches[1],depth-1, uv1, subdiv1);
	subdivide(patches[2],depth-1, uv2, subdiv2);
      } 

#if 0
      /* parametrization for quads */
      else if (N == 4) {
	const Vec2f uv_0(0.0f,0.0f);
	const Vec2f uv01(0.5f,0.0f);
	const Vec2f uv_1(1.0f,0.0f);
	const Vec2f uv12(1.0f,0.5f);
	const Vec2f uv_2(1.0f,1.0f);
	const Vec2f uv23(0.5f,1.0f);
	const Vec2f uv_3(0.0f,1.0f);
	const Vec2f uv30(0.0f,0.5f);
	const Vec2f uvcc(0.5f,0.5f);
	const Vec2f uv0[4] = { uv_0,uv01,uvcc,uv30 };
	const Vec2f uv1[4] = { uv_1,uv12,uvcc,uv01 };
	const Vec2f uv2[4] = { uv_2,uv23,uvcc,uv12 };
	const Vec2f uv3[4] = { uv_3,uv30,uvcc,uv23 };
	const bool subdiv0[4] = { false,csubdiv[1],csubdiv[3],false };
	const bool subdiv1[4] = { false,csubdiv[2],csubdiv[0],false };
	const bool subdiv2[4] = { false,csubdiv[3],csubdiv[1],false };
	const bool subdiv3[4] = { false,csubdiv[0],csubdiv[2],false };
	subdivide(patches[0],depth-1, uv0, subdiv0);
	subdivide(patches[1],depth-1, uv1, subdiv1);
	subdivide(patches[2],depth-1, uv2, subdiv2);
	subdivide(patches[3],depth-1, uv3, subdiv3);
      } 
#endif

      /* parametrization for arbitrary polygons */
      else 
      {
	for (size_t i=0; i<N; i++) 
	{
	  const Vec2f uv[4] = { Vec2f(float(i)+0.0f,0.0f),Vec2f(float(i)+0.0f,1.0f),Vec2f(float(i)+1.0f,1.0f),Vec2f(float(i)+1.0f,0.0f) };
	  const bool subdiv[4] = { false,csubdiv[(i+1)%N],csubdiv[(i-1)%N],false };
	  subdivide(patches[i],depth-1,uv,subdiv);
	  
	}
      }
    }

    void subdivide(const CatmullClarkPatch& patch, int depth, const Vec2f uv[4], const bool subdiv[4])
    {
      if (patch.isGregory() || (depth <= 0))
        return tessellator(patch,uv,subdiv);

      CatmullClarkPatch patches[4]; 
      patch.subdivide(patches);

      const bool noleaf = depth > 1;
      const bool subdivide0 = noleaf && !patches[0].isGregory();
      const bool subdivide1 = noleaf && !patches[1].isGregory();
      const bool subdivide2 = noleaf && !patches[2].isGregory();
      const bool subdivide3 = noleaf && !patches[3].isGregory();

      const Vec2f uv01 = 0.5f*(uv[0]+uv[1]);
      const Vec2f uv12 = 0.5f*(uv[1]+uv[2]);
      const Vec2f uv23 = 0.5f*(uv[2]+uv[3]);
      const Vec2f uv30 = 0.5f*(uv[3]+uv[0]);
      const Vec2f uvcc = 0.25f*(uv[0]+uv[1]+uv[2]+uv[3]);

      const Vec2f uv0[4] = { uv[0],uv01,uvcc,uv30 };
      const Vec2f uv1[4] = { uv01,uv[1],uv12,uvcc };
      const Vec2f uv2[4] = { uvcc,uv12,uv[2],uv23 };
      const Vec2f uv3[4] = { uv30,uvcc,uv23,uv[3] };

      const bool subdivf[4] = { false,false,false,false };
      const bool subdiv0[4] = { false,subdivide1,subdivide3,false };
      const bool subdiv1[4] = { false,false,subdivide2,subdivide0 };
      const bool subdiv2[4] = { subdivide1,false,false,subdivide3 };
      const bool subdiv3[4] = { subdivide0,subdivide2,false,false };
      

      if (subdivide0) subdivide  (patches[0],depth-1, uv0, subdivf);
      else            tessellator(patches[0],         uv0, subdiv0);

      if (subdivide1) subdivide  (patches[1],depth-1, uv1, subdivf);
      else            tessellator(patches[1],         uv1, subdiv1);
      
      if (subdivide2) subdivide  (patches[2],depth-1, uv2, subdivf);
      else            tessellator(patches[2],         uv2, subdiv2);
      
      if (subdivide3) subdivide  (patches[3],depth-1, uv3, subdivf);
      else            tessellator(patches[3],         uv3, subdiv3);
    }
  };

   template<typename Tessellator>
     inline void feature_adaptive_subdivision_gregory (const SubdivMesh::HalfEdge* h, const Vec3fa* vertices, Tessellator tessellator)
   {
     FeatureAdaptiveSubdivisionGregory<Tessellator>(h,vertices,tessellator);
   }
}
