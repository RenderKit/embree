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
  template<typename Tessellator>
  struct FeatureAdaptiveSubdivisionGregory
  {
    Tessellator& tessellator;

    __forceinline FeatureAdaptiveSubdivisionGregory (int primID, const SubdivMesh::HalfEdge* h, const BufferT<Vec3fa>& vertices, Tessellator& tessellator)
      : tessellator(tessellator)
    {

      /* fast path for regular input primitives */
      //if (likely(h->isRegularFace()))
      if (likely(h->isGregoryFace()))
      {
	CatmullClarkPatch3fa patch; 
        patch.init(h,vertices);
        int neighborSubdiv[4] = { 0,0,0,0 };
        const Vec2f uv[4] = { Vec2f(0.0f,0.0f),Vec2f(1.0f,0.0f),Vec2f(1.0f,1.0f),Vec2f(0.0f,1.0f) };
        tessellator(patch,uv,neighborSubdiv);        
        return;
      }      
      /* slow path for everything else */
      int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE];
      GeneralCatmullClarkPatch3fa patch;
      patch.init(h,vertices);
      assert( patch.size() <= GeneralCatmullClarkPatch3fa::SIZE);

      for (size_t i=0; i<patch.size(); i++) {
	neighborSubdiv[i] = h->hasOpposite() ? !h->opposite()->isGregoryFace() : 0; h = h->next();
      }

      subdivide(patch,0,neighborSubdiv);

    }


    void subdivide(const GeneralCatmullClarkPatch3fa& patch, int depth, int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE])
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch()  /* && patch.isRegular() */ )) 
      {
        const Vec2f uv[4] = { Vec2f(0.0f,0.0f),Vec2f(1.0f,0.0f),Vec2f(1.0f,1.0f),Vec2f(0.0f,1.0f) };
	CatmullClarkPatch3fa qpatch; patch.init(qpatch);
	subdivide(qpatch,depth,uv,neighborSubdiv);
	return;
      }

      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch3fa,GeneralCatmullClarkPatch3fa::SIZE> patches; 
      patch.subdivide(patches,N);

      /* check if subpatches need further subdivision */
      bool childSubdiv[GeneralCatmullClarkPatch3fa::SIZE];
      for (size_t i=0; i<N; i++) {
        assert( patches[i].checkPositions() );
        childSubdiv[i] = !patches[i].isGregoryOrFinal(depth);
      }

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
	const int neighborSubdiv0[4] = { false,childSubdiv[1],childSubdiv[2],false };
	const int neighborSubdiv1[4] = { false,childSubdiv[2],childSubdiv[0],false };
	const int neighborSubdiv2[4] = { false,childSubdiv[0],childSubdiv[1],false };
	subdivide(patches[0],depth+1, uv0, neighborSubdiv0);
	subdivide(patches[1],depth+1, uv1, neighborSubdiv1);
	subdivide(patches[2],depth+1, uv2, neighborSubdiv2);
      } 

      /* parametrization for quads */
      else if (N == 4) { // FIXME: can this get reached?
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
	const int neighborSubdiv0[4] = { false,childSubdiv[1],childSubdiv[3],false };
	const int neighborSubdiv1[4] = { false,childSubdiv[2],childSubdiv[0],false };
	const int neighborSubdiv2[4] = { false,childSubdiv[3],childSubdiv[1],false };
	const int neighborSubdiv3[4] = { false,childSubdiv[0],childSubdiv[2],false };
	subdivide(patches[0],depth+1, uv0, neighborSubdiv0);
	subdivide(patches[1],depth+1, uv1, neighborSubdiv1);
	subdivide(patches[2],depth+1, uv2, neighborSubdiv2);
	subdivide(patches[3],depth+1, uv3, neighborSubdiv3);
      } 

      /* parametrization for arbitrary polygons */
      else 
      {
	for (size_t i=0; i<N; i++) 
	{
	  const Vec2f uv[4] = { Vec2f(float(i)+0.0f,0.0f),Vec2f(float(i)+1.0f,0.0f),Vec2f(float(i)+1.0f,1.0f),Vec2f(float(i)+0.0f,1.0f) };
	  const int neighborSubdiv[4] = { false,childSubdiv[(i+1)%N],childSubdiv[(i-1)%N],false };
	  subdivide(patches[i],depth+1,uv,neighborSubdiv);
	}
      }
    }

    void subdivide(const CatmullClarkPatch3fa& patch, int depth, const Vec2f uv[4], const int neighborSubdiv[4])
    {
      if (depth <= 1)
	if (patch.isGregoryOrFinal(depth))
	  return tessellator(patch,uv,neighborSubdiv);
       
      array_t<CatmullClarkPatch3fa,4> patches; 
      patch.subdivide(patches);

      const bool childSubdiv0 = !patches[0].isGregoryOrFinal(depth);
      const bool childSubdiv1 = !patches[1].isGregoryOrFinal(depth);
      const bool childSubdiv2 = !patches[2].isGregoryOrFinal(depth);
      const bool childSubdiv3 = !patches[3].isGregoryOrFinal(depth);

      const Vec2f uv01 = 0.5f*(uv[0]+uv[1]);
      const Vec2f uv12 = 0.5f*(uv[1]+uv[2]);
      const Vec2f uv23 = 0.5f*(uv[2]+uv[3]);
      const Vec2f uv30 = 0.5f*(uv[3]+uv[0]);
      const Vec2f uvcc = 0.25f*(uv[0]+uv[1]+uv[2]+uv[3]);

      const Vec2f uv0[4] = { uv[0],uv01,uvcc,uv30 };
      const Vec2f uv1[4] = { uv01,uv[1],uv12,uvcc };
      const Vec2f uv2[4] = { uvcc,uv12,uv[2],uv23 };
      const Vec2f uv3[4] = { uv30,uvcc,uv23,uv[3] };

      const int neighborSubdivf[4] = { false,false,false,false };
      const int neighborSubdiv0[4] = { false,childSubdiv1,childSubdiv3,false };
      const int neighborSubdiv1[4] = { false,false,childSubdiv2,childSubdiv0 };
      const int neighborSubdiv2[4] = { childSubdiv1,false,false,childSubdiv3 };
      const int neighborSubdiv3[4] = { childSubdiv0,childSubdiv2,false,false };
      
      if (childSubdiv0) subdivide  (patches[0],depth+1, uv0, neighborSubdivf);
      else              tessellator(patches[0],         uv0, neighborSubdiv0);

      if (childSubdiv1) subdivide  (patches[1],depth+1, uv1, neighborSubdivf);
      else              tessellator(patches[1],         uv1, neighborSubdiv1);
      
      if (childSubdiv2) subdivide  (patches[2],depth+1, uv2, neighborSubdivf);
      else              tessellator(patches[2],         uv2, neighborSubdiv2);
      
      if (childSubdiv3) subdivide  (patches[3],depth+1, uv3, neighborSubdivf);
      else              tessellator(patches[3],         uv3, neighborSubdiv3);
    }
  };

   template<typename Tessellator>
     inline void feature_adaptive_subdivision_gregory (int primID, const SubdivMesh::HalfEdge* h, const BufferT<Vec3fa>& vertices, Tessellator tessellator)
   {
     FeatureAdaptiveSubdivisionGregory<Tessellator>(primID,h,vertices,tessellator);
   }

   inline bool needsAdaptiveSubdivision(int primID, const SubdivMesh::HalfEdge* h_start, const BufferT<Vec3fa>& vertices)
   {
     int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE];
     int subdiv = 0;
     const SubdivMesh::HalfEdge* h = h_start;
     size_t valence = 0;
     do {
       neighborSubdiv[valence] = h->hasOpposite() ? !h->opposite()->isGregoryFace() : 0; h = h->next();
       subdiv += neighborSubdiv[valence];
       valence++;

     } while( h != h_start );
     return (subdiv != 0 || valence > 4);
   }
}
