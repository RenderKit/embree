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
#include "bezier_curve.h"


namespace embree
{
  __forceinline int feature_adaptive_gregory_neighbor_subdiv(const HalfEdge& h) {
   return h.hasOpposite() ? !h.opposite()->isGregoryFace() : 0;  
  }

  template<typename Tessellator>
  struct FeatureAdaptiveSubdivisionGregory
  {
    Tessellator& tessellator;

    __forceinline FeatureAdaptiveSubdivisionGregory (int primID, const HalfEdge* h, const BufferT<Vec3fa>& vertices, Tessellator& tessellator)
      : tessellator(tessellator)
    {
      /* fast path for regular input primitives */
      if (likely(h->isGregoryFace()))
      {
	CatmullClarkPatch3fa patch; 
        patch.init(h,vertices);

        int neighborSubdiv[4] = {
          feature_adaptive_gregory_neighbor_subdiv(h[0]),
          feature_adaptive_gregory_neighbor_subdiv(h[1]),
          feature_adaptive_gregory_neighbor_subdiv(h[2]),
          feature_adaptive_gregory_neighbor_subdiv(h[3])
        };

        const Vec2f uv[4] = { Vec2f(0.0f,0.0f),Vec2f(1.0f,0.0f),Vec2f(1.0f,1.0f),Vec2f(0.0f,1.0f) };
        tessellator(patch,0,uv,neighborSubdiv, nullptr, 0);        
        return;
      }      

      /* slow path for everything else */
      int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE];
      GeneralCatmullClarkPatch3fa patch;
      patch.init(h,vertices);
      assert( patch.size() <= GeneralCatmullClarkPatch3fa::SIZE);

      for (size_t i=0; i<patch.size(); i++)
        neighborSubdiv[i] = feature_adaptive_gregory_neighbor_subdiv(h[i]);
      
      subdivide(patch,0,neighborSubdiv);
    }


    void subdivide(const GeneralCatmullClarkPatch3fa& patch, int depth, int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE])
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch() )) 
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

        if ( patches[0].isGregoryOrFinal(depth+1) &&
             patches[1].isGregoryOrFinal(depth+1) &&
             patches[2].isGregoryOrFinal(depth+1))
        {
          const Vec3fa t0_p = patch.ring[0].getLimitTangent();
          const Vec3fa t0_m = patch.ring[0].getSecondLimitTangent();

          const Vec3fa t1_p = patch.ring[1].getLimitTangent();
          const Vec3fa t1_m = patch.ring[1].getSecondLimitTangent();

          const Vec3fa t2_p = patch.ring[2].getLimitTangent();
          const Vec3fa t2_m = patch.ring[2].getSecondLimitTangent();

          const Vec3fa b00 = patch.ring[0].getLimitVertex();
          const Vec3fa b03 = patch.ring[1].getLimitVertex();
          const Vec3fa b33 = patch.ring[2].getLimitVertex();

          const Vec3fa b01 = b00 + 1.0/3.0f * t0_p;
          const Vec3fa b11 = b00 + 1.0/3.0f * t0_m;

          const Vec3fa b13 = b03 + 1.0/3.0f * t1_p;
          const Vec3fa b02 = b03 + 1.0/3.0f * t1_m;

          const Vec3fa b22 = b33 + 1.0/3.0f * t2_p;
          const Vec3fa b23 = b33 + 1.0/3.0f * t2_m;

          {
            int flags = BORDER_BEZIER_CURVE_IGNORE;
            if (neighborSubdiv[0] == 0) flags |= BORDER_BEZIER_CURVE_FIRST;
            if (neighborSubdiv[2] == 0) flags |= BORDER_BEZIER_CURVE_SECOND;

            BezierCurve3fa border_curves[2];
            border_curves[0] = BezierCurve3fa(b00,b01,b02,b03);
            border_curves[1] = BezierCurve3fa(b33,b22,b11,b00);
            tessellator(patches[0],depth+1,uv0,neighborSubdiv0, border_curves, flags);
          }

          {
            int flags = BORDER_BEZIER_CURVE_IGNORE;
            if (neighborSubdiv[1] == 0) flags |= BORDER_BEZIER_CURVE_FIRST;
            if (neighborSubdiv[0] == 0) flags |= BORDER_BEZIER_CURVE_SECOND;

            BezierCurve3fa border_curves[2];
            border_curves[0] = BezierCurve3fa(b03,b13,b23,b33);
            border_curves[1] = BezierCurve3fa(b00,b01,b02,b03);
            tessellator(patches[1],depth+1,uv1,neighborSubdiv1, border_curves, flags);
          }
          
          {
            int flags = BORDER_BEZIER_CURVE_IGNORE;
            if (neighborSubdiv[2] == 0) flags |= BORDER_BEZIER_CURVE_FIRST;
            if (neighborSubdiv[1] == 0) flags |= BORDER_BEZIER_CURVE_SECOND;

            BezierCurve3fa border_curves[2];
            border_curves[0] = BezierCurve3fa(b33,b22,b11,b00);
            border_curves[1] = BezierCurve3fa(b03,b13,b23,b33);            
            tessellator(patches[2],depth+1,uv2,neighborSubdiv2, border_curves, flags);
          }
        }
        else
        {
          subdivide(patches[0],depth+1, uv0, neighborSubdiv0);
          subdivide(patches[1],depth+1, uv1, neighborSubdiv1);
          subdivide(patches[2],depth+1, uv2, neighborSubdiv2);
        }
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

        if ( patches[0].isGregoryOrFinal(depth+1) &&
             patches[1].isGregoryOrFinal(depth+1) &&
             patches[2].isGregoryOrFinal(depth+1) &&
             patches[3].isGregoryOrFinal(depth+1))
        {
          const Vec3fa t0_p = patch.ring[0].getLimitTangent();
          const Vec3fa t0_m = patch.ring[0].getSecondLimitTangent();

          const Vec3fa t1_p = patch.ring[1].getLimitTangent();
          const Vec3fa t1_m = patch.ring[1].getSecondLimitTangent();

          const Vec3fa t2_p = patch.ring[2].getLimitTangent();
          const Vec3fa t2_m = patch.ring[2].getSecondLimitTangent();

          const Vec3fa t3_p = patch.ring[3].getLimitTangent();
          const Vec3fa t3_m = patch.ring[3].getSecondLimitTangent();

          const Vec3fa b00 = patch.ring[0].getLimitVertex();
          const Vec3fa b03 = patch.ring[1].getLimitVertex();
          const Vec3fa b33 = patch.ring[2].getLimitVertex();
          const Vec3fa b30 = patch.ring[3].getLimitVertex();

          const Vec3fa b01 = b00 + 1.0/3.0f * t0_p;
          const Vec3fa b10 = b00 + 1.0/3.0f * t0_m;

          const Vec3fa b13 = b03 + 1.0/3.0f * t1_p;
          const Vec3fa b02 = b03 + 1.0/3.0f * t1_m;

          const Vec3fa b32 = b33 + 1.0/3.0f * t2_p;
          const Vec3fa b23 = b33 + 1.0/3.0f * t2_m;

          const Vec3fa b20 = b30 + 1.0/3.0f * t3_p;
          const Vec3fa b31 = b30 + 1.0/3.0f * t3_m;
            

          {
            int flags = BORDER_BEZIER_CURVE_IGNORE;
            if (neighborSubdiv[0] == 0) flags |= BORDER_BEZIER_CURVE_FIRST;
            if (neighborSubdiv[3] == 0) flags |= BORDER_BEZIER_CURVE_SECOND;

            BezierCurve3fa border_curves[2];
            border_curves[0] = BezierCurve3fa(b00,b01,b02,b03);
            border_curves[1] = BezierCurve3fa(b30,b20,b10,b00);
            tessellator(patches[0],depth+1,uv0,neighborSubdiv0, border_curves, flags);
          }
            

          {
            int flags = BORDER_BEZIER_CURVE_IGNORE;
            if (neighborSubdiv[1] == 0) flags |= BORDER_BEZIER_CURVE_FIRST;
            if (neighborSubdiv[0] == 0) flags |= BORDER_BEZIER_CURVE_SECOND;

            BezierCurve3fa border_curves[2];
            border_curves[0] = BezierCurve3fa(b03,b13,b23,b33);
            border_curves[1] = BezierCurve3fa(b00,b01,b02,b03);
            tessellator(patches[1],depth+1,uv1,neighborSubdiv1, border_curves, flags);
          }

          
          {
            int flags = BORDER_BEZIER_CURVE_IGNORE;
            if (neighborSubdiv[2] == 0) flags |= BORDER_BEZIER_CURVE_FIRST;
            if (neighborSubdiv[1] == 0) flags |= BORDER_BEZIER_CURVE_SECOND;
            
            BezierCurve3fa border_curves[2];
            border_curves[0] = BezierCurve3fa(b33,b32,b31,b30);
            border_curves[1] = BezierCurve3fa(b03,b13,b23,b33);            
            tessellator(patches[2],depth+1,uv2,neighborSubdiv2, border_curves, flags);
          }
            
          {
            int flags = BORDER_BEZIER_CURVE_IGNORE;
            if (neighborSubdiv[3] == 0) flags |= BORDER_BEZIER_CURVE_FIRST;
            if (neighborSubdiv[2] == 0) flags |= BORDER_BEZIER_CURVE_SECOND;

            BezierCurve3fa border_curves[2];
            border_curves[0] = BezierCurve3fa(b30,b20,b10,b00);
            border_curves[1] = BezierCurve3fa(b33,b32,b31,b30);
            tessellator(patches[3],depth+1,uv3,neighborSubdiv3, border_curves, flags);
          }

        }
        else
        {
          subdivide(patches[0],depth+1, uv0, neighborSubdiv0);
          subdivide(patches[1],depth+1, uv1, neighborSubdiv1);
          subdivide(patches[2],depth+1, uv2, neighborSubdiv2);
          subdivide(patches[3],depth+1, uv3, neighborSubdiv3);
        }
      } 

      /* parametrization for arbitrary polygons */
      else 
      {
	for (size_t i=0; i<N; i++) 
	{
          assert(i<MAX_PATCH_VALENCE);
          static_assert(MAX_PATCH_VALENCE <= 16, "MAX_PATCH_VALENCE > 16");
          const int h = (i >> 2) & 3, l = i & 3;
	  const Vec2f uv[4] = { (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.0f,0.0f)),
                                (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.5f,0.0f)),
                                (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.5f,0.5f)),
                                (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.0f,0.5f)) };
	  const int neighborSubdiv[4] = { false,childSubdiv[(i+1)%N],childSubdiv[(i-1)%N],false };
	  subdivide(patches[i],depth+1,uv,neighborSubdiv);
	}
      }
    }

    void subdivide(const CatmullClarkPatch3fa& patch, const int depth, const Vec2f uv[4], const int neighborSubdiv[4])
    {
      if (depth <= 1)
	if (patch.isGregoryOrFinal(depth))
	  return tessellator(patch,depth,uv,neighborSubdiv, nullptr, 0);
       
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
      else              tessellator(patches[0],depth+1, uv0, neighborSubdiv0, nullptr, 0);

      if (childSubdiv1) subdivide  (patches[1],depth+1, uv1, neighborSubdivf);
      else              tessellator(patches[1],depth+1, uv1, neighborSubdiv1, nullptr, 0);
      
      if (childSubdiv2) subdivide  (patches[2],depth+1, uv2, neighborSubdivf);
      else              tessellator(patches[2],depth+1, uv2, neighborSubdiv2, nullptr, 0);
      
      if (childSubdiv3) subdivide  (patches[3],depth+1, uv3, neighborSubdivf);
      else              tessellator(patches[3],depth+1, uv3, neighborSubdiv3, nullptr, 0);
    }
  };

   template<typename Tessellator>
     inline void feature_adaptive_subdivision_gregory (int primID, const HalfEdge* h, const BufferT<Vec3fa>& vertices, Tessellator tessellator)
   {
     FeatureAdaptiveSubdivisionGregory<Tessellator>(primID,h,vertices,tessellator);
   }

}
