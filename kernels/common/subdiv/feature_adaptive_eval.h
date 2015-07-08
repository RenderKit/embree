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
#include "bspline_patch.h"
#include "gregory_patch.h"
#include "patch.h"

namespace embree
{
  namespace isa {
  struct FeatureAdaptiveEval
  {
    const size_t x0,x1;
    const size_t y0,y1;
    const size_t swidth,sheight;
    Vec3fa* const P;
    Vec3fa* const Ng;
    const size_t dwidth,dheight;
    size_t count;

    typedef BilinearPatch3fa BilinearPatch;
    typedef BSplinePatch3fa BSplinePatch;
    typedef BezierPatch3fa BezierPatch;
    typedef GregoryPatch3fa GregoryPatch;
    
    __forceinline FeatureAdaptiveEval (const CatmullClarkPatch3fa& patch, 
				       const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
				       Vec3fa* P, Vec3fa* Ng, const size_t dwidth, const size_t dheight)
      : x0(x0), x1(x1), y0(y0), y1(y1), swidth(swidth), sheight(sheight), P(P), Ng(Ng), dwidth(dwidth), dheight(dheight), count(0)
    {
      assert(swidth < (2<<20) && sheight < (2<<20));
      const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(swidth-1,sheight-1));
      const BBox2f erange(Vec2f(x0,y0),Vec2f(x1,y1));
      eval(patch, srange, erange, 0);
      //assert(count == (x1-x0+1)*(y1-y0+1));
    }

    template<typename Patch>
    __forceinline void evalLocalGrid(const Patch& patch, const BBox2f& srange, const float lx0, const float lx1, const float ly0, const float ly1)
    {
      const float scale_x = rcp(srange.upper.x-srange.lower.x);
      const float scale_y = rcp(srange.upper.y-srange.lower.y);
      count += (lx1-lx0)*(ly1-ly0);

      for (float fy=ly0; fy<ly1; fy++) {
        for (float fx=lx0; fx<lx1; fx++) {
          assert(fx<swidth && fy<sheight);
          const float lu = select(fx == swidth -1, float(1.0f), (float(fx)-srange.lower.x)*scale_x);
          const float lv = select(fy == sheight-1, float(1.0f), (float(fy)-srange.lower.y)*scale_y);
          const size_t ix = (size_t) fx, iy = (size_t) fy;
          assert(ix-x0 < dwidth && iy-y0 < dheight);
          const int ofs = (iy-y0)*dwidth+(ix-x0);
          if (P ) P [ofs] = patch.eval(lu,lv);
          if (Ng) Ng[ofs] = normalize_safe(patch.normal(lu,lv));
        }
      }
    }

    __forceinline bool final(const CatmullClarkPatch3fa& patch, size_t depth) 
    {
#if PATCH_MIN_RESOLUTION
      return patch.isFinalResolution(PATCH_MIN_RESOLUTION) || depth>=PATCH_MAX_EVAL_DEPTH;
#else
      return depth>=PATCH_MAX_EVAL_DEPTH;
#endif
    }

    void eval(const CatmullClarkPatch3fa& patch, const BBox2f& srange, const BBox2f& erange, const size_t depth)
    {
      if (erange.empty())
	return;
      
      float lx0 = ceilf (erange.lower.x);
      float lx1 = erange.upper.x + (erange.upper.x >= x1);
      float ly0 = ceilf (erange.lower.y);
      float ly1 = erange.upper.y + (erange.upper.y >= y1);
      if (lx0 >= lx1 || ly0 >= ly1) return;
      
      if (unlikely(patch.isRegular2())) {
        RegularPatch rpatch(patch);
        evalLocalGrid(rpatch,srange,lx0,lx1,ly0,ly1);
        return;
      }
#if PATCH_USE_GREGORY == 2
      else if (unlikely(final(patch,depth) || patch.isGregory())) {
        GregoryPatch gpatch(patch);
        evalLocalGrid(gpatch,srange,lx0,lx1,ly0,ly1);
        return;
      }
#else
      else if (unlikely(final(patch,depth)))
      {
#if PATCH_USE_GREGORY == 1
        GregoryPatch gpatch(patch);
        evalLocalGrid(gpatch,srange,lx0,lx1,ly0,ly1);
#else
        BilinearPatch bpatch(patch);
        evalLocalGrid(bpatch,srange,lx0,lx1,ly0,ly1);
#endif
        return;
      }
#endif
      else
      {
        array_t<CatmullClarkPatch3fa,4> patches; 
        patch.subdivide(patches);
        
        const Vec2f c = srange.center();
        const BBox2f srange0(srange.lower,c);
        const BBox2f srange1(Vec2f(c.x,srange.lower.y),Vec2f(srange.upper.x,c.y));
        const BBox2f srange2(c,srange.upper);
        const BBox2f srange3(Vec2f(srange.lower.x,c.y),Vec2f(c.x,srange.upper.y));
        
        eval(patches[0],srange0,intersect(srange0,erange),depth+1);
        eval(patches[1],srange1,intersect(srange1,erange),depth+1);
        eval(patches[2],srange2,intersect(srange2,erange),depth+1);
        eval(patches[3],srange3,intersect(srange3,erange),depth+1);
      }
    }
  };

  __forceinline void feature_adaptive_eval (const CatmullClarkPatch3fa& patch, 
					    const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
					    Vec3fa* P, Vec3fa* Ng, const size_t dwidth, const size_t dheight)
  {
    FeatureAdaptiveEval eval(patch,x0,x1,y0,y1,swidth,sheight,P,Ng,dwidth,dheight);
  }

  template<typename Tessellator>
  struct FeatureAdaptiveEvalSubdivision
  {
    Tessellator& tessellator;

    __forceinline FeatureAdaptiveEvalSubdivision (const SubdivMesh::HalfEdge* h, const BufferT<Vec3fa>& vertices, Tessellator& tessellator)
      : tessellator(tessellator)
    {
      int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE]; // FIXME: use array_t
      GeneralCatmullClarkPatch3fa patch;
      patch.init(h,vertices);
      for (size_t i=0; i<patch.size(); i++) {
	neighborSubdiv[i] = h->hasOpposite() ? !h->opposite()->isGregoryFace() : 0; h = h->next();
      }
      subdivide(patch,0,neighborSubdiv);
    }

    void subdivide(const GeneralCatmullClarkPatch3fa& patch, int depth, int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE])
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch())) 
      {
	const Vec2f uv[4] = { Vec2f(0.0f,0.0f), Vec2f(1.0f,0.0f), Vec2f(1.0f,1.0f), Vec2f(0.0f,1.0f) };
	CatmullClarkPatch3fa qpatch; patch.init(qpatch);
	subdivide(qpatch,depth,uv,neighborSubdiv,0);
	return;
      }

      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch3fa,GeneralCatmullClarkPatch3fa::SIZE> patches; 
      patch.subdivide(patches,N);

      /* check if subpatches need further subdivision */
      array_t<bool,GeneralCatmullClarkPatch3fa::SIZE> childSubdiv;
      for (size_t i=0; i<N; i++)
        childSubdiv[i] = !patches[i].isGregoryOrFinal(depth);

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
	subdivide(patches[0],depth+1, uv0, neighborSubdiv0, 0);
	subdivide(patches[1],depth+1, uv1, neighborSubdiv1, 1);
	subdivide(patches[2],depth+1, uv2, neighborSubdiv2, 2);
      } 

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
	const int neighborSubdiv0[4] = { false,childSubdiv[1],childSubdiv[3],false };
	const int neighborSubdiv1[4] = { false,childSubdiv[2],childSubdiv[0],false };
	const int neighborSubdiv2[4] = { false,childSubdiv[3],childSubdiv[1],false };
	const int neighborSubdiv3[4] = { false,childSubdiv[0],childSubdiv[2],false };
	subdivide(patches[0],depth+1, uv0, neighborSubdiv0, 0);
	subdivide(patches[1],depth+1, uv1, neighborSubdiv1, 1);
	subdivide(patches[2],depth+1, uv2, neighborSubdiv2, 2);
	subdivide(patches[3],depth+1, uv3, neighborSubdiv3, 3);
      } 

      /* parametrization for arbitrary polygons */
      else 
      {
	for (size_t i=0; i<N; i++) 
	{
          assert(i<SubdivMesh::MAX_VALENCE);
          static_assert(SubdivMesh::MAX_VALENCE <= 16, "MAX_VALENCE > 16");
          const int h = (i >> 2) & 3, l = i & 3;
	  const Vec2f uv[4] = { (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.0f,0.0f)),
                                (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.5f,0.0f)),
                                (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.5f,0.5f)),
                                (1.0f/4.0f) * (Vec2f(l,h) + Vec2f(0.0f,0.5f)) };
	  const int neighborSubdiv[4] = { false,childSubdiv[(i+1)%N],childSubdiv[(i-1)%N],false };
	  subdivide(patches[i],depth+1,uv,neighborSubdiv, i);
	}
      }
    }
    
    void subdivide(const CatmullClarkPatch3fa& patch, int depth, const Vec2f uv[4], const int neighborSubdiv[4], const int id) {
      return tessellator(patch,uv,neighborSubdiv,id);
    }
  };

   template<typename Tessellator>
     inline void feature_adaptive_subdivision_eval (const SubdivMesh::HalfEdge* h, const BufferT<Vec3fa>& vertices, Tessellator tessellator)
   {
     FeatureAdaptiveEvalSubdivision<Tessellator>(h,vertices,tessellator);
   }
  }
}

