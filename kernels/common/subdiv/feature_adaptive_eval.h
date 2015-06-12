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

namespace embree
{
  __forceinline Vec2f map_tri_to_quad(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2f& uv)
  {
    const Vec2f A = a, B = ab-a, C = ac-a, D = a-ab-ac+abc;
    const float AA = det(D,C), BB = det(D,A) + det(B,C) + det(uv,D), CC = det(B,A) + det(uv,B);
    const float vv = (-BB+sqrtf(BB*BB-4.0f*AA*CC))/(2.0f*AA);
    const float uu = (uv.x - A.x - vv*C.x)/(B.x + vv*D.x);
    return Vec2f(uu,vv);
  }
  
  __forceinline Vec2f map_quad_to_tri_dx(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2f& xy) {
    return (1.0f-xy.y)*(ab-a) + xy.y*(abc-ac);
  }
  
  __forceinline Vec2f map_quad_to_tri_dy(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2f& xy) {
    return (1.0f-xy.x)*(ac-a) + xy.x*(abc-ab);
  }
  
  __forceinline bool right_of_line(const Vec2f& A, const Vec2f& B, const Vec2f& P) {
    return det(A-P,B-P) <= 0.0f;
  }

  template<typename Vertex, typename Vertex_t = Vertex>
    struct FeatureAdaptivePointEval
  {
    typedef BSplinePatchT<Vertex,Vertex_t> BSplinePatch;
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
    typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;

    const float u,v;
    Vertex* P;
    Vertex* dPdu;
    Vertex* dPdv;
    
    __forceinline FeatureAdaptivePointEval (const GeneralCatmullClarkPatch& patch, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      : u(u), v(v), P(P), dPdu(dPdu), dPdv(dPdv)
    {
      const float vv = clamp(v,0.0f,1.0f); // FIXME: remove clamps, add assertions
      const float uu = clamp(u,0.0f,1.0f); 
      eval(patch,Vec2f(uu,vv),size_t(0));
    }

    void eval(const GeneralCatmullClarkPatch& patch, Vec2f uv, const size_t depth) // FIXME: no recursion required
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch())) 
      {
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));
        CatmullClarkPatch qpatch; patch.init(qpatch);
	eval(qpatch,uv,srange,depth); 
	return;
      }

      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
      patch.subdivide(patches,N); // FIXME: only have to generate one of the patches

      /* parametrization for triangles */
      if (N == 3) 
      {
        const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
        const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
        const bool ab_abc = right_of_line(ab,abc,uv);
        const bool ac_abc = right_of_line(ac,abc,uv);
        const bool bc_abc = right_of_line(bc,abc,uv);
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));

        const float u = uv.x, v = uv.y, w = 1.0f-u-v;
        if  (!ab_abc &&  ac_abc) { // FIXME: simplify
          const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(u,v));
          eval(patches[0],xy,srange,depth+1);
          if (dPdu && dPdv) {
            const Vertex dpdx = *dPdu, dpdy = *dPdv;
            const Vec2f duvdx = map_quad_to_tri_dx(a,ab,abc,ac,xy);
            const Vec2f duvdy = map_quad_to_tri_dy(a,ab,abc,ac,xy);
            const LinearSpace2f J = rcp(LinearSpace2f(duvdx,duvdy));
            *dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
            *dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
          }
        }
        else if ( ab_abc && !bc_abc) {
          const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(v,w));
          eval(patches[1],xy,srange,depth+1);
          if (dPdu && dPdv) {
            const Vertex dpdx = *dPdu, dpdy = *dPdv;
            const Vec2f duvdx = map_quad_to_tri_dx(b,bc,abc,ab,xy);
            const Vec2f duvdy = map_quad_to_tri_dy(b,bc,abc,ab,xy);
            const LinearSpace2f J = rcp(LinearSpace2f(duvdx,duvdy));
            *dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
            *dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
          }
        }
        else {
          const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(w,u));
          eval(patches[2],xy,srange,depth+1);
          if (dPdu && dPdv) {
            const Vertex dpdx = *dPdu, dpdy = *dPdv;
            const Vec2f duvdx = map_quad_to_tri_dx(c,ac,abc,bc,xy);
            const Vec2f duvdy = map_quad_to_tri_dy(c,ac,abc,bc,xy);
            const LinearSpace2f J = rcp(LinearSpace2f(duvdx,duvdy));
            *dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
            *dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
          }
        }
      } 

      /* parametrization for quads */
      else if (N == 4) 
      {
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));
        const Vec2f c = srange.center();
        if (uv.y < c.y) {
          if (uv.x < c.x) eval(patches[0],uv,BBox2f(srange.lower,c),depth+1);
          else            eval(patches[1],uv,BBox2f(Vec2f(c.x,srange.lower.y),Vec2f(srange.upper.x,c.y)),depth+1);
        } else {
          if (uv.x > c.x) eval(patches[2],uv,BBox2f(c,srange.upper),depth+1);
          else            eval(patches[3],uv,BBox2f(Vec2f(srange.lower.x,c.y),Vec2f(c.x,srange.upper.y)),depth+1);
        }
      }

      /* parametrization for arbitrary polygons */
      else 
      {
        const unsigned i = floorf(uv.x); assert(i<N);
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));
        eval(patches[i],Vec2f(floorf(uv.x),uv.y),srange,depth+1); // FIXME: uv encoding creates issues as uv=(1,0) will refer to second quad
      }
    }

    void eval(CatmullClarkPatch& patch, const Vec2f& uv, BBox2f srange, size_t depth)
    {
      /*! recursively subdivide */
      while (!patch.isRegularOrFinal2(depth)) 
      {
        array_t<CatmullClarkPatch,4> patches; 
        patch.subdivide(patches); // FIXME: only have to generate one of the patches
        
        const Vec2f c = srange.center();
        if (uv.y < c.y) {
          if (uv.x < c.x) { patch = patches[0]; srange = BBox2f(srange.lower,c); }
          else            { patch = patches[1]; srange = BBox2f(Vec2f(c.x,srange.lower.y),Vec2f(srange.upper.x,c.y)); }
        } else {
          if (uv.x > c.x) { patch = patches[2]; srange = BBox2f(c,srange.upper); }
          else            { patch = patches[3]; srange = BBox2f(Vec2f(srange.lower.x,c.y),Vec2f(c.x,srange.upper.y)); }
        }
        depth++;
      }

      const float scaleU = rcp(srange.upper.x-srange.lower.x);
      const float scaleV = rcp(srange.upper.y-srange.lower.y);

      /*! use either B-spline or bilinear patch to interpolate */
      if (patch.isRegular()) 
      {
        BSplinePatch bspline; bspline.init(patch);
        const float fx = (uv.x-srange.lower.x)*scaleU;
        const float fy = (uv.y-srange.lower.y)*scaleV;
        if (P   ) *P    = bspline.eval(fx,fy);
        if (dPdu) *dPdu = bspline.tangentU(fx,fy)*scaleU; 
        if (dPdv) *dPdv = bspline.tangentV(fx,fy)*scaleV; 
      }
      else 
      {
        const float sx1 = (uv.x-srange.lower.x)*scaleU, sx0 = 1.0f-sx1;
        const float sy1 = (uv.y-srange.lower.y)*scaleV, sy0 = 1.0f-sy1;
        const Vertex P0 = patch.ring[0].getLimitVertex();
        const Vertex P1 = patch.ring[1].getLimitVertex();
        const Vertex P2 = patch.ring[2].getLimitVertex();
        const Vertex P3 = patch.ring[3].getLimitVertex();
        if (P   ) *P    = sy0*(sx0*P0+sx1*P1) + sy1*(sx0*P3+sx1*P2);
        if (dPdu) *dPdu = (sy0*(P1-P0) + sy1*(P2-P3))*scaleU; 
        if (dPdv) *dPdv = (sx0*(P3-P0) + sx1*(P2-P1))*scaleV;
      }
    }
  };

  template<typename Ty, typename Loader>
    __forceinline void feature_adaptive_point_eval (const SubdivMesh::HalfEdge* edge, const Loader& loader, const float u, const float v, Ty* P, Ty* dPdu, Ty* dPdv)  // FIXME: remove?
  {
    switch (edge->type) 
    {
    case SubdivMesh::REGULAR_QUAD_PATCH: 
    {
      BSplinePatchT<Ty> bspline;
      bspline.init(edge,loader);
      if (P   ) *P    = bspline.eval(u,v);
      if (dPdu) *dPdu = bspline.tangentU(u,v);
      if (dPdv) *dPdv = bspline.tangentV(u,v);
      break;
    }
    case SubdivMesh::IRREGULAR_QUAD_PATCH:
    {
      CatmullClarkPatchT<Ty> patch;
      patch.init2(edge,loader);
      GregoryPatchT<Ty> gregory;
      gregory.init(patch);
      if (P   ) *P    = gregory.eval(u,v);
      if (dPdu) *dPdu = gregory.tangentU(u,v);
      if (dPdv) *dPdv = gregory.tangentV(u,v);
      break;
    }
    default: 
    {
      GeneralCatmullClarkPatchT<Ty> patch;
      patch.init2(edge,loader);
      FeatureAdaptivePointEval<Ty> eval(patch,u,v,P,dPdu,dPdv); 
      break;
    }
    }
  }

  struct FeatureAdaptiveEval
  {
    const size_t x0,x1;
    const size_t y0,y1;
    const size_t swidth,sheight;
    Vec3fa* const P;
    Vec3fa* const Ng;
    const size_t dwidth,dheight;
    
    __forceinline FeatureAdaptiveEval (const CatmullClarkPatch3fa& patch, 
				       const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
				       Vec3fa* P, Vec3fa* Ng, const size_t dwidth, const size_t dheight)
      : x0(x0), x1(x1), y0(y0), y1(y1), swidth(swidth), sheight(sheight), P(P), Ng(Ng), dwidth(dwidth), dheight(dheight)
    {
      assert(swidth < (2<<20) && sheight < (2<<20));
      const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(swidth-1,sheight-1));
      const BBox2f erange(Vec2f(x0,y0),Vec2f(x1,y1));
      eval(patch, srange, erange, 0);
    }

    void dice(const CatmullClarkPatch3fa& patch, const BBox2f& srange, const BBox2f& erange)
    {
      float lx0 = ceilf (erange.lower.x);
      float lx1 = erange.upper.x + (erange.upper.x >= x1);
      float ly0 = ceilf (erange.lower.y);
      float ly1 = erange.upper.y + (erange.upper.y >= y1);
      if (lx0 >= lx1 || ly0 >= ly1) return;

      const float scale_x = rcp(srange.upper.x-srange.lower.x);
      const float scale_y = rcp(srange.upper.y-srange.lower.y);

      if (patch.isRegular())
      //if (true)
      {
	BSplinePatch3fa patcheval; patcheval.init(patch);
	//GregoryPatch patcheval; patcheval.init(patch);
	for (float y=ly0; y<ly1; y++) 
	{
	  for (float x=lx0; x<lx1; x++) 
	  { 
	    assert(x<swidth && y<sheight);
	    const float fx = x == srange.upper.x ? 1.0f : (float(x)-srange.lower.x)*scale_x;
	    const float fy = y == srange.upper.y ? 1.0f : (float(y)-srange.lower.y)*scale_y;

	    const size_t ix = (size_t) x, iy = (size_t) y;
	    assert(ix-x0 < dwidth && iy-y0 < dheight);

	    P [(iy-y0)*dwidth+(ix-x0)] = patcheval.eval  (fx,fy);
	    Ng[(iy-y0)*dwidth+(ix-x0)] = normalize_safe(patcheval.normal(fx,fy));
	  }
	}
      }
      else 
      {
	for (float y=ly0; y<ly1; y++) 
	{
	  for (float x=lx0; x<lx1; x++) 
	  { 
	    assert(x<swidth && y<sheight);
	    
	    const float sx1 = x == srange.upper.x ? 1.0f : (float(x)-srange.lower.x)*scale_x, sx0 = 1.0f-sx1;
	    const float sy1 = y == srange.upper.y ? 1.0f : (float(y)-srange.lower.y)*scale_y, sy0 = 1.0f-sy1;
	    const size_t ix = (size_t) x, iy = (size_t) y;
	    assert(ix-x0 < dwidth && iy-y0 < dheight);

	    const Vec3fa P0 = patch.ring[0].getLimitVertex();
	    const Vec3fa P1 = patch.ring[1].getLimitVertex();
	    const Vec3fa P2 = patch.ring[2].getLimitVertex();
	    const Vec3fa P3 = patch.ring[3].getLimitVertex();
	    P [(iy-y0)*dwidth+(ix-x0)] = sy0*(sx0*P0+sx1*P1) + sy1*(sx0*P3+sx1*P2);

	    const Vec3fa Ng0 = patch.ring[0].getNormal();
	    const Vec3fa Ng1 = patch.ring[1].getNormal();
	    const Vec3fa Ng2 = patch.ring[2].getNormal();
	    const Vec3fa Ng3 = patch.ring[3].getNormal();
	    Ng[(iy-y0)*dwidth+(ix-x0)] = normalize_safe(sy0*(sx0*Ng0+sx1*Ng1) + sy1*(sx0*Ng3+sx1*Ng2));
	  }
	}
      }
    }

    void eval(const CatmullClarkPatch3fa& patch, const BBox2f& srange, const BBox2f& erange, const size_t depth)
    {
      if (erange.empty())
	return;
      
      if (patch.isRegularOrFinal2(depth))
	return dice(patch,srange,erange);

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
	  const Vec2f uv[4] = { Vec2f(float(i)+0.0f,0.0f),Vec2f(float(i)+1.0f,0.0f),Vec2f(float(i)+1.0f,1.0f),Vec2f(float(i)+0.0f,1.0f) };
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

