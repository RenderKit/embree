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
#if 1
  struct FeatureAdaptiveEval
  {
    const size_t x0,x1;
    const size_t y0,y1;
    const size_t swidth,sheight;
    Vec3fa* const P;
    Vec3fa* const Ng;
    const size_t dwidth,dheight;
    
    __forceinline FeatureAdaptiveEval (const CatmullClarkPatch& patch, 
				       const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
				       Vec3fa* P, Vec3fa* Ng, const size_t dwidth, const size_t dheight)
      : x0(x0), x1(x1), y0(y0), y1(y1), swidth(swidth), sheight(sheight), P(P), Ng(Ng), dwidth(dwidth), dheight(dheight)
    {
      //PRINT2(x0,x1);
      //PRINT2(y0,y1);
      //PRINT2(swidth,sheight);
      //PRINT2(dwidth,dheight);
      //PING;
      
      //for (size_t y=0; y<=y1-y0; y++)
      //for (size_t x=0; x<=x1-x0; x++)
      //  P[y*dwidth+x] = Vec3fa(100.0f);

      assert(swidth < (2<<20) && sheight < (2<<20));
      const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(swidth-1,sheight-1));
      const BBox2f erange(Vec2f(x0,y0),Vec2f(x1,y1));
      eval(patch, srange, erange, 0);
    }

    void dice(const CatmullClarkPatch& patch, const BBox2f& srange, const BBox2f& erange)
    {
      //PRINT("dice");
      //PRINT(srange);
      //PRINT(erange);
      float lx0 = ceil (erange.lower.x);
      float lx1 = erange.upper.x + (erange.upper.x >= x1-0.001f); //floor(erange.upper.x);
      float ly0 = ceil (erange.lower.y);
      float ly1 = erange.upper.y + (erange.upper.y >= y1-0.001f); //floor(erange.upper.y);
      //PRINT2(lx0,lx1);
      //PRINT2(ly0,ly1);
      if (lx0 >= lx1 || ly0 >= ly1) return;
      //PRINT("passed");

      const float scale_x = rcp(srange.upper.x-srange.lower.x);
      const float scale_y = rcp(srange.upper.y-srange.lower.y);
      //PRINT(scale);

      if (patch.isRegular()) 
      {
	BSplinePatch patcheval;
	patcheval.init(patch);
	for (float y=ly0; y<ly1; y++) 
	{
	  for (float x=lx0; x<lx1; x++) 
	  { 
	    assert(x<swidth && y<sheight);
	    const float fx = (float(x)-srange.lower.x)*scale_x;
	    const float fy = (float(y)-srange.lower.y)*scale_y;
	    //if (fx < -0.001f || fx > 1.001f || fy < -0.001f || fy > 1.001f) continue;
	    const size_t ix = (size_t) x, iy = (size_t) y;
	    assert(ix-x0 < dwidth && iy-y0 < dheight);
	    
	    P [(iy-y0)*dwidth+(ix-x0)] = patcheval.eval  (fx,fy);
	    Ng[(iy-y0)*dwidth+(ix-x0)] = normalize_safe(patcheval.normal(fx,fy));

	    //PRINT(scale);
	    //PRINT2(x,y);
	    //PRINT2(fx,fy);
	    //PRINT4("bspline",ix-x0,iy-y0,P [(iy-y0)*dwidth+(ix-x0)]);
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
	    
	    const float sx1 = (float(x)-srange.lower.x)*scale_x, sx0 = 1.0f-sx1;
	    const float sy1 = (float(y)-srange.lower.y)*scale_y, sy0 = 1.0f-sy1;
	    //if (sx1 < -0.001f || sx1 > 1.001f || sy1 < -0.001f || sy1 > 1.001f) continue;
	    const size_t ix = (size_t) x, iy = (size_t) y;
	    //PRINT2(ix,iy);
	    //PRINT2(dwidth,dheight);
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

	    //PRINT4("quad",ix-x0,iy-y0,P [(iy-y0)*dwidth+(ix-x0)]);
	  }
	}
      }
    }

    void eval(const CatmullClarkPatch& patch, const BBox2f& srange, const BBox2f& erange, const size_t depth)
    {
      //PRINT(depth);
      //PRINT(srange);
      //PRINT(erange);

      if (erange.empty())
	return;
      
      if (patch.isRegularOrFinal2(depth))
	return dice(patch,srange,erange);

      CatmullClarkPatch patches[4]; 
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

  __forceinline void feature_adaptive_eval (const CatmullClarkPatch& patch, 
					    const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
					    Vec3fa* P, Vec3fa* Ng, const size_t dwidth, const size_t dheight)
  {
    FeatureAdaptiveEval eval(patch,
			     x0,x1,y0,y1,swidth,sheight,
			     P,Ng,dwidth,dheight);
  }

#else

  struct FeatureAdaptiveEval
  {
    const size_t x0,x1;
    const size_t y0,y1;
    const size_t swidth,sheight;
    Vec3fa* const P;
    Vec3fa* const Ng;
    const size_t dwidth,dheight;
    
    __forceinline FeatureAdaptiveEval (const CatmullClarkPatch& patch, 
				       const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
				       Vec3fa* P, Vec3fa* Ng, const size_t dwidth, const size_t dheight)
      : x0(x0), x1(x1), y0(y0), y1(y1), swidth(swidth), sheight(sheight), P(P), Ng(Ng), dwidth(dwidth), dheight(dheight)
    {
      const Vec2f lower(float(x0)/float(swidth -1), float(y0)/float(sheight-1));
      const Vec2f upper(float(x1)/float(swidth -1), float(y1)/float(sheight-1));
      eval(patch, BBox2f(zero,one), BBox2f(lower,upper), 0);
    }

    void dice(const CatmullClarkPatch& patch, const BBox2f& srange, const BBox2f& erange)
    {
      size_t lx0 = clamp((size_t)floor(erange.lower.x*(swidth -1)),x0,x1);
      size_t lx1 = clamp((size_t)ceil (erange.upper.x*(swidth -1)),x0,x1); lx1 += (lx1 == x1);
      size_t ly0 = clamp((size_t)floor(erange.lower.y*(sheight-1)),y0,y1);
      size_t ly1 = clamp((size_t)ceil (erange.upper.y*(sheight-1)),y0,y1); ly1 += (ly1 == y1);
      if (lx0 >= lx1 || ly0 >= ly1) return;

      if (patch.isRegular()) 
      {
	BSplinePatch patcheval;
	patcheval.init(patch);
	for (size_t y=ly0; y<ly1; y++) 
	{
	  for (size_t x=lx0; x<lx1; x++) 
	  { 
	    assert(x<swidth && y<sheight);
	    const float fx = (float(x)*rcp((float)(swidth -1))-srange.lower.x)*rcp(srange.upper.x-srange.lower.x);
	    const float fy = (float(y)*rcp((float)(sheight-1))-srange.lower.y)*rcp(srange.upper.y-srange.lower.y);
	    if (fx < -0.001f || fx > 1.001f || fy < -0.001f || fy > 1.001f) continue;
	    P [(y-y0)*dwidth+(x-x0)] = patcheval.eval  (fx,fy);
	    Ng[(y-y0)*dwidth+(x-x0)] = normalize_safe(patcheval.normal(fx,fy));
	  }
	}
      }
      else 
      {
	for (size_t y=ly0; y<ly1; y++) 
	{
	  for (size_t x=lx0; x<lx1; x++) 
	  { 
	    assert(x<swidth && y<sheight);
	    
	    const float sx1 = (float(x)*rcp((float)(swidth -1))-srange.lower.x)*rcp(srange.upper.x-srange.lower.x), sx0 = 1.0f-sx1;
	    const float sy1 = (float(y)*rcp((float)(sheight-1))-srange.lower.y)*rcp(srange.upper.y-srange.lower.y), sy0 = 1.0f-sy1;
	    if (sx1 < -0.001f || sx1 > 1.001f || sy1 < -0.001f || sy1 > 1.001f) continue;
	    const Vec3fa P0 = patch.ring[0].getLimitVertex();
	    const Vec3fa P1 = patch.ring[1].getLimitVertex();
	    const Vec3fa P2 = patch.ring[2].getLimitVertex();
	    const Vec3fa P3 = patch.ring[3].getLimitVertex();
	    P [(y-y0)*dwidth+(x-x0)] = sy0*(sx0*P0+sx1*P1) + sy1*(sx0*P3+sx1*P2);

	    const Vec3fa Ng0 = patch.ring[0].getNormal();
	    const Vec3fa Ng1 = patch.ring[1].getNormal();
	    const Vec3fa Ng2 = patch.ring[2].getNormal();
	    const Vec3fa Ng3 = patch.ring[3].getNormal();
	    Ng[(y-y0)*dwidth+(x-x0)] = sy0*(sx0*Ng0+sx1*Ng1) + sy1*(sx0*Ng3+sx1*Ng2);
	  }
	}
      }
    }

    void eval(const CatmullClarkPatch& patch, const BBox2f& srange, const BBox2f& erange, const size_t depth)
    {
      if (erange.empty())
	return;
      
      if (patch.isRegularOrFinal2(depth))
	return dice(patch,srange,erange);

      CatmullClarkPatch patches[4]; 
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

  __forceinline void feature_adaptive_eval (const CatmullClarkPatch& patch, 
					    const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
					    Vec3fa* P, Vec3fa* Ng, const size_t dwidth, const size_t dheight)
  {
    FeatureAdaptiveEval eval(patch,
			     x0,x1,y0,y1,swidth,sheight,
			     P,Ng,dwidth,dheight);
  }
#endif
}

