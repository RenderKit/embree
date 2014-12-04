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

#if 0

#include "catmullclark_patch.h"

namespace embree
{
  struct FeatureAdaptiveEval
  {
    const size_t x0,x1;
    const size_t y0,y1;
    const size_t width,height;
    Vec3fa* dst;
    
    __forceinline FeatureAdaptiveEval (const CatmullClarkPatch& patch, 
				       const size_t x0, const size_t x1, const size_t y0, const size_t y1, 
				       const size_t width, const size_t height, Vec3fa* dst)
      : x0(x0), x1(x1), y0(y0), y1(y1), width(width), height(height), dst(dst) 
    {
      const Vec2f lower(float(x0)/float(width) , float(x1)/float(width ));
      const Vec2f lower(float(y0)/float(height), float(y1)/float(height));
      eval(patch, BBox2f(zero,one), BBox2f(lower,upper), 0);
    }

    __forceinline Vec3fa& get(const size_t x, const size_t y)
    {
      assert(x<width && y<height);
      return dst[y*width+x];
    }

    void dice(const CatmullClarkPatch& patch, const BBox2f& srange, const BBox2f& erange)
    {
      size_t lx0 = clamp(floor(erange.lower.x*width ),x0,x1);
      size_t lx1 = clamp(ceil (erange.upper.x*width ),x0,x1); lx1 += (lx1 == x1);
      size_t ly0 = clamp(floor(erange.lower.y*height),y0,y1);
      size_t ly1 = clamp(ceil (erange.upper.y*height),y0,y1); ly1 += (ly1 == y1);
      if (lx0 >= lx1 || ly0 >= ly1) return;

      if (patch.isRegular()) 
      {
	BSplinePatch patcheval;
	patcheval.init(patch);
	for (size_t y=ly0; y<ly1; y++) {
	  for (size_t x=lx0; x<lx1; x++) { 
	    const float fx = (float(x)*rcp(width )-srange.lower.x)*rcp(srange.upper.x-srange.lower.x);
	    const float fy = (float(y)*rcp(height)-srange.lower.y)*rcp(srange.upper.y-srange.lower.y);
	    get(x,y) = patcheval.eval(fx,fy);
	  }
	}
      }
      else 
      {
	for (size_t y=ly0; y<ly1; y++) {
	  for (size_t x=lx0; x<lx1; x++) { 
	    const float sx1 = (float(x)*rcp(width )-srange.lower.x)*rcp(srange.upper.x-srange.lower.x), sx0 = 1.0f-sx1;
	    const float sy1 = (float(y)*rcp(height)-srange.lower.y)*rcp(srange.upper.y-srange.lower.y), sy0 = 1.0f-sy1;
	    get(x,y) = sy0*(sx0*patch.ring[0].vtx+sx1*patch.ring[1].vtx) + sy1*(sx0*patch.ring[3].vtx+sx1*patch.ring[2].vtx);
	  }
	}
      }
    }

    void eval(const CatmullClarkPatch& patch, const BBox2f& srange, const BBox2f& erange, const size_t depth)
    {
      if (erange.empty())
	return;
      
      if (patch.isRegularOrFinal(depth))
	return dice(patch,erange);

      CatmullClarkPatch patches[4]; 
      patch.subdivide(patches);

      const Vec2f c = range.center();
      const BBox2f srange0(srange.lower,c);
      const BBox2f srange1(Vec2f(c.x,srange.lower.y),Vec2f(srange.upper.x,c.y));
      const BBox2f srange2(c,srange.upper);
      const BBox2f srange3(Vec2f(srange.lower.x,c.y),Vec2f(c.x,srange.upper.y));

      eval(patches[0],depth+1,srange0,intersect(srange0,erange));
      eval(patches[1],depth+1,srange1,intersect(srange1,erange));
      eval(patches[2],depth+1,srange2,intersect(srange2,erange));
      eval(patches[3],depth+1,srange3,intersect(srange3,erange));
    }
  };

  __forceinline void feature_adaptive_eval (const CatmullClarkPatch& patch, 
					    const size_t x0, const size_t x1, const size_t y0, const size_t y1, 
					    const size_t width, const size_t height, Vec3fa* dst)
  {
    FeatureAdaptiveEval eval(patch,x0,x1,y0,y1,width,height,dst);
  }
}
#endif
