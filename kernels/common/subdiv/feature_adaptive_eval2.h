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
#include "tessellation.h"

namespace embree
{
  struct FeatureAdaptiveEval2
  {
    const size_t x0,x1;
    const size_t y0,y1;
    const size_t swidth,sheight;
    float* const Px;
    float* const Py;
    float* const Pz;
    float* const U;
    float* const V;
    const size_t dwidth,dheight;
    
    __forceinline FeatureAdaptiveEval2 (const GeneralCatmullClarkPatch3fa& patch, size_t subPatch,
                                        const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
                                        float* Px, float* Py, float* Pz, float* U, float* V, const size_t dwidth, const size_t dheight)
      : x0(x0), x1(x1), y0(y0), y1(y1), swidth(swidth), sheight(sheight), Px(Px), Py(Py), Pz(Pz), U(U), V(V), dwidth(dwidth), dheight(dheight)
    {
      assert(swidth < (2<<20) && sheight < (2<<20));
      const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(swidth-1,sheight-1));
      const BBox2f erange(Vec2f(x0,y0),Vec2f(x1,y1));

      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch())) 
      {
        CatmullClarkPatch3fa qpatch; patch.init(qpatch);
        eval(patch, srange, erange, 0);
        return;
      }

      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch3fa,GeneralCatmullClarkPatch3fa::SIZE> patches; 
      patch.subdivide(patches,N);
      assert(subPatch < N);
      eval(patches[subPatch], srange, erange, 0);
    }

    void dice(const CatmullClarkPatch3fa& patch, const BBox2f& srange, const BBox2f& erange)
    {
      int lx0 = ceilf (erange.lower.x);
      int lx1 = floorf(erange.upper.x); if (lx1 == x1) lx1++;
      int ly0 = ceilf (erange.lower.y);
      int ly1 = floorf(erange.upper.y); if (ly1 == y1) ly1++;
      if (lx0 >= lx1 || ly0 >= ly1) return;

      const float scale_x = rcp(srange.upper.x-srange.lower.x);
      const float scale_y = rcp(srange.upper.y-srange.lower.y);

      if (patch.isGregory1())
      {
	//BSplinePatch3fa patcheval; patcheval.init(patch);
	GregoryPatch patcheval; patcheval.init(patch);

        //gridUVTessellator(patch.grid_u_res,patch.grid_v_res,grid_u,grid_v);

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
      
      //if (patch.isRegularOrFinal(depth))
      if (patch.isGregoryOrFinal(depth))
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

  __forceinline void feature_adaptive_eval (const SubdivMesh::HalfEdge* h, size_t subPatch, const BufferT<Vec3fa>& vertices,
					    const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
					    float* Px, float* Py, float* Pz, float* U, float* V, const size_t dwidth, const size_t dheight)
  {
    GeneralCatmullClarkPatch3fa patch;
    patch.init(h,vertices);
    FeatureAdaptiveEval2(patch,x0,x1,y0,y1,swidth,sheight,Px,Py,Pz,U,V,dwidth,dheight);
  }
}

