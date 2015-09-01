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

#include "patch.h"
#include "catmullclark_patch.h"
#include "bspline_patch.h"
#include "gregory_patch.h"
#include "tessellation.h"

namespace embree
{
  namespace isa 
  {
    struct FeatureAdaptiveEvalGrid
    {
      typedef CatmullClark1Ring3fa CatmullClarkRing;
      typedef CatmullClarkPatch3fa CatmullClarkPatch;
      typedef BilinearPatch3fa BilinearPatch;
      typedef BSplinePatch3fa BSplinePatch;
      typedef BezierPatch3fa BezierPatch;
      typedef GregoryPatch3fa GregoryPatch;

    private:
      const size_t x0,x1;
      const size_t y0,y1;
      const size_t swidth,sheight;
      const float rcp_swidth, rcp_sheight;
      float* const Px;
      float* const Py;
      float* const Pz;
      float* const U;
      float* const V;
      float* const Nx;
      float* const Ny;
      float* const Nz;
      const size_t dwidth,dheight;
      size_t count;
      

    public:      
      FeatureAdaptiveEvalGrid (const GeneralCatmullClarkPatch3fa& patch, size_t subPatch,
                               const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
                               float* Px, float* Py, float* Pz, float* U, float* V, 
                               float* Nx, float* Ny, float* Nz,
                               const size_t dwidth, const size_t dheight);
      
      FeatureAdaptiveEvalGrid (const CatmullClarkPatch3fa& patch,
                               const BBox2f& srange, const BBox2f& erange, const size_t depth,
                               const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
                               float* Px, float* Py, float* Pz, float* U, float* V, 
                               float* Nx, float* Ny, float* Nz,
                               const size_t dwidth, const size_t dheight);

      template<typename Patch>
      void evalLocalGrid(const Patch& patch, const BBox2f& srange, const int lx0, const int lx1, const int ly0, const int ly1);
      
      bool final(const CatmullClarkPatch3fa& patch, const CatmullClarkRing::Type type, size_t depth);
      
      void eval(const CatmullClarkPatch3fa& patch, const BBox2f& srange, const BBox2f& erange, const size_t depth, 
                const BezierCurve3fa* border0 = nullptr, const BezierCurve3fa* border1 = nullptr, const BezierCurve3fa* border2 = nullptr, const BezierCurve3fa* border3 = nullptr);
    };
    
    template<typename Eval, typename Patch>
      bool stitch_col(const Patch& patch, int subPatch,
                      const bool right, const size_t y0, const size_t y1, const int fine_y, const int coarse_y, 
                      float* Px, float* Py, float* Pz, float* U, float* V, float* Nx, float* Ny, float* Nz, const size_t dx0, const size_t dwidth, const size_t dheight)
    {
      assert(coarse_y <= fine_y);
      if (likely(fine_y == coarse_y))
        return false;
      
      const size_t y0s = stitch(y0,fine_y,coarse_y);
      const size_t y1s = stitch(y1,fine_y,coarse_y);
      assert(y1s-y0s < 4097);
      
      float px[4097], py[4097], pz[4097], u[4097], v[4097], nx[4097], ny[4097], nz[4097]; // FIXME: limits maximal level
      Eval(patch,subPatch, right,right, y0s,y1s, 2,coarse_y+1, px,py,pz,u,v, Nx?nx:nullptr,Ny?ny:nullptr,Nz?nz:nullptr, 1,4097);
      
      for (int y=y0; y<=y1; y++) {
        const size_t ys = stitch(y,fine_y,coarse_y)-y0s;
        Px[(y-y0)*dwidth+dx0] = px[ys];
        Py[(y-y0)*dwidth+dx0] = py[ys];
        Pz[(y-y0)*dwidth+dx0] = pz[ys];
        U [(y-y0)*dwidth+dx0] = u[ys];
        V [(y-y0)*dwidth+dx0] = v[ys];
        if (unlikely(Nx != nullptr)) {
          Nx[(y-y0)*dwidth+dx0] = nx[ys];
          Ny[(y-y0)*dwidth+dx0] = ny[ys];
          Nz[(y-y0)*dwidth+dx0] = nz[ys];
        }
      }
      return true;
    }
    
    template<typename Eval, typename Patch>
      bool stitch_row(const Patch& patch, int subPatch, 
                      const bool bottom, const size_t x0, const size_t x1, const int fine_x, const int coarse_x, 
                      float* Px, float* Py, float* Pz, float* U, float* V, float* Nx, float* Ny, float* Nz, const size_t dy0, const size_t dwidth, const size_t dheight)
    {
      assert(coarse_x <= fine_x);
      if (likely(fine_x == coarse_x))
	return false;
      
      const size_t x0s = stitch(x0,fine_x,coarse_x);
      const size_t x1s = stitch(x1,fine_x,coarse_x);
      assert(x1s-x0s < 4097);
      
      float px[4097], py[4097], pz[4097], u[4097], v[4097], nx[4097], ny[4097], nz[4097]; // FIXME: limits maximal level
      Eval(patch,subPatch, x0s,x1s, bottom,bottom, coarse_x+1,2, px,py,pz,u,v, Nx?nx:nullptr,Ny?ny:nullptr,Nz?nz:nullptr, 4097,1);
      
      for (int x=x0; x<=x1; x++) {
	const size_t xs = stitch(x,fine_x,coarse_x)-x0s;
	Px[dy0*dwidth+x-x0] = px[xs];
        Py[dy0*dwidth+x-x0] = py[xs];
        Pz[dy0*dwidth+x-x0] = pz[xs];
        U [dy0*dwidth+x-x0] = u[xs];
        V [dy0*dwidth+x-x0] = v[xs];
        if (unlikely(Nx != nullptr)) {
          Nx[dy0*dwidth+x-x0] = nx[xs];
          Ny[dy0*dwidth+x-x0] = ny[xs];
          Nz[dy0*dwidth+x-x0] = nz[xs];
        }
      }
      return true;
    }
    
    template<typename Eval, typename Patch>
    void feature_adaptive_eval_grid (const Patch& patch, size_t subPatch, const float levels[4],
                                     const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
                                     float* Px, float* Py, float* Pz, float* U, float* V, float* Nx, float* Ny, float* Nz, const size_t dwidth, const size_t dheight)
    {
      bool sl = false, sr = false, st = false, sb = false;
      if (levels) {
        sl = x0 == 0         && stitch_col<Eval,Patch>(patch,subPatch,0,y0,y1,sheight-1,levels[3], Px,Py,Pz,U,V,Nx,Ny,Nz, 0    ,dwidth,dheight);
        sr = x1 == swidth-1  && stitch_col<Eval,Patch>(patch,subPatch,1,y0,y1,sheight-1,levels[1], Px,Py,Pz,U,V,Nx,Ny,Nz, x1-x0,dwidth,dheight);
        st = y0 == 0         && stitch_row<Eval,Patch>(patch,subPatch,0,x0,x1,swidth-1,levels[0], Px,Py,Pz,U,V,Nx,Ny,Nz, 0    ,dwidth,dheight);
        sb = y1 == sheight-1 && stitch_row<Eval,Patch>(patch,subPatch,1,x0,x1,swidth-1,levels[2], Px,Py,Pz,U,V,Nx,Ny,Nz, y1-y0,dwidth,dheight);
      }
      const size_t ofs = st*dwidth+sl;
      Eval(patch,subPatch,x0+sl,x1-sr,y0+st,y1-sb, swidth,sheight, Px+ofs,Py+ofs,Pz+ofs,U+ofs,V+ofs,Nx?Nx+ofs:nullptr,Ny?Ny+ofs:nullptr,Nz?Nz+ofs:nullptr, dwidth,dheight);
    }
  }
}

