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
                               const size_t dwidth, const size_t dheight)
      : x0(x0), x1(x1), y0(y0), y1(y1), swidth(swidth), sheight(sheight), rcp_swidth(1.0f/(swidth-1.0f)), rcp_sheight(1.0f/(sheight-1.0f)), 
        Px(Px), Py(Py), Pz(Pz), U(U), V(V), Nx(Nx), Ny(Ny), Nz(Nz), dwidth(dwidth), dheight(dheight), count(0)
      {
        assert(swidth < (2<<20) && sheight < (2<<20));
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(swidth-1,sheight-1));
        const BBox2f erange(Vec2f(x0,y0),Vec2f(x1,y1));
        
        /* convert into standard quad patch if possible */
        if (likely(patch.isQuadPatch())) 
        {
          CatmullClarkPatch3fa qpatch; patch.init(qpatch);
          eval(qpatch, srange, erange, 0);
          assert(count == (x1-x0+1)*(y1-y0+1));
          return;
        }
        
        /* subdivide patch */
        size_t N;
        array_t<CatmullClarkPatch3fa,GeneralCatmullClarkPatch3fa::SIZE> patches; 
        patch.subdivide(patches,N);
        
        if (N == 4)
        {
          const Vec2f c = srange.center();
          const BBox2f srange0(srange.lower,c);
          const BBox2f srange1(Vec2f(c.x,srange.lower.y),Vec2f(srange.upper.x,c.y));
          const BBox2f srange2(c,srange.upper);
          const BBox2f srange3(Vec2f(srange.lower.x,c.y),Vec2f(c.x,srange.upper.y));

#if PATCH_USE_GREGORY == 2
          BezierCurve3fa borders[GeneralCatmullClarkPatch3fa::SIZE]; patch.getLimitBorder(borders);
          BezierCurve3fa border0l,border0r; borders[0].subdivide(border0l,border0r);
          BezierCurve3fa border1l,border1r; borders[1].subdivide(border1l,border1r);
          BezierCurve3fa border2l,border2r; borders[2].subdivide(border2l,border2r);
          BezierCurve3fa border3l,border3r; borders[3].subdivide(border3l,border3r);
          GeneralCatmullClarkPatch3fa::fix_quad_ring_order(patches);
          eval(patches[0],srange0,intersect(srange0,erange),1,&border0l,nullptr,nullptr,&border3r);
          eval(patches[1],srange1,intersect(srange1,erange),1,&border0r,&border1l,nullptr,nullptr);
          eval(patches[2],srange2,intersect(srange2,erange),1,nullptr,&border1r,&border2l,nullptr);
          eval(patches[3],srange3,intersect(srange3,erange),1,nullptr,nullptr,&border2r,&border3l);
#else
          GeneralCatmullClarkPatch3fa::fix_quad_ring_order(patches);
          eval(patches[0],srange0,intersect(srange0,erange),1);
          eval(patches[1],srange1,intersect(srange1,erange),1);
          eval(patches[2],srange2,intersect(srange2,erange),1);
          eval(patches[3],srange3,intersect(srange3,erange),1);
#endif
        }
        else
        {
          assert(subPatch < N);
          
#if PATCH_USE_GREGORY == 2
          BezierCurve3fa borders[2]; patch.getLimitBorder(borders,subPatch);
          BezierCurve3fa border0l,border0r; borders[0].subdivide(border0l,border0r);
          BezierCurve3fa border2l,border2r; borders[1].subdivide(border2l,border2r);
          eval(patches[subPatch], srange, erange, 1, &border0l, nullptr, nullptr, &border2r);
#else
          eval(patches[subPatch], srange, erange, 1);
#endif
          
        }
        assert(count == (x1-x0+1)*(y1-y0+1));
      }
      
      FeatureAdaptiveEvalGrid (const CatmullClarkPatch3fa& patch,
                               const BBox2f& srange, const BBox2f& erange, const size_t depth,
                               const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight, 
                               float* Px, float* Py, float* Pz, float* U, float* V, 
                               float* Nx, float* Ny, float* Nz,
                               const size_t dwidth, const size_t dheight)
      : x0(x0), x1(x1), y0(y0), y1(y1), swidth(swidth), sheight(sheight), rcp_swidth(1.0f/(swidth-1.0f)), rcp_sheight(1.0f/(sheight-1.0f)), 
        Px(Px), Py(Py), Pz(Pz), U(U), V(V), Nx(Nx), Ny(Ny), Nz(Nz), dwidth(dwidth), dheight(dheight), count(0)
      {
        eval(patch,srange,erange,depth);
      }

      template<typename Patch>
      void evalLocalGrid(const Patch& patch, const BBox2f& srange, const int lx0, const int lx1, const int ly0, const int ly1)
      {
        const float scale_x = rcp(srange.upper.x-srange.lower.x);
        const float scale_y = rcp(srange.upper.y-srange.lower.y);
        count += (lx1-lx0)*(ly1-ly0);
        
#if 0
        for (size_t iy=ly0; iy<ly1; iy++) {
          for (size_t ix=lx0; ix<lx1; ix++) {
            const float lu = select(ix == swidth -1, float(1.0f), (float(ix)-srange.lower.x)*scale_x);
            const float lv = select(iy == sheight-1, float(1.0f), (float(iy)-srange.lower.y)*scale_y);
            const Vec3fa p = patch.eval(lu,lv);
            const float u = float(ix)*rcp_swidth;
            const float v = float(iy)*rcp_sheight;
            const int ofs = (iy-y0)*dwidth+(ix-x0);
            Px[ofs] = p.x;
            Py[ofs] = p.y;
            Pz[ofs] = p.z;
            U[ofs] = u;
            V[ofs] = v;
          }
        }
#else
        foreach2(lx0,lx1,ly0,ly1,[&](const vbool& valid, const vint& ix, const vint& iy) {
            const vfloat lu = select(ix == swidth -1, vfloat(1.0f), (vfloat(ix)-srange.lower.x)*scale_x);
            const vfloat lv = select(iy == sheight-1, vfloat(1.0f), (vfloat(iy)-srange.lower.y)*scale_y);
            const Vec3<vfloat> p = patch.eval(lu,lv);
            Vec3<vfloat> n = zero;
            if (unlikely(Nx != nullptr)) n = normalize_safe(patch.normal(lu,lv));
            const vfloat u = vfloat(ix)*rcp_swidth;
            const vfloat v = vfloat(iy)*rcp_sheight;
            const vint ofs = (iy-y0)*dwidth+(ix-x0);
            if (likely(all(valid)) && all(iy==iy[0])) {
              const size_t ofs2 = ofs[0];
              vfloat::storeu(Px+ofs2,p.x);
              vfloat::storeu(Py+ofs2,p.y);
              vfloat::storeu(Pz+ofs2,p.z);
              vfloat::storeu(U+ofs2,u);
              vfloat::storeu(V+ofs2,v);
              if (unlikely(Nx != nullptr)) {
                vfloat::storeu(Nx+ofs2,n.x);
                vfloat::storeu(Ny+ofs2,n.y);
                vfloat::storeu(Nz+ofs2,n.z);
              }
            } else {
              foreach_unique_index(valid,iy,[&](const vbool& valid, const int iy0, const int j) {
                  const size_t ofs2 = ofs[j]-j;
                  vfloat::storeu(valid,Px+ofs2,p.x);
                  vfloat::storeu(valid,Py+ofs2,p.y);
                  vfloat::storeu(valid,Pz+ofs2,p.z);
                  vfloat::storeu(valid,U+ofs2,u);
                  vfloat::storeu(valid,V+ofs2,v);
                  if (unlikely(Nx != nullptr)) {
                    vfloat::storeu(valid,Nx+ofs2,n.x);
                    vfloat::storeu(valid,Ny+ofs2,n.y);
                    vfloat::storeu(valid,Nz+ofs2,n.z);
                  }
                });
            }
          });
#endif
      }
      
      __forceinline bool final(const CatmullClarkPatch3fa& patch, const CatmullClarkRing::Type type, size_t depth) 
      {
        const int max_eval_depth = (type & CatmullClarkRing::TYPE_CREASES) ? PATCH_MAX_EVAL_DEPTH_CREASE : PATCH_MAX_EVAL_DEPTH_IRREGULAR;
//#if PATCH_MIN_RESOLUTION
//        return patch.isFinalResolution(PATCH_MIN_RESOLUTION) || depth>=max_eval_depth;
//#else
        return depth>=max_eval_depth;
//#endif
      }
      
      void eval(const CatmullClarkPatch3fa& patch, const BBox2f& srange, const BBox2f& erange, const size_t depth, 
                const BezierCurve3fa* border0 = nullptr, const BezierCurve3fa* border1 = nullptr, const BezierCurve3fa* border2 = nullptr, const BezierCurve3fa* border3 = nullptr)
      {
        if (erange.empty())
          return;
        
        int lx0 = ceilf(erange.lower.x);
        int lx1 = ceilf(erange.upper.x) + (erange.upper.x == x1 && (srange.lower.x < erange.upper.x || erange.upper.x == 0));
        int ly0 = ceilf(erange.lower.y);
        int ly1 = ceilf(erange.upper.y) + (erange.upper.y == y1 && (srange.lower.y < erange.upper.y || erange.upper.y == 0));
        if (lx0 >= lx1 || ly0 >= ly1) return;

        CatmullClarkPatch::Type ty = patch.type();

        if (unlikely(final(patch,ty,depth)))
        {
          if (ty & CatmullClarkRing::TYPE_REGULAR) {
            RegularPatch rpatch(patch,border0,border1,border2,border3);
            evalLocalGrid(rpatch,srange,lx0,lx1,ly0,ly1);
            return;
          } else {
            IrregularFillPatch ipatch(patch,border0,border1,border2,border3);
            evalLocalGrid(ipatch,srange,lx0,lx1,ly0,ly1);
            return;
          }
        }
        else if (ty & CatmullClarkRing::TYPE_REGULAR_CREASES) { 
          assert(depth > 0); 
          RegularPatch rpatch(patch,border0,border1,border2,border3);
          evalLocalGrid(rpatch,srange,lx0,lx1,ly0,ly1);
          return;
        }
#if PATCH_USE_GREGORY == 2
        else if (ty & CatmullClarkRing::TYPE_GREGORY_CREASES) { 
          assert(depth > 0); 
          GregoryPatch gpatch(patch,border0,border1,border2,border3);
          evalLocalGrid(gpatch,srange,lx0,lx1,ly0,ly1);
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

