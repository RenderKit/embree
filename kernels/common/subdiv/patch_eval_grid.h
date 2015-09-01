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
#include "feature_adaptive_eval_grid.h"

namespace embree
{
  namespace isa 
  {
    struct PatchEvalGrid
    {
      typedef Patch3fa Patch;
      typedef Patch::Ref Ref;
      typedef GeneralCatmullClarkPatch3fa GeneralCatmullClarkPatch;
      typedef CatmullClarkPatch3fa CatmullClarkPatch;
      typedef BSplinePatch3fa BSplinePatch;
      typedef BezierPatch3fa BezierPatch;
      typedef GregoryPatch3fa GregoryPatch;
      typedef BilinearPatch3fa BilinearPatch;

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

      PatchEvalGrid (Ref patch, size_t subPatch,
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
        bool done = eval(patch,subPatch,srange,erange);
        assert(done);
        assert(count == (x1-x0+1)*(y1-y0+1));
      }

      template<typename Patch>
      __forceinline void evalLocalGrid(const Patch* patch, const BBox2f& srange, const int lx0, const int lx1, const int ly0, const int ly1)
      {
        const float scale_x = rcp(srange.upper.x-srange.lower.x);
        const float scale_y = rcp(srange.upper.y-srange.lower.y);
        count += (lx1-lx0)*(ly1-ly0);
        
#if 0
        for (size_t iy=ly0; iy<ly1; iy++) {
          for (size_t ix=lx0; ix<lx1; ix++) {
            const float lu = select(ix == swidth -1, float(1.0f), (float(ix)-srange.lower.x)*scale_x);
            const float lv = select(iy == sheight-1, float(1.0f), (float(iy)-srange.lower.y)*scale_y);
            const Vec3fa p = patch->patch.eval(lu,lv);
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
            const Vec3<vfloat> p = patch->patch.eval(lu,lv);
            Vec3<vfloat> n = zero;
            if (unlikely(Nx != nullptr)) n = normalize_safe(patch->patch.normal(lu,lv));
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

      bool eval(Ref This, const BBox2f& srange, const BBox2f& erange, const size_t depth) 
      {
        if (erange.empty())
          return true;
        
        const int lx0 = ceilf(erange.lower.x);
        const int lx1 = ceilf(erange.upper.x) + (erange.upper.x == x1 && (srange.lower.x < erange.upper.x || erange.upper.x == 0));
        const int ly0 = ceilf(erange.lower.y);
        const int ly1 = ceilf(erange.upper.y) + (erange.upper.y == y1 && (srange.lower.y < erange.upper.y || erange.upper.y == 0));
        if (lx0 >= lx1 || ly0 >= ly1) 
          return true;

        if (!This) 
          return false;
        
        switch (This.type()) 
        {
        case Patch::BILINEAR_PATCH: {
          evalLocalGrid((Patch::BilinearPatch*)This.object(),srange,lx0,lx1,ly0,ly1);
          return true;
        }
        case Patch::BSPLINE_PATCH: {
          evalLocalGrid((Patch::BSplinePatch*)This.object(),srange,lx0,lx1,ly0,ly1);
          return true;
        }
        case Patch::BEZIER_PATCH: {
          evalLocalGrid((Patch::BezierPatch*)This.object(),srange,lx0,lx1,ly0,ly1);
          return true;
        }
        case Patch::GREGORY_PATCH: {
          evalLocalGrid((Patch::GregoryPatch*)This.object(),srange,lx0,lx1,ly0,ly1);
          return true;
        }
        case Patch::SUBDIVIDED_QUAD_PATCH: 
        {
          const Vec2f c = srange.center();
          const BBox2f srange0(srange.lower,c);
          const BBox2f srange1(Vec2f(c.x,srange.lower.y),Vec2f(srange.upper.x,c.y));
          const BBox2f srange2(c,srange.upper);
          const BBox2f srange3(Vec2f(srange.lower.x,c.y),Vec2f(c.x,srange.upper.y));
          
          Patch::SubdividedQuadPatch* patch = (Patch::SubdividedQuadPatch*)This.object();
          eval(patch->child[0],srange0,intersect(srange0,erange),depth+1);
          eval(patch->child[1],srange1,intersect(srange1,erange),depth+1);
          eval(patch->child[2],srange2,intersect(srange2,erange),depth+1);
          eval(patch->child[3],srange3,intersect(srange3,erange),depth+1);
          return true;
        }
        case Patch::EVAL_PATCH: { 
          CatmullClarkPatch patch; patch.deserialize(This.object());
          FeatureAdaptiveEvalGrid(patch,srange,erange,depth,x0,x1,y0,y1,swidth,sheight,Px,Py,Pz,U,V,Nx,Ny,Nz,dwidth,dheight);
          count += (lx1-lx0)*(ly1-ly0);
          return true;
        }
        default: 
          assert(false); 
          return false;
        }
      }

      bool eval(Ref This, size_t subPatch, const BBox2f& srange, const BBox2f& erange) 
      {
        if (!This) 
          return false;

        switch (This.type()) 
        {
        case Patch::SUBDIVIDED_GENERAL_TRIANGLE_PATCH: { 
          Patch::SubdividedGeneralTrianglePatch* patch = (Patch::SubdividedGeneralTrianglePatch*)This.object();
          assert(subPatch < 3);
          return eval(patch->child[subPatch],srange,erange,1);
        }
        case Patch::SUBDIVIDED_GENERAL_PATCH: { 
          Patch::SubdividedGeneralPatch* patch = (Patch::SubdividedGeneralPatch*)This.object();
          assert(subPatch < patch->N);
          return eval(patch->child[subPatch],srange,erange,1);
        }
        default: 
          assert(subPatch == 0);
          return eval(This,srange,erange,0);
        }
      }
    };

    __forceinline size_t patch_eval_subdivision_count (const HalfEdge* h)
    {
      const size_t N = h->numEdges();
      if (N == 4) return 1;
      else return N;
    }
    
    template<typename Tessellator>
      inline void patch_eval_subdivision (const HalfEdge* h, Tessellator tessellator)
    {
      const size_t N = h->numEdges();
      int neighborSubdiv[GeneralCatmullClarkPatch3fa::SIZE]; // FIXME: use array_t
      float levels[GeneralCatmullClarkPatch3fa::SIZE];
      for (size_t i=0; i<N; i++) {
        assert(i<GeneralCatmullClarkPatch3fa::SIZE);
        neighborSubdiv[i] = h->hasOpposite() ? h->opposite()->numEdges() != 4 : 0; 
        levels[i] = h->edge_level;
        h = h->next();
      }
      
      if (N == 4)
      {
        const Vec2f uv[4] = { Vec2f(0.0f,0.0f), Vec2f(1.0f,0.0f), Vec2f(1.0f,1.0f), Vec2f(0.0f,1.0f) };
        tessellator(uv,neighborSubdiv,levels,0);
      }
      else if (N == 3)
      {
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
        const int neighborSubdiv0[4] = { 0,0,0,0 };
        const int neighborSubdiv1[4] = { 0,0,0,0 };
        const int neighborSubdiv2[4] = { 0,0,0,0 };
        const float levels0[4] = { 0.5f*levels[0], 0.5f*levels[0], 0.5f*levels[2], 0.5f*levels[2] };
        const float levels1[4] = { 0.5f*levels[1], 0.5f*levels[1], 0.5f*levels[0], 0.5f*levels[0] };
        const float levels2[4] = { 0.5f*levels[2], 0.5f*levels[2], 0.5f*levels[1], 0.5f*levels[1] };
        tessellator(uv0, neighborSubdiv0, levels0, 0);
        tessellator(uv1, neighborSubdiv1, levels1, 1);
        tessellator(uv2, neighborSubdiv2, levels2, 2);
      }
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
          const int neighborSubdiv1[4] = { 0,0,0,0 }; 
          const float levels1[4] = { 0.5f*levels[(i+0)%N], 0.5f*levels[(i+0)%N], 0.5f*levels[(i-1)%N], 0.5f*levels[(i-1)%N] };
          tessellator(uv,neighborSubdiv1,levels1,i);
        }
      }
    }
  }
}

