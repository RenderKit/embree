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
#include "feature_adaptive_eval.h"

namespace embree
{
  template<typename Vertex, typename Vertex_t = Vertex>
    struct PatchEval
    {
    public:
      
      typedef PatchT<Vertex,Vertex_t> Patch;
      typedef typename Patch::Ref Ref;
      typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
      typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
      typedef BSplinePatchT<Vertex,Vertex_t> BSplinePatch;
      typedef BezierPatchT<Vertex,Vertex_t> BezierPatch;
      typedef GregoryPatchT<Vertex,Vertex_t> GregoryPatch;
      typedef BilinearPatchT<Vertex,Vertex_t> BilinearPatch;
      
      __forceinline static bool eval_general_triangle(const typename Patch::SubdividedGeneralTrianglePatch* This, size_t subPatch, const float x, const float y, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        assert(subPatch < 3);
        if (!eval(This->child[subPatch],x,y,P,dPdu,dPdv,1.0f)) return false;
        if (!dPdu || !dPdv) return true; 
        switch (subPatch) {
        case 0: map_quad0_to_tri(Vec2f(x,y),*dPdu,*dPdv); break;
        case 1: map_quad1_to_tri(Vec2f(x,y),*dPdu,*dPdv); break;
        case 2: map_quad2_to_tri(Vec2f(x,y),*dPdu,*dPdv); break;
        }
        return true;
      }

      static bool eval_general_triangle(const typename Patch::SubdividedGeneralTrianglePatch* This, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        const bool ab_abc = right_of_line_ab_abc(Vec2f(u,v));
        const bool ac_abc = right_of_line_ac_abc(Vec2f(u,v));
        const bool bc_abc = right_of_line_bc_abc(Vec2f(u,v));
        
        const float w = 1.0f-u-v;
        if  (!ab_abc &&  ac_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(u,v));
          if (!eval(This->child[0],xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
          if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
        }
        else if ( ab_abc && !bc_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(v,w));
          if (!eval(This->child[1],xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
          if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
        }
        else {
          const Vec2f xy = map_tri_to_quad(Vec2f(w,u));
          if (!eval(This->child[2],xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
          if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
        }
        return true;
      }
      
      static bool eval_quad(const typename Patch::SubdividedQuadPatch* This, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale)
      {
        if (v < 0.5f) {
          if (u < 0.5f) return eval(This->child[0],2.0f*u,2.0f*v,P,dPdu,dPdv,2.0f*dscale);
          else          return eval(This->child[1],2.0f*u-1.0f,2.0f*v,P,dPdu,dPdv,2.0f*dscale);
        } else {
          if (u > 0.5f) return eval(This->child[2],2.0f*u-1.0f,2.0f*v-1.0f,P,dPdu,dPdv,2.0f*dscale);
          else          return eval(This->child[3],2.0f*u,2.0f*v-1.0f,P,dPdu,dPdv,2.0f*dscale);
        }
      }
      
      __forceinline static bool eval_general(const typename Patch::SubdividedGeneralPatch* This, size_t subPatch, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        assert(subPatch < This->N);
        return eval(This->child[subPatch],u,v,P,dPdu,dPdv,1.0f);
      }

      static bool eval_general(const typename Patch::SubdividedGeneralPatch* This, const float U, const float V, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        const unsigned l = floor(4.0f*U); const float u = 2.0f*frac(4.0f*U); 
        const unsigned h = floor(4.0f*V); const float v = 2.0f*frac(4.0f*V); 
        const unsigned i = 4*h+l; assert(i<This->N);
        return eval(This->child[i],u,v,P,dPdu,dPdv,8.0f);
      }
      
      static bool eval(Ref This, const float& u, const float& v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale) 
      {
        if (!This) return false;
        
        switch (This.type()) 
        {
        case Patch::BILINEAR_PATCH: {
          ((typename Patch::BilinearPatch*)This.object())->patch.eval(u,v,P,dPdu,dPdv,dscale); 
          PATCH_DEBUG_SUBDIVISION(This,c,c,-1);
          return true;
        }
        case Patch::BSPLINE_PATCH: {
          ((typename Patch::BSplinePatch*)This.object())->patch.eval(u,v,P,dPdu,dPdv,dscale);
          PATCH_DEBUG_SUBDIVISION(This,-1,c,-1);
          return true;
        }
        case Patch::BEZIER_PATCH: {
          ((typename Patch::BezierPatch*)This.object())->patch.eval(u,v,P,dPdu,dPdv,dscale);
          PATCH_DEBUG_SUBDIVISION(This,-1,c,-1);
          return true;
        }
        case Patch::GREGORY_PATCH: {
          ((typename Patch::GregoryPatch*)This.object())->patch.eval(u,v,P,dPdu,dPdv,dscale); 
          PATCH_DEBUG_SUBDIVISION(This,-1,-1,c);
          return true;
        }
        case Patch::SUBDIVIDED_QUAD_PATCH: {
          return eval_quad(((typename Patch::SubdividedQuadPatch*)This.object()),u,v,P,dPdu,dPdv,dscale);
        }
        case Patch::SUBDIVIDED_GENERAL_TRIANGLE_PATCH: { 
          assert(dscale == 1.0f); 
          return eval_general_triangle(((typename Patch::SubdividedGeneralTrianglePatch*)This.object()),u,v,P,dPdu,dPdv); 
        }
        case Patch::SUBDIVIDED_GENERAL_PATCH: { 
          assert(dscale == 1.0f); 
          return eval_general(((typename Patch::SubdividedGeneralPatch*)This.object()),u,v,P,dPdu,dPdv); 
        }
        default: 
          assert(false); 
          return false;
        }
      }

      static bool eval(Ref This, size_t subPatch, const float& u, const float& v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale) 
      {
        if (!This) return false;
        
        switch (This.type()) 
        {
        case Patch::SUBDIVIDED_GENERAL_TRIANGLE_PATCH: { 
          assert(dscale == 1.0f); 
          return eval_general_triangle(((typename Patch::SubdividedGeneralTrianglePatch*)This.object()),subPatch,u,v,P,dPdu,dPdv); 
        }
        case Patch::SUBDIVIDED_GENERAL_PATCH: { 
          assert(dscale == 1.0f); 
          return eval_general(((typename Patch::SubdividedGeneralPatch*)This.object()),subPatch,u,v,P,dPdu,dPdv); 
        }
        default: 
          assert(subPatch == 0);
          return eval(This,u,v,P,dPdu,dPdv,dscale);
        }
      }

      static void eval (SharedLazyTessellationCache::CacheEntry& entry, size_t commitCounter, 
                        const HalfEdge* edge, const char* vertices, size_t stride, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        Ref patch = SharedLazyTessellationCache::lookup(entry,commitCounter,[&] () {
            auto alloc = [](size_t bytes) { return SharedLazyTessellationCache::malloc(bytes); };
            return Patch::create(alloc,edge,vertices,stride);
          });
        
        if (patch && eval(patch,u,v,P,dPdu,dPdv,1.0f)) {
          SharedLazyTessellationCache::unlock();
          return;
        }
        SharedLazyTessellationCache::unlock();
        FeatureAdaptiveEval<Vertex,Vertex_t>::eval_direct (edge,vertices,stride,u,v,P,dPdu,dPdv);
        PATCH_DEBUG_SUBDIVISION(edge,c,-1,-1);
      }

      static void eval (SharedLazyTessellationCache::CacheEntry& entry, size_t commitCounter, 
                        const HalfEdge* edge, const size_t subPatch, const char* vertices, size_t stride, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        Ref patch = SharedLazyTessellationCache::lookup(entry,commitCounter,[&] () {
            auto alloc = [](size_t bytes) { return SharedLazyTessellationCache::malloc(bytes); };
            return Patch::create(alloc,edge,vertices,stride);
          });
        
        if (patch && eval(patch,subPatch,u,v,P,dPdu,dPdv,1.0f)) {
          SharedLazyTessellationCache::unlock();
          return;
        }
        SharedLazyTessellationCache::unlock();
        FeatureAdaptiveEval<Vertex,Vertex_t>::eval_direct (edge,subPatch,vertices,stride,u,v,P,dPdu,dPdv);
        PATCH_DEBUG_SUBDIVISION(edge,c,-1,-1);
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

#if !defined(__MIC__)
  __forceinline void interpolate( SubdivMesh* mesh, SharedLazyTessellationCache::CacheEntry& entry, char* src, size_t stride, 
                                  unsigned primID, unsigned subPrim, float u, float v, Vec3fa& P, Vec3fa& dPdu, Vec3fa& dPdv)
  {
#if defined(__AVX__)
    if (stride > 4) {
      float8 P8,dPdu8,dPdv8;
      PatchEval<float8>::eval(entry,mesh->parent->commitCounter,mesh->getHalfEdge(primID),subPrim,src,stride,u,v,&P8,&dPdu8,&dPdv8);
      P = (Vec3fa) extract<0>(P8); dPdu = (Vec3fa) extract<0>(dPdu8); dPdv = (Vec3fa) extract<0>(dPdv8);
    } else 
#endif
    {
      float4 P4,dPdu4,dPdv4;
      PatchEval<float4>::eval(entry,mesh->parent->commitCounter,mesh->getHalfEdge(primID),subPrim,src,stride,u,v,&P4,&dPdu4,&dPdv4);
      P = (Vec3fa) P4; dPdu = (Vec3fa) dPdu4; dPdv = (Vec3fa) dPdv4;
    }
  }
#endif
}
