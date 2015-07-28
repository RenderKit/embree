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
#include "feature_adaptive_eval_simd.h"

namespace embree
{
  namespace isa
  {
    template<typename vbool, typename vint, typename vfloat, typename Vertex, typename Vertex_t = Vertex>
      struct PatchEvalSimd
      {
      public:
        
        typedef PatchT<Vertex,Vertex_t> Patch;
        typedef typename Patch::Ref Ref;
        typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;

        PatchEvalSimd (SharedLazyTessellationCache::CacheEntry& entry, size_t commitCounter, 
                       const HalfEdge* edge, const char* vertices, size_t stride, const vbool& valid0, const vfloat& u, const vfloat& v, 
                       float* P, float* dPdu, float* dPdv, const size_t dstride, const size_t N)
        : P(P), dPdu(dPdu), dPdv(dPdv), dstride(dstride), N(N)
        {
          Ref patch = SharedLazyTessellationCache::lookup(entry,commitCounter,[&] () {
              auto alloc = [](size_t bytes) { return SharedLazyTessellationCache::malloc(bytes); };
              return Patch::create(alloc,edge,vertices,stride);
            });
          
          /* use cached data structure for calculations */
          const vbool valid1 = patch ? eval(valid0,patch,u,v,1.0f,0) : vbool(false);
          SharedLazyTessellationCache::unlock();
          const vbool valid2 = valid0 & !valid1;
          if (any(valid2)) {
            FeatureAdaptiveEvalSimd<vbool,vint,vfloat,Vertex,Vertex_t>(edge,vertices,stride,valid2,u,v,P,dPdu,dPdv,dstride,N);
          }
        }
        
        vbool eval_quad(const vbool& valid, const typename Patch::SubdividedQuadPatch* This, const vfloat& u, const vfloat& v, const float dscale, const size_t depth)
        {
          vbool ret = false;
          const vbool u0_mask = u < 0.5f, u1_mask = u >= 0.5f;
          const vbool v0_mask = v < 0.5f, v1_mask = v >= 0.5f;
          const vbool u0v0_mask = valid & u0_mask & v0_mask;
          const vbool u0v1_mask = valid & u0_mask & v1_mask;
          const vbool u1v0_mask = valid & u1_mask & v0_mask;
          const vbool u1v1_mask = valid & u1_mask & v1_mask;
          if (any(u0v0_mask)) ret |= eval(u0v0_mask,This->child[0],2.0f*u,2.0f*v,2.0f*dscale,depth+1);
          if (any(u1v0_mask)) ret |= eval(u1v0_mask,This->child[1],2.0f*u-1.0f,2.0f*v,2.0f*dscale,depth+1);
          if (any(u1v1_mask)) ret |= eval(u1v1_mask,This->child[2],2.0f*u-1.0f,2.0f*v-1.0f,2.0f*dscale,depth+1);
          if (any(u0v1_mask)) ret |= eval(u0v1_mask,This->child[3],2.0f*u,2.0f*v-1.0f,2.0f*dscale,depth+1);
          return ret;
        }
        
        vbool eval_general_triangle(const vbool& valid, const typename Patch::SubdividedGeneralTrianglePatch* This, const vfloat& u, const vfloat& v, const size_t depth)
        {
          vbool ret = false;
          const vbool ab_abc = right_of_line_ab_abc(Vec2<vfloat>(u,v));
          const vbool ac_abc = right_of_line_ac_abc(Vec2<vfloat>(u,v));
          const vbool bc_abc = right_of_line_bc_abc(Vec2<vfloat>(u,v));
          const vbool tri0_mask = valid & !ab_abc &  ac_abc;
          const vbool tri1_mask = valid &  ab_abc & !bc_abc & !tri0_mask;
          const vbool tri2_mask = valid & !tri0_mask & !tri1_mask;
          const vfloat w = 1.0f-u-v;
          
          if  (any(tri0_mask)) {
            const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(u,v));
            ret |= eval(tri0_mask,This->child[0],xy.x,xy.y,1.0f,depth+1);
            if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad0_to_tri(tri0_mask,xy,dPdu,dPdv,dstride,i);
          }
          if (any(tri1_mask)) {
            const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(v,w));
            ret |= eval(tri1_mask,This->child[1],xy.x,xy.y,1.0f,depth+1);
            if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad1_to_tri(tri1_mask,xy,dPdu,dPdv,dstride,i);
          }
          if (any(tri2_mask)) {
            const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(w,u));
            ret |= eval(tri2_mask,This->child[2],xy.x,xy.y,1.0f,depth+1);
            if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad2_to_tri(tri2_mask,xy,dPdu,dPdv,dstride,i);
          }
          return ret;
        }
        
        vbool eval_general(const vbool& valid, const typename Patch::SubdividedGeneralPatch* patch, const vfloat& U, const vfloat& V, const size_t depth)
        {
          vbool ret = false;
          const vint l = (vint)floor(4.0f*U); const vfloat u = 2.0f*frac(4.0f*U); 
          const vint h = (vint)floor(4.0f*V); const vfloat v = 2.0f*frac(4.0f*V); 
          const vint i = (h<<2)+l; assert(all(valid,i<patch->N));
          foreach_unique(valid,i,[&](const vbool& valid, const int i) {
              ret |= eval(valid,patch->child[i],u,v,8.0f,depth+1);
            });
          return ret;
        }
        
        vbool eval(const vbool& valid, Ref This, const vfloat& u, const vfloat& v, const float dscale, const size_t depth) 
        {
          if (!This) return false;
          
          switch (This.type()) 
          {
          case Patch::BILINEAR_PATCH: {
            ((typename Patch::BilinearPatch*)This.object())->patch.eval(valid,u,v,P,dPdu,dPdv,dscale,dstride,N); 
            return valid;
          }
          case Patch::BSPLINE_PATCH: {
            ((typename Patch::BSplinePatch*)This.object())->patch.eval(valid,u,v,P,dPdu,dPdv,dscale,dstride,N);
            return valid;
          }
          case Patch::BEZIER_PATCH: {
            ((typename Patch::BezierPatch*)This.object())->patch.eval(valid,u,v,P,dPdu,dPdv,dscale,dstride,N);
            return valid;
          }
          case Patch::GREGORY_PATCH: {
            ((typename Patch::GregoryPatch*)This.object())->patch.eval(valid,u,v,P,dPdu,dPdv,dscale,dstride,N); 
            return valid;
          }
          case Patch::SUBDIVIDED_QUAD_PATCH: {
            return eval_quad(valid,((typename Patch::SubdividedQuadPatch*)This.object()),u,v,dscale,depth);
          }
          case Patch::SUBDIVIDED_GENERAL_TRIANGLE_PATCH: { 
            assert(dscale == 1.0f); 
            return eval_general_triangle(valid,((typename Patch::SubdividedGeneralTrianglePatch*)This.object()),u,v,depth); 
          }
          case Patch::SUBDIVIDED_GENERAL_PATCH: { 
            assert(dscale == 1.0f); 
            return eval_general(valid,((typename Patch::SubdividedGeneralPatch*)This.object()),u,v,depth); 
          }
          case Patch::EVAL_PATCH: { 
            CatmullClarkPatch patch; patch.deserialize(This.object());
            FeatureAdaptiveEvalSimd<vbool,vint,vfloat,Vertex,Vertex_t>(patch,valid,u,v,dscale,depth,P,dPdu,dPdv,dstride,N);
            return valid;
          }
          default: 
            assert(false); 
            return false;
          }
        }

      private:
        float* const P;
        float* const dPdu;
        float* const dPdv;
        const size_t dstride;
        const size_t N;
      };
  }
}
