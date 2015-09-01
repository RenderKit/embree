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
  namespace isa
  {
    template<typename Vertex, typename Vertex_t = Vertex>
      struct PatchEval
      {
      public:
        
        typedef PatchT<Vertex,Vertex_t> Patch;
        typedef typename Patch::Ref Ref;
        typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
        
        PatchEval (SharedLazyTessellationCache::CacheEntry& entry, size_t commitCounter, 
                   const HalfEdge* edge, const char* vertices, size_t stride, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
        : P(P), dPdu(dPdu), dPdv(dPdv)
        {
          Ref patch = SharedLazyTessellationCache::lookup(entry,commitCounter,[&] () {
              auto alloc = [](size_t bytes) { return SharedLazyTessellationCache::malloc(bytes); };
              return Patch::create(alloc,edge,vertices,stride);
            });
          
          if (patch && eval(patch,u,v,1.0f,0)) {
            SharedLazyTessellationCache::unlock();
            return;
          }
          SharedLazyTessellationCache::unlock();
          FeatureAdaptiveEval<Vertex,Vertex_t>(edge,vertices,stride,u,v,P,dPdu,dPdv);
          PATCH_DEBUG_SUBDIVISION(edge,c,-1,-1);
        }
        
        __forceinline bool eval_quad(const typename Patch::SubdividedQuadPatch* This, const float u, const float v, const float dscale, const size_t depth)
        {
          if (v < 0.5f) {
            if (u < 0.5f) return eval(This->child[0],2.0f*u,2.0f*v,2.0f*dscale,depth+1);
            else          return eval(This->child[1],2.0f*u-1.0f,2.0f*v,2.0f*dscale,depth+1);
          } else {
            if (u > 0.5f) return eval(This->child[2],2.0f*u-1.0f,2.0f*v-1.0f,2.0f*dscale,depth+1);
            else          return eval(This->child[3],2.0f*u,2.0f*v-1.0f,2.0f*dscale,depth+1);
          }
        }
        
        bool eval_general_triangle(const typename Patch::SubdividedGeneralTrianglePatch* This, const float u, const float v, const size_t depth)
        {
          const bool ab_abc = right_of_line_ab_abc(Vec2f(u,v));
          const bool ac_abc = right_of_line_ac_abc(Vec2f(u,v));
          const bool bc_abc = right_of_line_bc_abc(Vec2f(u,v));
          
          const float w = 1.0f-u-v;
          if  (!ab_abc &&  ac_abc) {
            const Vec2f xy = map_tri_to_quad(Vec2f(u,v));
            if (!eval(This->child[0],xy.x,xy.y,1.0f,depth+1)) return false;
            if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
          }
          else if ( ab_abc && !bc_abc) {
            const Vec2f xy = map_tri_to_quad(Vec2f(v,w));
            if (!eval(This->child[1],xy.x,xy.y,1.0f,depth+1)) return false;
            if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
          }
          else {
            const Vec2f xy = map_tri_to_quad(Vec2f(w,u));
            if (!eval(This->child[2],xy.x,xy.y,1.0f,depth+1)) return false;
            if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
          }
          return true;
        }
        
        bool eval_general(const typename Patch::SubdividedGeneralPatch* This, const float U, const float V, const size_t depth)
        {
          const unsigned l = floor(4.0f*U); const float u = 2.0f*frac(4.0f*U); 
          const unsigned h = floor(4.0f*V); const float v = 2.0f*frac(4.0f*V); 
          const unsigned i = 4*h+l; assert(i<This->N);
          return eval(This->child[i],u,v,8.0f,depth+1);
        }
        
        bool eval(Ref This, const float& u, const float& v, const float dscale, const size_t depth) 
        {
          if (!This) return false;
          
          switch (This.type()) 
          {
          case Patch::BILINEAR_PATCH: {
            ((typename Patch::BilinearPatch*)This.object())->patch.eval(u,v,P,dPdu,dPdv,dscale); 
            PATCH_DEBUG_SUBDIVISION(This,-1,c,c);
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
            return eval_quad(((typename Patch::SubdividedQuadPatch*)This.object()),u,v,dscale,depth);
          }
          case Patch::SUBDIVIDED_GENERAL_TRIANGLE_PATCH: { 
            assert(dscale == 1.0f); 
            return eval_general_triangle(((typename Patch::SubdividedGeneralTrianglePatch*)This.object()),u,v,depth); 
          }
          case Patch::SUBDIVIDED_GENERAL_PATCH: { 
            assert(dscale == 1.0f); 
            return eval_general(((typename Patch::SubdividedGeneralPatch*)This.object()),u,v,depth); 
          }
          case Patch::EVAL_PATCH: { 
            CatmullClarkPatch patch; patch.deserialize(This.object());
            FeatureAdaptiveEval<Vertex,Vertex_t>(patch,u,v,dscale,depth,P,dPdu,dPdv);
            return true;
          }
          default: 
            assert(false); 
            return false;
          }
        }
        
      private:
        Vertex* const P;
        Vertex* const dPdu;
        Vertex* const dPdv;
      };
  }
}
  
