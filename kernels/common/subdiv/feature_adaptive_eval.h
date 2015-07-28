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

namespace embree
{
  namespace isa
  {
    template<typename Vertex, typename Vertex_t = Vertex>
      struct FeatureAdaptiveEval
      {
      public:
        
        typedef PatchT<Vertex,Vertex_t> Patch;
        typedef typename Patch::Ref Ref;
        typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
        typedef CatmullClark1RingT<Vertex,Vertex_t> CatmullClarkRing;
        typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
        typedef BSplinePatchT<Vertex,Vertex_t> BSplinePatch;
        typedef BezierPatchT<Vertex,Vertex_t> BezierPatch;
        typedef GregoryPatchT<Vertex,Vertex_t> GregoryPatch;
        typedef BilinearPatchT<Vertex,Vertex_t> BilinearPatch;
        
      public:
        
        FeatureAdaptiveEval (const HalfEdge* edge, const char* vertices, size_t stride, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
        : P(P), dPdu(dPdu), dPdv(dPdv)
        {
          switch (edge->patch_type) {
          case HalfEdge::REGULAR_QUAD_PATCH: RegularPatchT(edge,vertices,stride).eval(u,v,P,dPdu,dPdv,1.0f); break;
#if PATCH_USE_GREGORY == 2
          case HalfEdge::IRREGULAR_QUAD_PATCH: GregoryPatch(edge,vertices,stride).eval(u,v,P,dPdu,dPdv,1.0f); break;
#endif
          default: {
            GeneralCatmullClarkPatch patch(edge,vertices,stride);
            eval(patch,Vec2f(u,v),0);
            break;
          }
          }
        }
        
        void eval_general_triangle(array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2f& uv, size_t depth)
        {
          const bool ab_abc = right_of_line_ab_abc(uv);
          const bool ac_abc = right_of_line_ac_abc(uv);
          const bool bc_abc = right_of_line_bc_abc(uv);
          
          const float u = uv.x, v = uv.y, w = 1.0f-u-v;
          if  (!ab_abc &&  ac_abc) {
            const Vec2f xy = map_tri_to_quad(Vec2f(u,v));
            eval(patches[0],xy,1.0f,depth+1);
            if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
          }
          else if ( ab_abc && !bc_abc) {
            const Vec2f xy = map_tri_to_quad(Vec2f(v,w));
            eval(patches[1],xy,1.0f,depth+1);
            if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
          }
          else {
            const Vec2f xy = map_tri_to_quad(Vec2f(w,u));
            eval(patches[2],xy,1.0f,depth+1);
            if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
          }
        }
        
        void eval_general_quad(array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2f& uv, size_t depth)
        {
          float u = uv.x, v = uv.y;
          if (v < 0.5f) {
            if (u < 0.5f) {
              eval(patches[0],Vec2f(2.0f*u,2.0f*v),2.0f,depth+1);
              if (dPdu && dPdv) {
                const Vertex dpdx = *dPdu, dpdy = *dPdv;
                *dPdu = dpdx; *dPdv = dpdy;
              }
            }
            else {
              eval(patches[1],Vec2f(2.0f*v,2.0f-2.0f*u),2.0f,depth+1);
              if (dPdu && dPdv) {
                const Vertex dpdx = *dPdu, dpdy = *dPdv;
                *dPdu = -dpdy; *dPdv = dpdx;
              }
            }
          } else {
            if (u > 0.5f) {
              eval(patches[2],Vec2f(2.0f-2.0f*u,2.0f-2.0f*v),2.0f,depth+1);
              if (dPdu && dPdv) {
                const Vertex dpdx = *dPdu, dpdy = *dPdv;
                *dPdu = -dpdx; *dPdv = -dpdy;
              }
            }
            else {
              eval(patches[3],Vec2f(2.0f-2.0f*v,2.0f*u),2.0f,depth+1);
              if (dPdu && dPdv) {
                const Vertex dpdx = *dPdu, dpdy = *dPdv;
                *dPdu = dpdy; *dPdv = -dpdx;
              }
            }
          }
        }

        __forceinline bool final(const CatmullClarkPatch& patch, size_t depth) 
        {
#if PATCH_MIN_RESOLUTION
          return patch.isFinalResolution(PATCH_MIN_RESOLUTION) || depth>=PATCH_MAX_EVAL_DEPTH;
#else
          return depth>=PATCH_MAX_EVAL_DEPTH;
#endif
        }
        
        void eval(CatmullClarkPatch& patch, Vec2f uv, float dscale, size_t depth)
        {
          while (true) 
          {
            typename CatmullClarkPatch::Type ty = patch.type();

            if (unlikely(final(patch,depth)))
            {
              if (ty & CatmullClarkRing::TYPE_REGULAR) { 
                RegularPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale); return;
              } else {
                IrregularFillPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale); return;
              }
            }
            else if (ty & CatmullClarkRing::TYPE_REGULAR_CREASES) { 
              assert(depth > 0); RegularPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale); return;
            }
#if PATCH_USE_GREGORY == 2
            else if (ty & CatmullClarkRing::TYPE_GREGORY_CREASES) { 
              assert(depth > 0); GregoryPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale); return;
            }
#endif
            else
            {
              array_t<CatmullClarkPatch,4> patches; 
              patch.subdivide(patches); // FIXME: only have to generate one of the patches
              
              const float u = uv.x, v = uv.y;
              if (v < 0.5f) {
                if (u < 0.5f) { patch = patches[0]; uv = Vec2f(2.0f*u,2.0f*v); dscale *= 2.0f; }
                else          { patch = patches[1]; uv = Vec2f(2.0f*u-1.0f,2.0f*v); dscale *= 2.0f; }
              } else {
                if (u > 0.5f) { patch = patches[2]; uv = Vec2f(2.0f*u-1.0f,2.0f*v-1.0f); dscale *= 2.0f; }
                else          { patch = patches[3]; uv = Vec2f(2.0f*u,2.0f*v-1.0f); dscale *= 2.0f; }
              }
              depth++;
            }
          }
        }
        
        void eval(const GeneralCatmullClarkPatch& patch, const Vec2f& uv, const size_t depth) 
        {
          /* convert into standard quad patch if possible */
          if (likely(patch.isQuadPatch())) 
          {
            CatmullClarkPatch qpatch; patch.init(qpatch);
            return eval(qpatch,uv,1.0f,depth); 
          }
          
          /* subdivide patch */
          size_t N;
          array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
          patch.subdivide(patches,N); // FIXME: only have to generate one of the patches
          
          /* parametrization for triangles */
          if (N == 3)  // FIXME: border handling
            eval_general_triangle(patches,uv,depth);
          
          /* parametrization for quads */
          else if (N == 4) // FIXME: border handling
            eval_general_quad(patches,uv,depth);
          
          /* parametrization for arbitrary polygons */
          else { // FIXME: border handling
            const unsigned l = floor(4.0f*uv.x); const float u = 2.0f*frac(4.0f*uv.x); 
            const unsigned h = floor(4.0f*uv.y); const float v = 2.0f*frac(4.0f*uv.y); 
            const unsigned i = 4*h+l; assert(i<N);
            eval(patches[i],Vec2f(u,v),8.0f,depth+1);
          }
        }
        
      private:
        Vertex* const P;
        Vertex* const dPdu;
        Vertex* const dPdv;
      };
  }
}
