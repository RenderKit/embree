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
  template<typename Vertex, typename Vertex_t = Vertex>
    struct FeatureAdaptiveEval
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
      
      static void eval_general_triangle_direct(array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2f& uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, size_t depth)
      {
        const bool ab_abc = right_of_line_ab_abc(uv);
        const bool ac_abc = right_of_line_ac_abc(uv);
        const bool bc_abc = right_of_line_bc_abc(uv);
        
        const float u = uv.x, v = uv.y, w = 1.0f-u-v;
        if  (!ab_abc &&  ac_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(u,v));
          eval_direct(patches[0],xy,P,dPdu,dPdv,1.0f,depth+1);
          if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
        }
        else if ( ab_abc && !bc_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(v,w));
          eval_direct(patches[1],xy,P,dPdu,dPdv,1.0f,depth+1);
          if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
        }
        else {
          const Vec2f xy = map_tri_to_quad(Vec2f(w,u));
          eval_direct(patches[2],xy,P,dPdu,dPdv,1.0f,depth+1);
          if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
        }
      }
      
      __forceinline static void eval_quad_direct(CatmullClarkPatch& patch, Vec2f& uv, float& dscale)
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
      }
      
      static void eval_general_quad_direct(array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2f& uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, size_t depth)
      {
        float u = uv.x, v = uv.y;
        if (v < 0.5f) {
          if (u < 0.5f) {
            eval_direct(patches[0],Vec2f(2.0f*u,2.0f*v),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = dpdx; *dPdv = dpdy;
            }
          }
          else {
            eval_direct(patches[1],Vec2f(2.0f*v,2.0f-2.0f*u),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -dpdy; *dPdv = dpdx;
            }
          }
        } else {
          if (u > 0.5f) {
            eval_direct(patches[2],Vec2f(2.0f-2.0f*u,2.0f-2.0f*v),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -dpdx; *dPdv = -dpdy;
            }
          }
          else {
            eval_direct(patches[3],Vec2f(2.0f-2.0f*v,2.0f*u),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = dpdy; *dPdv = -dpdx;
            }
          }
        }
      }

      static void eval_direct(const GeneralCatmullClarkPatch& patch, const Vec2f& uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, const size_t depth) 
      {
        /* convert into standard quad patch if possible */
        if (likely(patch.isQuadPatch())) 
        {
          CatmullClarkPatch qpatch; patch.init(qpatch);
          return eval_direct(qpatch,uv,P,dPdu,dPdv,1.0f,depth); 
        }
        
        /* subdivide patch */
        size_t N;
        array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
        patch.subdivide(patches,N); // FIXME: only have to generate one of the patches
        
        /* parametrization for triangles */
        if (N == 3) 
          eval_general_triangle_direct(patches,uv,P,dPdu,dPdv,depth);
        
        /* parametrization for quads */
        else if (N == 4) 
          eval_general_quad_direct(patches,uv,P,dPdu,dPdv,depth);
        
        /* parametrization for arbitrary polygons */
        else {
          const unsigned l = floor(4.0f*uv.x); const float u = 2.0f*frac(4.0f*uv.x); 
          const unsigned h = floor(4.0f*uv.y); const float v = 2.0f*frac(4.0f*uv.y); 
          const unsigned i = 4*h+l; assert(i<N);
          eval_direct(patches[i],Vec2f(u,v),P,dPdu,dPdv,8.0f,depth+1);
        }
      }

      static void eval_direct(const GeneralCatmullClarkPatch& patch, size_t subPatch, const Vec2f& uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, const size_t depth) 
      {
        /* convert into standard quad patch if possible */
        if (likely(patch.isQuadPatch())) 
        {
          CatmullClarkPatch qpatch; patch.init(qpatch);
          return eval_direct(qpatch,uv,P,dPdu,dPdv,1.0f,depth); 
        }
        
        /* subdivide patch */
        size_t N;
        array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
        patch.subdivide(patches,N); // FIXME: only have to generate one of the patches
        
        /* parametrization for triangles */
        if (N == 3) 
          eval_direct(patches[subPatch],uv,P,dPdu,dPdv,1.0f,depth+1);
        
        /* parametrization for quads */
        else if (N == 4) 
          eval_general_quad_direct(patches,uv,P,dPdu,dPdv,depth);
        
        /* parametrization for arbitrary polygons */
        else
          eval_direct(patches[subPatch],uv,P,dPdu,dPdv,1.0f,depth+1);
      }
      
      static void eval_direct(CatmullClarkPatch& patch, Vec2f uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, float dscale, size_t depth)
      {
        while (true) 
        {
          if (unlikely(patch.isRegular2())) { 
            assert(depth > 0); RegularPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale); return;
          }
#if PATCH_USE_GREGORY == 2
          else if (unlikely(depth>=PATCH_MAX_EVAL_DEPTH || patch.isGregory())) {
            assert(depth > 0); GregoryPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale); return;
          }
#else
          else if (unlikely(depth>=PATCH_MAX_EVAL_DEPTH))
          {
#if PATCH_USE_GREGORY == 1
            GregoryPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale);
#else
            BilinearPatch(patch).eval(uv.x,uv.y,P,dPdu,dPdv,dscale);
#endif
            return;
          }
#endif
          else {
            eval_quad_direct(patch,uv,dscale);
            depth++;
          }
        }
      }  
      
      static void eval_direct (const HalfEdge* edge, const char* vertices, size_t stride, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        auto loader = [&](const HalfEdge* p) -> Vertex { 
          const unsigned vtx = p->getStartVertexIndex();
          return Vertex_t::loadu((float*)&vertices[vtx*stride]); 
        };
        
        switch (edge->patch_type) {
        case HalfEdge::REGULAR_QUAD_PATCH: RegularPatchT(edge,loader).eval(u,v,P,dPdu,dPdv); break;
#if PATCH_USE_GREGORY == 2
        case HalfEdge::IRREGULAR_QUAD_PATCH: GregoryPatchT<Vertex,Vertex_t>(edge,loader).eval(u,v,P,dPdu,dPdv); break;
#endif
        default: {
          GeneralCatmullClarkPatch patch(edge,loader);
          eval_direct(patch,Vec2f(u,v),P,dPdu,dPdv,0);
          break;
        }
        }
      }

      static void eval_direct (const HalfEdge* edge, size_t subPatch, const char* vertices, size_t stride, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        auto loader = [&](const HalfEdge* p) -> Vertex { 
          const unsigned vtx = p->getStartVertexIndex();
          return Vertex_t::loadu((float*)&vertices[vtx*stride]); 
        };
        
        switch (edge->patch_type) {
        case HalfEdge::REGULAR_QUAD_PATCH: RegularPatchT(edge,loader).eval(u,v,P,dPdu,dPdv); break;
#if PATCH_USE_GREGORY == 2
        case HalfEdge::IRREGULAR_QUAD_PATCH: GregoryPatchT<Vertex,Vertex_t>(edge,loader).eval(u,v,P,dPdu,dPdv); break;
#endif
        default: {
          GeneralCatmullClarkPatch patch(edge,loader);
          eval_direct(patch,subPatch,Vec2f(u,v),P,dPdu,dPdv,0);
          break;
        }
        }
      }
    };
}
