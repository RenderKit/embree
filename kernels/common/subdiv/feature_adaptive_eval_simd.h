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
    template<typename vbool, typename vint, typename vfloat, typename Vertex, typename Vertex_t = Vertex>
      struct FeatureAdaptiveEvalSimd
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

        FeatureAdaptiveEvalSimd (const HalfEdge* edge, const char* vertices, size_t stride, const vbool& valid, const vfloat& u, const vfloat& v, float* P, float* dPdu, float* dPdv, const size_t dstride, const size_t N)
        : P(P), dPdu(dPdu), dPdv(dPdv), dstride(dstride), N(N)
        {
          switch (edge->patch_type) {
          case HalfEdge::REGULAR_QUAD_PATCH: RegularPatchT(edge,vertices,stride).eval(valid,u,v,P,dPdu,dPdv,1.0f,dstride,N); break;
#if PATCH_USE_GREGORY == 2
          case HalfEdge::IRREGULAR_QUAD_PATCH: GregoryPatchT<Vertex,Vertex_t>(edge,vertices,stride).eval(valid,u,v,P,dPdu,dPdv,1.0f,dstride,N); break;
#endif
          default: {
            GeneralCatmullClarkPatch patch(edge,vertices,stride);
            eval_direct(valid,patch,Vec2<vfloat>(u,v),0);
            break;
          }
          }
        }
        
        template<size_t N>
        __forceinline void eval_quad_direct(const vbool& valid, array_t<CatmullClarkPatch,N>& patches, const Vec2<vfloat>& uv, float dscale, size_t depth)
        {
          const vfloat u = uv.x, v = uv.y;
          const vbool u0_mask = u < 0.5f, u1_mask = u >= 0.5f;
          const vbool v0_mask = v < 0.5f, v1_mask = v >= 0.5f;
          const vbool u0v0_mask = valid & u0_mask & v0_mask;
          const vbool u0v1_mask = valid & u0_mask & v1_mask;
          const vbool u1v0_mask = valid & u1_mask & v0_mask;
          const vbool u1v1_mask = valid & u1_mask & v1_mask;
          if (any(u0v0_mask)) eval_direct(u0v0_mask,patches[0],Vec2<vfloat>(2.0f*u,2.0f*v),2.0f*dscale,depth+1);
          if (any(u1v0_mask)) eval_direct(u1v0_mask,patches[1],Vec2<vfloat>(2.0f*u-1.0f,2.0f*v),2.0f*dscale,depth+1);
          if (any(u1v1_mask)) eval_direct(u1v1_mask,patches[2],Vec2<vfloat>(2.0f*u-1.0f,2.0f*v-1.0f),2.0f*dscale,depth+1);
          if (any(u0v1_mask)) eval_direct(u0v1_mask,patches[3],Vec2<vfloat>(2.0f*u,2.0f*v-1.0f),2.0f*dscale,depth+1);
        }
        
        void eval_general_triangle_direct(const vbool& valid, array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2<vfloat>& uv, size_t depth)
        {
          const vbool ab_abc = right_of_line_ab_abc(uv);
          const vbool ac_abc = right_of_line_ac_abc(uv);
          const vbool bc_abc = right_of_line_bc_abc(uv);
          const vbool tri0_mask = valid & !ab_abc &  ac_abc;
          const vbool tri1_mask = valid &  ab_abc & !bc_abc & !tri0_mask;
          const vbool tri2_mask = valid & !tri0_mask & !tri1_mask;
          const vfloat u = uv.x, v = uv.y, w = 1.0f-u-v;
          
          if  (any(tri0_mask)) {
            const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(u,v));
            eval_direct(tri0_mask,patches[0],xy,1.0f,depth+1);
            if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad0_to_tri(tri0_mask,xy,dPdu,dPdv,dstride,i);
          }
          if (any(tri1_mask)) {
            const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(v,w));
            eval_direct(tri1_mask,patches[1],xy,1.0f,depth+1);
            if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad1_to_tri(tri1_mask,xy,dPdu,dPdv,dstride,i);
          }
          if (any(tri2_mask)) {
            const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(w,u));
            eval_direct(tri2_mask,patches[2],xy,1.0f,depth+1);
            if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad2_to_tri(tri2_mask,xy,dPdu,dPdv,dstride,i);
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

        void eval_direct(const vbool& valid, CatmullClarkPatch& patch, const Vec2<vfloat>& uv, float dscale, size_t depth)
        {
          typename CatmullClarkPatch::Type ty = patch.type();

          if (unlikely(final(patch,depth)))
          {
            if (ty & CatmullClarkRing::TYPE_REGULAR) { 
              RegularPatch(patch).eval(valid,uv.x,uv.y,P,dPdu,dPdv,dscale,dstride,N);
            } else {
              IrregularFillPatch(patch).eval(valid,uv.x,uv.y,P,dPdu,dPdv,dscale,dstride,N);
            }
          }
          else if (ty & CatmullClarkRing::TYPE_REGULAR_CREASES) { 
            assert(depth > 0); RegularPatch(patch).eval(valid,uv.x,uv.y,P,dPdu,dPdv,dscale,dstride,N);
          }
#if PATCH_USE_GREGORY == 2
          else if (ty & CatmullClarkRing::TYPE_GREGORY_CREASES) { 
            assert(depth > 0); GregoryPatch(patch).eval(valid,uv.x,uv.y,P,dPdu,dPdv,dscale,dstride,N);
          }
#endif
          else
          {
            array_t<CatmullClarkPatch,4> patches; 
            patch.subdivide(patches); // FIXME: only have to generate one of the patches
            eval_quad_direct(valid,patches,uv,dscale,depth);
          }
        }  

        void eval_direct(const vbool& valid, const GeneralCatmullClarkPatch& patch, const Vec2<vfloat>& uv, const size_t depth) 
        {
          /* convert into standard quad patch if possible */
          if (likely(patch.isQuadPatch())) {
            CatmullClarkPatch qpatch; patch.init(qpatch);
            return eval_direct(valid,qpatch,uv,1.0f,depth);
          }
          
          /* subdivide patch */
          size_t Nc;
          array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
          patch.subdivide(patches,Nc); // FIXME: only have to generate one of the patches
          
          /* parametrization for triangles */
          if (Nc == 3) // FIXME: border handling
            eval_general_triangle_direct(valid,patches,uv,depth);
          
          /* parametrization for quads */
          else if (Nc == 4) { // FIXME: border handling
            GeneralCatmullClarkPatch::fix_quad_ring_order(patches);
            eval_quad_direct(valid,patches,uv,1.0f,depth);
          }
          
          /* parametrization for arbitrary polygons */
          else { // FIXME: border handling
            const vint l = (vint)floor(4.0f*uv.x); const vfloat u = 2.0f*frac(4.0f*uv.x); 
            const vint h = (vint)floor(4.0f*uv.y); const vfloat v = 2.0f*frac(4.0f*uv.y); 
            const vint i = (h<<2)+l; assert(all(valid,i<Nc));
            foreach_unique(valid,i,[&](const vbool& valid, const int i) {
                eval_direct(valid,patches[i],Vec2<vfloat>(u,v),8.0f,depth+1);
              });
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
