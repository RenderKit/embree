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
  template<typename vbool, typename vint, typename vfloat, typename Vertex, typename Vertex_t = Vertex>
    struct PatchEvalSimd
    {
    public:
      
      typedef PatchT<Vertex,Vertex_t> Patch;
      typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
      typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
      typedef BSplinePatchT<Vertex,Vertex_t> BSplinePatch;
      typedef BezierPatchT<Vertex,Vertex_t> BezierPatch;
      typedef GregoryPatchT<Vertex,Vertex_t> GregoryPatch;

      static vbool eval_general_triangle(const vbool& valid, const typename Patch::SubdividedGeneralTrianglePatch* This, const vfloat& u, const vfloat& v, vfloat* P, vfloat* dPdu, vfloat* dPdv, const size_t N)
      {
        vbool ret = false;
        const vbool ab_abc = right_of_line_ab_abc(Vec2<vfloat>(u,v));
        const vbool ac_abc = right_of_line_ac_abc(Vec2<vfloat>(u,v));
        const vbool bc_abc = right_of_line_bc_abc(Vec2<vfloat>(u,v));
        const vbool tri0_mask = valid & !ab_abc &  ac_abc;
        const vbool tri1_mask = valid &  ab_abc & !bc_abc & !tri0_mask;
        const vbool tri2_mask = valid & !tri1_mask;
        const vfloat w = 1.0f-u-v;

        if  (any(tri0_mask)) {
          const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(u,v));
          ret |= eval(tri0_mask,This->child[0],xy.x,xy.y,P,dPdu,dPdv,1.0f,N);
          if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad0_to_tri(xy,dPdu[i],dPdv[i]);
        }
        if (any(tri1_mask)) {
          const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(v,w));
          ret |= eval(tri1_mask,This->child[1],xy.x,xy.y,P,dPdu,dPdv,1.0f,N);
          if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad1_to_tri(xy,dPdu[i],dPdv[i]);
        }
        if (any(tri2_mask)) {
          const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(w,u));
          ret |= eval(tri2_mask,This->child[2],xy.x,xy.y,P,dPdu,dPdv,1.0f,N);
          if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad2_to_tri(xy,dPdu[i],dPdv[i]);
        }
        return ret;
      }
 
      static void eval_general_triangle_direct(const vbool& valid, array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2<vfloat>& uv, vfloat* P, vfloat* dPdu, vfloat* dPdv, size_t depth, const size_t N)
      {
        const vbool ab_abc = right_of_line_ab_abc(uv);
        const vbool ac_abc = right_of_line_ac_abc(uv);
        const vbool bc_abc = right_of_line_bc_abc(uv);
        const vbool tri0_mask = valid & !ab_abc &  ac_abc;
        const vbool tri1_mask = valid &  ab_abc & !bc_abc & !tri0_mask;
        const vbool tri2_mask = valid & !tri1_mask;
        const vfloat u = uv.x, v = uv.y, w = 1.0f-u-v;

        if  (any(tri0_mask)) {
          const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(u,v));
          eval_direct(tri0_mask,patches[0],xy,P,dPdu,dPdv,1.0f,depth+1,N);
          if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad0_to_tri(xy,dPdu[i],dPdv[i]);
        }
        if (any(tri1_mask)) {
          const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(v,w));
          eval_direct(tri1_mask,patches[1],xy,P,dPdu,dPdv,1.0f,depth+1,N);
          if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad1_to_tri(xy,dPdu[i],dPdv[i]);
        }
        if (any(tri2_mask)) {
          const Vec2<vfloat> xy = map_tri_to_quad(Vec2<vfloat>(w,u));
          eval_direct(tri2_mask,patches[2],xy,P,dPdu,dPdv,1.0f,depth+1,N);
          if (dPdu && dPdv) for (size_t i=0; i<N; i++) map_quad2_to_tri(xy,dPdu[i],dPdv[i]);
        }
      }
 
      static vbool eval_quad(const vbool& valid, const typename Patch::SubdividedQuadPatch* This, const vfloat u, const vfloat v, vfloat* P, vfloat* dPdu, vfloat* dPdv, const float dscale, const size_t N)
      {
        vbool ret = false;
        const vbool u0_mask = u < 0.5f, u1_mask = u >= 0.5f;
        const vbool v0_mask = v < 0.5f, v1_mask = v >= 0.5f;
        const vbool u0v0_mask = valid & u0_mask & v0_mask;
        const vbool u0v1_mask = valid & u0_mask & v1_mask;
        const vbool u1v0_mask = valid & u1_mask & v0_mask;
        const vbool u1v1_mask = valid & u0_mask & v0_mask;
        if (any(u0v0_mask)) ret |= eval(u0v0_mask,This->child[0],2.0f*u,2.0f*v,P,dPdu,dPdv,2.0f*dscale,N);
        if (any(u1v0_mask)) ret |= eval(u1v0_mask,This->child[1],2.0f*u-1.0f,2.0f*v,P,dPdu,dPdv,2.0f*dscale,N);
        if (any(u1v1_mask)) ret |= eval(u1v1_mask,This->child[2],2.0f*u-1.0f,2.0f*v-1.0f,P,dPdu,dPdv,2.0f*dscale,N);
        if (any(u0v1_mask)) ret |= eval(u0v1_mask,This->child[3],2.0f*u,2.0f*v-1.0f,P,dPdu,dPdv,2.0f*dscale,N);
        return ret;
      }
     
      __forceinline static void eval_quad_direct(const vbool& valid, CatmullClarkPatch& patch, const Vec2<vfloat>& uv, vfloat* P, vfloat* dPdu, vfloat* dPdv, float dscale, size_t depth, const size_t N)
      {
        array_t<CatmullClarkPatch,4> patches; 
        patch.subdivide(patches); // FIXME: only have to generate one of the patches
      
        const vfloat u = uv.x, v = uv.y;
        const vbool u0_mask = u < 0.5f, u1_mask = u >= 0.5f;
        const vbool v0_mask = v < 0.5f, v1_mask = v >= 0.5f;
        const vbool u0v0_mask = valid & u0_mask & v0_mask;
        const vbool u0v1_mask = valid & u0_mask & v1_mask;
        const vbool u1v0_mask = valid & u1_mask & v0_mask;
        const vbool u1v1_mask = valid & u0_mask & v0_mask;
        if (any(u0v0_mask)) eval_direct(u0v0_mask,patches[0],Vec2<vfloat>(2.0f*u,2.0f*v),P,dPdu,dPdv,2.0f*dscale,depth+1,N);
        if (any(u1v0_mask)) eval_direct(u1v0_mask,patches[1],Vec2<vfloat>(2.0f*u-1.0f,2.0f*v),P,dPdu,dPdv,2.0f*dscale,depth+1,N);
        if (any(u1v1_mask)) eval_direct(u1v1_mask,patches[2],Vec2<vfloat>(2.0f*u-1.0f,2.0f*v-1.0f),P,dPdu,dPdv,2.0f*dscale,depth+1,N);
        if (any(u0v1_mask)) eval_direct(u0v1_mask,patches[3],Vec2<vfloat>(2.0f*u,2.0f*v-1.0f),P,dPdu,dPdv,2.0f*dscale,depth+1,N);
      }

      static vbool eval_general_quad(const vbool& valid, const typename Patch::SubdividedGeneralQuadPatch* This, const vfloat& u, const vfloat& v, vfloat* P, vfloat* dPdu, vfloat* dPdv, const size_t N)
      {
        vbool ret = false;
        const vbool u0_mask = u < 0.5f, u1_mask = u >= 0.5f;
        const vbool v0_mask = v < 0.5f, v1_mask = v >= 0.5f;
        const vbool u0v0_mask = valid & u0_mask & v0_mask;
        const vbool u0v1_mask = valid & u0_mask & v1_mask;
        const vbool u1v0_mask = valid & u1_mask & v0_mask;
        const vbool u1v1_mask = valid & u0_mask & v0_mask;
        if (any(u0v0_mask)) {
          ret |= eval(u0v0_mask,This->child[0],2.0f*u,2.0f*v,P,dPdu,dPdv,2.0f,N);
          if (dPdu && dPdv) {
            for (size_t i=0; i<N; i++) {
              const vfloat dpdx = dPdu[i], dpdy = dPdv[i];  dPdu[i] = dpdx; dPdv[i] = dpdy;
            }
          }
        }
        if (any(u1v0_mask)) {
          ret |= eval(u1v0_mask,This->child[1],2.0f*v,2.0f-2.0f*u,P,dPdu,dPdv,2.0f,N);
          for (size_t i=0; i<N; i++) {
            const vfloat dpdx = dPdu[i], dpdy = dPdv[i]; dPdu[i] = -dpdy; dPdv[i] = dpdx;
          }
        }
        if (any(u1v1_mask)) {
          ret |= eval(u1v1_mask,This->child[2],2.0f-2.0f*u,2.0f-2.0f*v,P,dPdu,dPdv,2.0f,N);
          for (size_t i=0; i<N; i++) {
            const Vertex dpdx = dPdu[i], dpdy = dPdv[i]; dPdu[i] = -dpdx; dPdv[i] = -dpdy;
          }
        }
        if (any(u0v1_mask)) {
          ret |= eval(u0v1_mask,This->child[3],2.0f-2.0f*v,2.0f*u,P,dPdu,dPdv,2.0f,N);
          for (size_t i=0; i<N; i++) {
            const Vertex dpdx = dPdu[i], dpdy = dPdv[i]; dPdu[i] = dpdy; dPdv[i] = -dpdx;
          }
        }
        return ret;
      }

      static void eval_general_quad_direct(const vbool& valid, array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2<vfloat>& uv, vfloat* P, vfloat* dPdu, vfloat* dPdv, size_t depth, size_t N)
      {
        const vfloat u = uv.x, v = uv.y;
        const vbool u0_mask = u < 0.5f, u1_mask = u >= 0.5f;
        const vbool v0_mask = v < 0.5f, v1_mask = v >= 0.5f;
        const vbool u0v0_mask = valid & u0_mask & v0_mask;
        const vbool u0v1_mask = valid & u0_mask & v1_mask;
        const vbool u1v0_mask = valid & u1_mask & v0_mask;
        const vbool u1v1_mask = valid & u0_mask & v0_mask;
        if (any(u0v0_mask)) {
          eval_direct(u0v0_mask,patches[0],Vec2<vfloat>(2.0f*u,2.0f*v),P,dPdu,dPdv,2.0f,depth+1,N);
          if (dPdu && dPdv) {
            for (size_t i=0; i<N; i++) {
              const vfloat dpdx = dPdu[i], dpdy = dPdv[i];  dPdu[i] = dpdx; dPdv[i] = dpdy;
            }
          }
        }
        if (any(u1v0_mask)) {
          eval_direct(u1v0_mask,patches[1],Vec2<vfloat>(2.0f*v,2.0f-2.0f*u),P,dPdu,dPdv,2.0f,depth+1,N);
          for (size_t i=0; i<N; i++) {
            const vfloat dpdx = dPdu[i], dpdy = dPdv[i]; dPdu[i] = -dpdy; dPdv[i] = dpdx;
          }
        }
        if (any(u1v1_mask)) {
          eval_direct(u1v1_mask,patches[2],Vec2<vfloat>(2.0f-2.0f*u,2.0f-2.0f*v),P,dPdu,dPdv,2.0f,depth+1,N);
          for (size_t i=0; i<N; i++) {
            const Vertex dpdx = dPdu[i], dpdy = dPdv[i]; dPdu[i] = -dpdx; dPdv[i] = -dpdy;
          }
        }
        if (any(u0v1_mask)) {
          eval_direct(u0v1_mask,patches[3],Vec2<vfloat>(2.0f-2.0f*v,2.0f*u),P,dPdu,dPdv,2.0f,depth+1,N);
          for (size_t i=0; i<N; i++) {
            const Vertex dpdx = dPdu[i], dpdy = dPdv[i]; dPdu[i] = dpdy; dPdv[i] = -dpdx;
          }
        }
      }

      static void eval_direct(const vbool& valid, const GeneralCatmullClarkPatch& patch, const Vec2<vfloat>& uv, vfloat* P, vfloat* dPdu, vfloat* dPdv, const size_t depth, const size_t N) 
      {
        /* convert into standard quad patch if possible */
        if (likely(patch.isQuadPatch())) 
        {
          CatmullClarkPatch qpatch; patch.init(qpatch);
          return eval_direct(valid,qpatch,uv,P,dPdu,dPdv,1.0f,depth,N);
        }
        
        /* subdivide patch */
        size_t Nc;
        array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
        patch.subdivide(patches,Nc); // FIXME: only have to generate one of the patches
        
        /* parametrization for triangles */
        if (Nc == 3) 
          eval_general_triangle_direct(valid,patches,uv,P,dPdu,dPdv,depth,N);
         
        /* parametrization for quads */
        else if (Nc == 4) 
          eval_general_quad_direct(valid,patches,uv,P,dPdu,dPdv,depth,N);
        
        /* parametrization for arbitrary polygons */
        else {
          const vint i = (vint)floor(uv.x); assert(all(valid,i<Nc));
          foreach_unique(valid,i,[&](const vbool& valid, const int i) {
              eval_direct(valid,patches[i],Vec2<vfloat>(frac(uv.x),uv.y),P,dPdu,dPdv,1.0f,depth+1,N); // FIXME: uv encoding creates issues as uv=(1,0) will refer to second quad
            });
        }
      }

      static void eval_direct(const vbool& valid, CatmullClarkPatch& patch, const Vec2<vfloat>& uv, vfloat* P, vfloat* dPdu, vfloat* dPdv, float dscale, size_t depth, const size_t N)
      {
        if (unlikely(patch.isRegular2())) { 
          assert(depth > 0); RegularPatch(patch).eval(N,uv.x,uv.y,P,dPdu,dPdv,dscale); return;
        }
#if PATCH_USE_GREGORY == 2
        else if (unlikely(depth>=PATCH_MAX_EVAL_DEPTH || patch.isGregory())) {
          assert(depth > 0); GregoryPatch(patch).eval(N,uv.x,uv.y,P,dPdu,dPdv,dscale); return;
        }
#else
        else if (unlikely(depth>=PATCH_MAX_EVAL_DEPTH))
        {
#if PATCH_USE_GREGORY == 1
          GregoryPatch(patch).eval(N,uv.x,uv.y,P,dPdu,dPdv,dscale);
#else
          BilinearPatch(patch).eval(N,uv.x,uv.y,P,dPdu,dPdv,dscale);
#endif
          return;
        }
#endif
        else {
          eval_quad_direct(valid,patch,uv,P,dPdu,dPdv,dscale,depth+1,N);
        }
      }  
 
      static vbool eval(const vbool& valid, Patch* This, const vfloat& u, const vfloat& v, vfloat* P, vfloat* dPdu, vfloat* dPdv, const float dscale, size_t N) 
      {
        if (This == nullptr) return false;
        
        switch (This->type) 
        {
        case Patch::BILINEAR_PATCH: {
          ((typename Patch::BilinearPatch*)This)->patch.eval(N,u,v,P,dPdu,dPdv,dscale); 
          PATCH_DEBUG_SUBDIVISION(c,c,-1);
          return true;
        }
        case Patch::BSPLINE_PATCH: {
          ((typename Patch::BSplinePatch*)This)->patch.eval(N,u,v,P,dPdu,dPdv,dscale);
          PATCH_DEBUG_SUBDIVISION(-1,c,-1);
          return true;
        }
        case Patch::BEZIER_PATCH: {
          ((typename Patch::BezierPatch*)This)->patch.eval(N,u,v,P,dPdu,dPdv,dscale);
          PATCH_DEBUG_SUBDIVISION(-1,c,-1);
          return true;
        }
        case Patch::GREGORY_PATCH: {
          ((typename Patch::GregoryPatch*)This)->patch.eval(N,u,v,P,dPdu,dPdv,dscale); 
          PATCH_DEBUG_SUBDIVISION(-1,-1,c);
          return true;
        }
        case Patch::SUBDIVIDED_QUAD_PATCH: 
          return eval_quad(valid,(typename Patch::SubdividedQuadPatch*)This,u,v,P,dPdu,dPdv,dscale,N);
        case Patch::SUBDIVIDED_GENERAL_QUAD_PATCH: { 
          assert(dscale == 1.0f); 
          return eval_general_quad(valid,(typename Patch::SubdividedGeneralQuadPatch*)This,u,v,P,dPdu,dPdv,N); 
        }
        case Patch::SUBDIVIDED_GENERAL_TRIANGLE_PATCH: { 
          assert(dscale == 1.0f); 
          return eval_general_triangle(valid,(typename Patch::SubdividedGeneralTrianglePatch*)This,u,v,P,dPdu,dPdv,N); 
        }
        default: 
          assert(false); 
          return false;
        }
      }
      
      static void eval_direct (const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, const vbool& valid, const vfloat& u, const vfloat& v, vfloat* P, vfloat* dPdu, vfloat* dPdv, const size_t N)
      {
        auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
          const unsigned vtx = p->getStartVertexIndex();
          return Vertex_t::loadu((float*)&vertices[vtx*stride]); 
        };
        
        switch (edge->patch_type) {
        case SubdivMesh::REGULAR_QUAD_PATCH: RegularPatchT(edge,loader).eval(N,u,v,P,dPdu,dPdv); break;
#if PATCH_USE_GREGORY == 2
        case SubdivMesh::IRREGULAR_QUAD_PATCH: GregoryPatchT<Vertex,Vertex_t>(edge,loader).eval(N,u,v,P,dPdu,dPdv); break;
#endif
        default: {
          GeneralCatmullClarkPatch patch(edge,loader);
          eval_direct(valid,patch,Vec2<vfloat>(u,v),P,dPdu,dPdv,0,N);
          break;
        }
        }
      }

      static void eval (SharedLazyTessellationCache::CacheEntry& entry, size_t commitCounter, 
                        const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, const vbool& valid0, const vfloat& u, const vfloat& v, vfloat* P, vfloat* dPdu, vfloat* dPdv, size_t N)
      {
        Patch* patch = SharedLazyTessellationCache::lookup(entry,commitCounter,[&] () {
            auto alloc = [](size_t bytes) { return SharedLazyTessellationCache::malloc(bytes); };
            return Patch::create(alloc,edge,vertices,stride);
          });
        
        /* use cached data structure for calculations */
        const vbool valid1 = patch ? eval(valid0,patch,u,v,P,dPdu,dPdv,1.0f,N) : vbool(false);
        SharedLazyTessellationCache::unlock();
        const vbool valid2 = valid0 & !valid1;
        if (any(valid2)) {
          eval_direct (edge,vertices,stride,valid2,u,v,P,dPdu,dPdv,N);
          PATCH_DEBUG_SUBDIVISION(c,-1,-1);
        }
      }
      
    };
}
