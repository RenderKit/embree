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

#include "catmullclark_patch.h"
#include "bspline_patch.h"
#include "gregory_patch.h"
#include "gregory_triangle_patch.h"

#define PATCH_DEBUG_SUBDIVISION(x,y,z)                  \
/*  {                                                   \
    size_t hex = (size_t)this;                          \
    for (size_t i=0; i<4; i++) hex = hex ^ (hex >> 8);  \
    const float c = (float)(hex&0xff)/255.0f;           \
    if (P) *P = Vertex(c,0.0f,0.0f,0.0f);               \
    }               */

#define PATCH_MAX_CACHE_DEPTH 2
#define PATCH_MAX_EVAL_DEPTH 4  // has to be larger or equal than PATCH_MAX_CACHE_DEPTH
#define PATCH_USE_GREGORY 1     // 0 = no gregory, 1 = fill, 2 = as early as possible

namespace embree
{
  __forceinline Vec2f map_tri_to_quad(const Vec2f& uv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    const Vec2f A = a, B = ab-a, C = ac-a, D = a-ab-ac+abc;
    const float AA = det(D,C), BB = det(D,A) + det(B,C) + det(uv,D), CC = det(B,A) + det(uv,B);
    const float vv = (-BB+sqrtf(BB*BB-4.0f*AA*CC))/(2.0f*AA);
    const float uu = (uv.x - A.x - vv*C.x)/(B.x + vv*D.x);
    return Vec2f(uu,vv);
  }
  
  __forceinline Vec2f map_quad_to_tri_dx(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2f& xy) {
    return (1.0f-xy.y)*(ab-a) + xy.y*(abc-ac);
  }
  
  __forceinline Vec2f map_quad_to_tri_dy(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2f& xy) {
    return (1.0f-xy.x)*(ac-a) + xy.x*(abc-ab);
  }
  
  __forceinline bool right_of_line(const Vec2f& A, const Vec2f& B, const Vec2f& P) {
    return det(A-P,B-P) <= 0.0f;
  }
  
  __forceinline bool right_of_line_ab_abc(const Vec2f& uv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    return right_of_line(ab,abc,uv);
  }
  
  __forceinline bool right_of_line_ac_abc(const Vec2f& uv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    return right_of_line(ac,abc,uv);
  }
  
  __forceinline bool right_of_line_bc_abc(const Vec2f& uv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    return right_of_line(bc,abc,uv);
  }
  
  template<typename Vertex>
    __forceinline void map_quad0_to_tri(const Vec2f& xy, Vertex& dPdu, Vertex& dPdv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    const Vertex dpdx = dPdu, dpdy = dPdv;
    const Vec2f duvdx = map_quad_to_tri_dx(a,ab,abc,ac,xy);
    const Vec2f duvdy = map_quad_to_tri_dy(a,ab,abc,ac,xy);
    const LinearSpace2f J = rcp(LinearSpace2f(duvdx,duvdy));
    dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
    dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
  }
  
  template<typename Vertex>
    __forceinline void map_quad1_to_tri(const Vec2f& xy, Vertex& dPdu, Vertex& dPdv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    const Vertex dpdx = dPdu, dpdy = dPdv;
    const Vec2f duvdx = map_quad_to_tri_dx(b,bc,abc,ab,xy);
    const Vec2f duvdy = map_quad_to_tri_dy(b,bc,abc,ab,xy);
    const LinearSpace2f J = rcp(LinearSpace2f(duvdx,duvdy));
    dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
    dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
  }
  
  template<typename Vertex>
    __forceinline void map_quad2_to_tri(const Vec2f& xy, Vertex& dPdu, Vertex& dPdv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    const Vertex dpdx = dPdu, dpdy = dPdv;
    const Vec2f duvdx = map_quad_to_tri_dx(c,ac,abc,bc,xy);
    const Vec2f duvdy = map_quad_to_tri_dy(c,ac,abc,bc,xy);
    const LinearSpace2f J = rcp(LinearSpace2f(duvdx,duvdy));
    dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
    dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
  }
  
  template<typename Vertex, typename Vertex_t = Vertex>
    struct __aligned(64) Patch
    {
    public:
    
    typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
    
    enum Type {
      INVALID_PATCH = 0,
      BSPLINE_PATCH = 1,  
      GREGORY_PATCH = 2,
      EVAL_PATCH = 3,
      SUBDIVIDED_GENERAL_TRIANGLE_PATCH = 4,
      SUBDIVIDED_GENERAL_QUAD_PATCH = 5,
      SUBDIVIDED_QUAD_PATCH = 6
    };
    
    struct BSplinePatch 
    {
      /* creates BSplinePatch from a half edge */
      template<typename Loader, typename Allocator>
        __noinline static BSplinePatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const Loader& loader) {
        return new (alloc(sizeof(BSplinePatch))) BSplinePatch(edge,loader);
      }
      
      template<typename Loader>
      __forceinline BSplinePatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
      : type(BSPLINE_PATCH), patch(edge,loader) {}
      
      /* creates BSplinePatch from a CatmullClarkPatch */
      template<typename Allocator>
      __noinline static BSplinePatch* create(const Allocator& alloc, const CatmullClarkPatch& patch) {
        return new (alloc(sizeof(BSplinePatch))) BSplinePatch(patch);
      }
      
      __forceinline BSplinePatch (const CatmullClarkPatch& patch) 
        : type(BSPLINE_PATCH), patch(patch) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale) const
      {
        patch.eval(u,v,P,dPdu,dPdv,dscale);
        PATCH_DEBUG_SUBDIVISION(c,0,0);
        return true;
      }
      
    public:
      Type type;
      BSplinePatchT<Vertex,Vertex_t> patch;
    };
    
    struct GregoryPatch
    {
      /* creates GregoryPatch from half edge */
      template<typename Loader, typename Allocator>
        __noinline static GregoryPatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const Loader& loader) {
        return new (alloc(sizeof(GregoryPatch))) GregoryPatch(edge,loader);
      }
      
      template<typename Loader>
      __forceinline GregoryPatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
      : type(GREGORY_PATCH) { CatmullClarkPatch ccpatch; ccpatch.init2(edge,loader); patch.init(ccpatch); }
      
      /* creates GregoryPatch from CatmullClarkPatch */
      template<typename Allocator>
      __noinline static GregoryPatch* create(const Allocator& alloc, const CatmullClarkPatch& patch) {
        return new (alloc(sizeof(GregoryPatch))) GregoryPatch(patch);
      }
      
      __forceinline GregoryPatch (const CatmullClarkPatch& patch) 
        : type(GREGORY_PATCH), patch(patch) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale) const
      {
        patch.eval(u,v,P,dPdu,dPdv,dscale);
        PATCH_DEBUG_SUBDIVISION(0,c,0);
        return true;
      }
      
    public:
      Type type;
      GregoryPatchT<Vertex,Vertex_t> patch;
    };
    
    struct EvalPatch
    {
      /* creates the EvalPatch from a half edge */
      template<typename Allocator>
      __noinline static EvalPatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride) {
        return new (alloc(sizeof(EvalPatch))) EvalPatch(edge,vertices,stride);
      }
      
      __forceinline EvalPatch (const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride)
        : type(EVAL_PATCH), edge(edge), vertices(vertices), stride(stride), child(nullptr) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
      {
        /* first try to fast path using cached subdivision tree */
        if (child && child->eval(u,v,P,dPdu,dPdv,1.0f)) 
          return true;
        
        /* if this fails as the cache does not store the required sub-patch, fallback into full evaluation */
        Patch::eval_direct(edge,vertices,stride,u,v,P,dPdu,dPdv);
        PATCH_DEBUG_SUBDIVISION(0,0,c);
        return true;
      }
      
    public:
      Type type;
      const SubdivMesh::HalfEdge* const edge;
      const char* const vertices;
      const size_t stride;
      Patch* child;
    };
    
    struct SubdividedGeneralTrianglePatch
    {
      template<typename Allocator>
      __noinline static SubdividedGeneralTrianglePatch* create(const Allocator& alloc) {
        return new (alloc(sizeof(SubdividedGeneralTrianglePatch))) SubdividedGeneralTrianglePatch;
      }
      
      __forceinline SubdividedGeneralTrianglePatch() 
        : type(SUBDIVIDED_GENERAL_TRIANGLE_PATCH) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        const bool ab_abc = right_of_line_ab_abc(Vec2f(u,v));
        const bool ac_abc = right_of_line_ac_abc(Vec2f(u,v));
        const bool bc_abc = right_of_line_bc_abc(Vec2f(u,v));
        
        const float w = 1.0f-u-v;
        if  (!ab_abc &&  ac_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(u,v));
          if (!child[0]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
          if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
        }
        else if ( ab_abc && !bc_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(v,w));
          if (!child[1]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
          if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
        }
        else {
          const Vec2f xy = map_tri_to_quad(Vec2f(w,u));
          if (!child[2]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
          if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
        }
        return true;
      }

      static void eval_direct(array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2f& uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, size_t depth)
      {
        const bool ab_abc = right_of_line_ab_abc(uv);
        const bool ac_abc = right_of_line_ac_abc(uv);
        const bool bc_abc = right_of_line_bc_abc(uv);
        
        const float u = uv.x, v = uv.y, w = 1.0f-u-v;
        if  (!ab_abc &&  ac_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(u,v));
          Patch::eval_direct(patches[0],xy,P,dPdu,dPdv,1.0f,depth+1);
          if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
        }
        else if ( ab_abc && !bc_abc) {
          const Vec2f xy = map_tri_to_quad(Vec2f(v,w));
          Patch::eval_direct(patches[1],xy,P,dPdu,dPdv,1.0f,depth+1);
          if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
        }
        else {
          const Vec2f xy = map_tri_to_quad(Vec2f(w,u));
          Patch::eval_direct(patches[2],xy,P,dPdu,dPdv,1.0f,depth+1);
          if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
        }
      }
      
      Type type;
      Patch* child[3];
    };
    
    struct SubdividedQuadPatch
    {
      template<typename Allocator>
      __noinline static SubdividedQuadPatch* create(const Allocator& alloc) {
        return new (alloc(sizeof(SubdividedQuadPatch))) SubdividedQuadPatch;
      }
      
      __forceinline SubdividedQuadPatch() : type(SUBDIVIDED_QUAD_PATCH) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale)
      {
        if (v < 0.5f) {
          if (u < 0.5f) return child[0]->eval(2.0f*u,2.0f*v,P,dPdu,dPdv,2.0f*dscale);
          else          return child[1]->eval(2.0f*u-1.0f,2.0f*v,P,dPdu,dPdv,2.0f*dscale);
        } else {
          if (u > 0.5f) return child[2]->eval(2.0f*u-1.0f,2.0f*v-1.0f,P,dPdu,dPdv,2.0f*dscale);
          else          return child[3]->eval(2.0f*u,2.0f*v-1.0f,P,dPdu,dPdv,2.0f*dscale);
        }
      }
      
    public:
      Type type;
      Patch* child[4];
    };
    
    struct SubdividedGeneralQuadPatch
    {
      template<typename Allocator>
      __noinline static SubdividedGeneralQuadPatch* create(const Allocator& alloc) {
        return new (alloc(sizeof(SubdividedGeneralQuadPatch))) SubdividedGeneralQuadPatch;
      }
      
      __forceinline SubdividedGeneralQuadPatch() : type(SUBDIVIDED_GENERAL_QUAD_PATCH) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      {
        if (v < 0.5f) {
          if (u < 0.5f) {
            if (!child[0]->eval(2.0f*u,2.0f*v,P,dPdu,dPdv,2.0f)) return false;
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = dpdx; *dPdv = dpdy;
            }
            return true;
          }
          else {
            if (!child[1]->eval(2.0f*v,2.0f-2.0f*u,P,dPdu,dPdv,2.0f)) return false;
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -dpdy; *dPdv = dpdx;
            }
            return true;
          }
        } else {
          if (u > 0.5f) {
            if (!child[2]->eval(2.0f-2.0f*u,2.0f-2.0f*v,P,dPdu,dPdv,2.0f)) return false;
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -dpdx; *dPdv = -dpdy;
            }
            return true;
          }
          else {
            if (!child[3]->eval(2.0f-2.0f*v,2.0f*u,P,dPdu,dPdv,2.0f)) return false;
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = dpdy; *dPdv = -dpdx;
            }
            return true;
          }
        }
      }

      static void eval_direct(array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE>& patches, const Vec2f& uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, size_t depth)
      {
        float u = uv.x, v = uv.y;
        if (uv.y < 0.5f) {
          if (uv.x < 0.5f) {
            Patch::eval_direct(patches[0],Vec2f(2.0f*u,2.0f*v),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = dpdx; *dPdv = dpdy;
            }
          }
          else {
            Patch::eval_direct(patches[1],Vec2f(2.0f*v,2.0f-2.0f*u),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -dpdy; *dPdv = dpdx;
            }
          }
        } else {
          if (uv.x > 0.5f) {
            Patch::eval_direct(patches[2],Vec2f(2.0f-2.0f*u,2.0f-2.0f*v),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -dpdx; *dPdv = -dpdy;
            }
          }
          else {
            Patch::eval_direct(patches[3],Vec2f(2.0f-2.0f*v,2.0f*u),P,dPdu,dPdv,2.0f,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = dpdy; *dPdv = -dpdx;
            }
          }
        }
      }
      
      Type type;
      Patch* child[4];
    };
    
    /*! Default constructor. */
    __forceinline Patch () {}
    
    template<typename Allocator>
    __noinline static Patch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride)
    {
      auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
        const unsigned vtx = p->getStartVertexIndex();
        return Vertex_t::loadu((float*)&vertices[vtx*stride]);
      };
      
      EvalPatch* root = EvalPatch::create(alloc,edge,vertices,stride);
      Patch* child = nullptr;
      
      if (PATCH_MAX_CACHE_DEPTH == 0) 
        return (Patch*) root;
      
      switch (edge->type) {
      case SubdivMesh::REGULAR_QUAD_PATCH:   child = (Patch*) BSplinePatch::create(alloc,edge,loader); break;
#if PATCH_USE_GREGORY == 2
      case SubdivMesh::IRREGULAR_QUAD_PATCH: child = (Patch*) GregoryPatch::create(alloc,edge,loader); break;
#endif
      default: {
        GeneralCatmullClarkPatch patch;
        patch.init2(edge,loader);
        child = (Patch*) Patch::create(alloc,patch,edge,vertices,stride,0);
        break;
      }
      }
      
      root->child = child;
      return (Patch*) root;
    }

    static void eval_direct (const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
    {
      auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
          const unsigned vtx = p->getStartVertexIndex();
          return Vertex_t::loadu((float*)&vertices[vtx*stride]); 
        };

      switch (edge->type) {
      case SubdivMesh::REGULAR_QUAD_PATCH: {
        BSplinePatchT<Vertex,Vertex_t>(edge,loader).eval(u,v,P,dPdu,dPdv);
        break;
      }
#if PATCH_USE_GREGORY == 2
      case SubdivMesh::IRREGULAR_QUAD_PATCH: {
        CatmullClarkPatch ccpatch; ccpatch.init2(edge,loader); 
        GregoryPatchT<Vertex,Vertex_t> patch; patch.init(ccpatch);
        patch.eval(u,v,P,dPdu,dPdv);
        break;
      }
#endif
      default: {
        GeneralCatmullClarkPatch patch;
        patch.init2(edge,loader);
        eval_direct(patch,Vec2f(u,v),P,dPdu,dPdv,size_t(0));
        break;
      }
      }
    }
    
    template<typename Allocator>
    __noinline static Patch* create(const Allocator& alloc, GeneralCatmullClarkPatch& patch, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, size_t depth)
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch())) {
        CatmullClarkPatch qpatch; patch.init(qpatch);
        return Patch::create(alloc,qpatch,edge,vertices,stride,depth);
      }
      
      if (depth >= PATCH_MAX_CACHE_DEPTH)
        return nullptr;
      
      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
      patch.subdivide(patches,N);
      
      if (N == 3) {
        SubdividedGeneralTrianglePatch* node = SubdividedGeneralTrianglePatch::create(alloc);
        for (size_t i=0; i<3; i++)
          node->child[i] = Patch::create(alloc,patches[i],edge,vertices,stride,depth+1);
        return (Patch*) node;
      } 
      else if (N == 4) {
        SubdividedGeneralQuadPatch* node = SubdividedGeneralQuadPatch::create(alloc);
        for (size_t i=0; i<4; i++)
          node->child[i] = Patch::create(alloc,patches[i],edge,vertices,stride,depth+1);
        return (Patch*) node;
      }
      else 
        assert(false);
      
      return nullptr;
    }

    static void eval_direct(const GeneralCatmullClarkPatch& patch, const Vec2f& uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, const size_t depth) 
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch())) 
      {
        CatmullClarkPatch qpatch; patch.init(qpatch);
        eval_direct(qpatch,uv,P,dPdu,dPdv,1.0f,depth); 
        return;
      }
      
      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
      patch.subdivide(patches,N); // FIXME: only have to generate one of the patches
      
      /* parametrization for triangles */
      if (N == 3) 
        SubdividedGeneralTrianglePatch::eval_direct(patches,uv,P,dPdu,dPdv,depth);
      
      /* parametrization for quads */
      else if (N == 4) 
        SubdividedGeneralQuadPatch::eval_direct(patches,uv,P,dPdu,dPdv,depth);
      
      /* parametrization for arbitrary polygons */
      else 
      {
        const unsigned i = floorf(uv.x); assert(i<N);
        eval_direct(patches[i],Vec2f(floorf(uv.x),uv.y),P,dPdu,dPdv,1.0f,depth+1); // FIXME: uv encoding creates issues as uv=(1,0) will refer to second quad
      }
    }
    
    template<typename Allocator>
    __noinline static Patch* create(const Allocator& alloc, CatmullClarkPatch& patch, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, size_t depth)
    {
      if (patch.isRegular()) { assert(depth > 0); return (Patch*) BSplinePatch::create(alloc,patch); }
#if PATCH_USE_GREGORY == 2
      else if (patch.isGregory()) { assert(depth > 0); return (Patch*) GregoryPatch::create(alloc,patch); }
#endif
      else if (depth >= PATCH_MAX_CACHE_DEPTH) return nullptr;
      else {
        SubdividedQuadPatch* node = SubdividedQuadPatch::create(alloc);
        array_t<CatmullClarkPatch,4> patches; 
        patch.subdivide(patches);
        for (size_t i=0; i<4; i++)
          node->child[i] = Patch::create(alloc,patches[i],edge,vertices,stride,depth+1);
        return (Patch*) node;
      }
    }
    
    bool eval(const float& u, const float& v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale) const
    {
      if (this == nullptr) return false;
      
      switch (type) {
      case BSPLINE_PATCH: return ((BSplinePatch*)this)->eval(u,v,P,dPdu,dPdv,dscale); 
      case GREGORY_PATCH: return ((GregoryPatch*)this)->eval(u,v,P,dPdu,dPdv,dscale); 
      case SUBDIVIDED_QUAD_PATCH: return ((SubdividedQuadPatch*)this)->eval(u,v,P,dPdu,dPdv,dscale);
      case SUBDIVIDED_GENERAL_QUAD_PATCH:     { assert(dscale == 1.0f); return ((SubdividedGeneralQuadPatch*)this)->eval(u,v,P,dPdu,dPdv); }
      case SUBDIVIDED_GENERAL_TRIANGLE_PATCH: { assert(dscale == 1.0f); return ((SubdividedGeneralTrianglePatch*)this)->eval(u,v,P,dPdu,dPdv); }
      default: assert(false); return false;
      }
    }
    
    bool eval(const float& u, const float& v, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
    {
      assert(type == EVAL_PATCH);
      return ((EvalPatch*)this)->eval(u,v,P,dPdu,dPdv); 
    }

    
    
    static void eval_direct(CatmullClarkPatch& patch, Vec2f uv, Vertex* P, Vertex* dPdu, Vertex* dPdv, float dscale, size_t depth)
    {
      while (true) 
      {
        if (unlikely(patch.isRegular()))
        {
          BSplinePatch bspline(patch);
          bspline.eval(uv.x,uv.y,P,dPdu,dPdv,dscale);
          return;
        }
#if PATCH_USE_GREGORY == 2
        else if (unlikely(depth>=PATCH_MAX_EVAL_DEPTH || patch.isGregory()))
        {
          GregoryPatch gregory(patch);
          gregory.eval(uv.x,uv.y,P,dPdu,dPdv,dscale);
          return;
        }
#else
        else if (unlikely(depth>=PATCH_MAX_EVAL_DEPTH))
        {
#if PATCH_USE_GREGORY == 1
          GregoryPatch gregory(patch);
          gregory.eval(uv.x,uv.y,P,dPdu,dPdv,dscale);
#else
          const float sx1 = uv.x, sx0 = 1.0f-sx1;
          const float sy1 = uv.y, sy0 = 1.0f-sy1;
          const Vertex P0 = patch.ring[0].getLimitVertex();
          const Vertex P1 = patch.ring[1].getLimitVertex();
          const Vertex P2 = patch.ring[2].getLimitVertex();
          const Vertex P3 = patch.ring[3].getLimitVertex();
          if (P   ) *P    = sy0*(sx0*P0+sx1*P1) + sy1*(sx0*P3+sx1*P2);
          if (dPdu) *dPdu = (sy0*(P1-P0) + sy1*(P2-P3))*dscale; 
          if (dPdv) *dPdv = (sx0*(P3-P0) + sx1*(P2-P1))*dscale;
#endif
          return;
        }
#endif
        else
        {
          array_t<CatmullClarkPatch,4> patches; 
          patch.subdivide(patches); // FIXME: only have to generate one of the patches
          
          const float u = uv.x, v = uv.y;
          if (uv.y < 0.5f) {
            if (uv.x < 0.5f) { patch = patches[0]; uv = Vec2f(2.0f*u,2.0f*v); }
            else             { patch = patches[1]; uv = Vec2f(2.0f*u-1.0f,2.0f*v); }
          } else {
            if (uv.x > 0.5f) { patch = patches[2]; uv = Vec2f(2.0f*u-1.0f,2.0f*v-1.0f); }
            else             { patch = patches[3]; uv = Vec2f(2.0f*u,2.0f*v-1.0f); }
          }
          dscale *= 2.0f;
          depth++;
        }
      }
    }  
    
    public:
    Type type;
  };
}
