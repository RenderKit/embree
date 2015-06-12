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
//#include "feature_adaptive_eval.h"

#define PATCH_DEBUG_SUBDIVISION 0

namespace embree
{
    __forceinline Vec2f map_tri_to_quad(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2f& uv)
  {
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
    struct FeatureAdaptivePointEval
  {
    typedef BSplinePatchT<Vertex,Vertex_t> BSplinePatch;
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
    typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;

    const float u,v;
    Vertex* P;
    Vertex* dPdu;
    Vertex* dPdv;
    
    __forceinline FeatureAdaptivePointEval (const GeneralCatmullClarkPatch& patch, const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
      : u(u), v(v), P(P), dPdu(dPdu), dPdv(dPdv)
    {
      const float vv = clamp(v,0.0f,1.0f); // FIXME: remove clamps, add assertions
      const float uu = clamp(u,0.0f,1.0f); 
      eval(patch,Vec2f(uu,vv),size_t(0));
    }

    void eval(const GeneralCatmullClarkPatch& patch, Vec2f uv, const size_t depth) // FIXME: no recursion required
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch())) 
      {
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));
        CatmullClarkPatch qpatch; patch.init(qpatch);
	eval(qpatch,uv,srange,depth); 
	return;
      }

      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
      patch.subdivide(patches,N); // FIXME: only have to generate one of the patches

      /* parametrization for triangles */
      if (N == 3) 
      {
        const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
        const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
        const bool ab_abc = right_of_line(ab,abc,uv);
        const bool ac_abc = right_of_line(ac,abc,uv);
        const bool bc_abc = right_of_line(bc,abc,uv);
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));

        const float u = uv.x, v = uv.y, w = 1.0f-u-v;
        if  (!ab_abc &&  ac_abc) { // FIXME: simplify
          const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(u,v));
          eval(patches[0],xy,srange,depth+1);
          if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
        }
        else if ( ab_abc && !bc_abc) {
          const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(v,w));
          eval(patches[1],xy,srange,depth+1);
          if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
        }
        else {
          const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(w,u));
          eval(patches[2],xy,srange,depth+1);
          if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
        }
      } 

      /* parametrization for quads */
      else if (N == 4) 
      {
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));
        float u = uv.x, v = uv.y;
        if (uv.y < 0.5f) {
          if (uv.x < 0.5f) {
            eval(patches[0],Vec2f(2.0f*u,2.0f*v),srange,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = 2.0f*dpdx;
              *dPdv = 2.0f*dpdy;
            }
          }
          else {
            eval(patches[1],Vec2f(2.0f*v,2.0f-2.0f*u),srange,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -2.0f*dpdy;
              *dPdv = 2.0f*dpdx;
            }
          }
        } else {
          if (uv.x > 0.5f) {
            eval(patches[2],Vec2f(2.0f-2.0f*u,2.0f-2.0f*v),srange,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = -2.0f*dpdx;
              *dPdv = -2.0f*dpdy;
            }
          }
          else {
            eval(patches[3],Vec2f(2.0f-2.0f*v,2.0f*u),srange,depth+1);
            if (dPdu && dPdv) {
              const Vertex dpdx = *dPdu, dpdy = *dPdv;
              *dPdu = 2.0f*dpdy;
              *dPdv = -2.0f*dpdx;
            }
          }
        }
      }

      /* parametrization for arbitrary polygons */
      else 
      {
        const unsigned i = floorf(uv.x); assert(i<N);
        const BBox2f srange(Vec2f(0.0f,0.0f),Vec2f(1.0f,1.0f));
        eval(patches[i],Vec2f(floorf(uv.x),uv.y),srange,depth+1); // FIXME: uv encoding creates issues as uv=(1,0) will refer to second quad
      }
    }

    void eval(CatmullClarkPatch& patch, const Vec2f& uv, BBox2f srange, size_t depth)
    {
      /*! recursively subdivide */
      while (!patch.isRegularOrFinal2(depth)) 
      {
        array_t<CatmullClarkPatch,4> patches; 
        patch.subdivide(patches); // FIXME: only have to generate one of the patches
        
        const Vec2f c = srange.center();
        if (uv.y < c.y) {
          if (uv.x < c.x) { patch = patches[0]; srange = BBox2f(srange.lower,c); }
          else            { patch = patches[1]; srange = BBox2f(Vec2f(c.x,srange.lower.y),Vec2f(srange.upper.x,c.y)); }
        } else {
          if (uv.x > c.x) { patch = patches[2]; srange = BBox2f(c,srange.upper); }
          else            { patch = patches[3]; srange = BBox2f(Vec2f(srange.lower.x,c.y),Vec2f(c.x,srange.upper.y)); }
        }
        depth++;
      }

      const float scaleU = rcp(srange.upper.x-srange.lower.x);
      const float scaleV = rcp(srange.upper.y-srange.lower.y);

      /*! use either B-spline or bilinear patch to interpolate */
      if (patch.isRegular()) 
      {
        BSplinePatch bspline; bspline.init(patch);
        const float fx = (uv.x-srange.lower.x)*scaleU;
        const float fy = (uv.y-srange.lower.y)*scaleV;
        if (P   ) *P    = bspline.eval(fx,fy);
        if (dPdu) *dPdu = bspline.tangentU(fx,fy)*scaleU; 
        if (dPdv) *dPdv = bspline.tangentV(fx,fy)*scaleV; 
      }
      else 
      {
        const float sx1 = (uv.x-srange.lower.x)*scaleU, sx0 = 1.0f-sx1;
        const float sy1 = (uv.y-srange.lower.y)*scaleV, sy0 = 1.0f-sy1;
        const Vertex P0 = patch.ring[0].getLimitVertex();
        const Vertex P1 = patch.ring[1].getLimitVertex();
        const Vertex P2 = patch.ring[2].getLimitVertex();
        const Vertex P3 = patch.ring[3].getLimitVertex();
        if (P   ) *P    = sy0*(sx0*P0+sx1*P1) + sy1*(sx0*P3+sx1*P2);
        if (dPdu) *dPdu = (sy0*(P1-P0) + sy1*(P2-P3))*scaleU; 
        if (dPdv) *dPdv = (sx0*(P3-P0) + sx1*(P2-P1))*scaleV;
      }
    }
  };

  template<typename Vertex, typename Vertex_t = Vertex>
    struct __aligned(64) Patch
  {
  public:

    static const size_t MAX_DEPTH = 4;
    
    typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;

    enum Type {
      INVALID_PATCH = 0,
      BSPLINE_PATCH = 1,  
      GREGORY_PATCH = 2,
      EVAL_PATCH = 3,
      SUBDIVIDED_GENERAL_QUAD_PATCH = 4,
      SUBDIVIDED_QUAD_PATCH = 5,
      SUBDIVIDED_GENERAL_TRIANGLE_PATCH = 6
    };

    struct BSplinePatch 
    {
      template<typename Loader, typename Allocator>
        __noinline static BSplinePatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const Loader& loader) {
        return new (alloc(sizeof(BSplinePatch))) BSplinePatch(edge,loader);
      }

      template<typename Allocator>
      __noinline static BSplinePatch* create(const Allocator& alloc, const CatmullClarkPatch& patch) {
        return new (alloc(sizeof(BSplinePatch))) BSplinePatch(patch);
      }

      template<typename Loader>
      __noinline BSplinePatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
        : type(BSPLINE_PATCH) { data.init(edge,loader); }

      __forceinline BSplinePatch (const CatmullClarkPatch& patch) 
        : type(BSPLINE_PATCH), data(patch) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale) const
      {
        if (P)    *P    = data.eval(u,v); 
        if (dPdu) *dPdu = data.tangentU(u,v)*dscale; 
        if (dPdv) *dPdv = data.tangentV(u,v)*dscale; 
#if PATCH_DEBUG_SUBDIVISION
        size_t hex = (size_t)this;
        for (size_t i=0; i<4; i++) hex = hex ^ (hex >> 8);
        const float c = (float)(hex&0xff)/255.0f;
        if (P) *P = Vertex(c,0.0f,0.0f,0.0f);
#endif  
        return true;
      }

      Type type;
      BSplinePatchT<Vertex,Vertex_t> data;
    };

    struct GregoryPatch
    {
      template<typename Loader, typename Allocator>
        __noinline static GregoryPatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const Loader& loader) {
        return new (alloc(sizeof(GregoryPatch))) GregoryPatch(edge,loader);
      }

      template<typename Allocator>
        __noinline static GregoryPatch* create(const Allocator& alloc, const CatmullClarkPatch& patch) {
        return new (alloc(sizeof(GregoryPatch))) GregoryPatch(patch);
      }

      template<typename Loader>
      __forceinline GregoryPatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
        : type(GREGORY_PATCH) { CatmullClarkPatch patch; patch.init2(edge,loader); data.init(patch); }

      __forceinline GregoryPatch (const CatmullClarkPatch& patch) 
        : type(GREGORY_PATCH), data(patch) {}

      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale) const
      {
        if (P)    *P    = data.eval(u,v); 
        if (dPdu) *dPdu = data.tangentU(u,v)*dscale; 
        if (dPdv) *dPdv = data.tangentV(u,v)*dscale; 
#if PATCH_DEBUG_SUBDIVISION
        size_t hex = (size_t)this;
        for (size_t i=0; i<4; i++) hex = hex ^ (hex >> 8);
        const float c = (float)(hex&0xff)/255.0f;
        if (P) *P = Vertex(0.0f,c,0.0f,0.0f);
#endif  
        return true;
      }

      Type type;
      GregoryPatchT<Vertex,Vertex_t> data;
    };

    struct EvalPatch
    {
      template<typename Allocator>
        __noinline static EvalPatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride) {
        return new (alloc(sizeof(EvalPatch))) EvalPatch(edge,vertices,stride);
      }

      __forceinline EvalPatch (const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride)
        : type(EVAL_PATCH), edge(edge), vertices(vertices), stride(stride), child(nullptr) {}
      
      bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
      {
        auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
          const unsigned vtx = p->getStartVertexIndex();
          return Vertex_t::loadu((float*)&vertices[vtx*stride]); 
        };
      
        if (child && child->eval(u,v,P,dPdu,dPdv,1.0f)) 
          return true;
        
        GeneralCatmullClarkPatch patch;
        patch.init2(edge,loader);
        FeatureAdaptivePointEval<Vertex,Vertex_t> eval(patch,u,v,P,dPdu,dPdv); 
        
#if PATCH_DEBUG_SUBDIVISION
        size_t hex = (size_t)this;
        for (size_t i=0; i<4; i++) hex = hex ^ (hex >> 8);
        const float c = (float)(hex&0xff)/255.0f;
        if (P) *P = Vertex(0.0f,0.0f,c,0.0f);
#endif  
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
    
    __forceinline SubdividedGeneralTrianglePatch() : type(SUBDIVIDED_GENERAL_TRIANGLE_PATCH) {}
    
    bool eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv)
    {
      const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
      const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
      const bool ab_abc = right_of_line(ab,abc,Vec2f(u,v));
      const bool ac_abc = right_of_line(ac,abc,Vec2f(u,v));
      const bool bc_abc = right_of_line(bc,abc,Vec2f(u,v));

      const float w = 1.0f-u-v;
      if  (!ab_abc &&  ac_abc) {
        const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(u,v));
        if (!child[0]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
        if (dPdu && dPdv) map_quad0_to_tri(xy,*dPdu,*dPdv);
      }
      else if ( ab_abc && !bc_abc) {
        const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(v,w));
        if (!child[1]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
        if (dPdu && dPdv) map_quad1_to_tri(xy,*dPdu,*dPdv);
      }
      else {
        const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(w,u));
        if (!child[2]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f)) return false;
        if (dPdu && dPdv) map_quad2_to_tri(xy,*dPdu,*dPdv);
      }
      return true;
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
      
      switch (edge->type) {
      case SubdivMesh::REGULAR_QUAD_PATCH:   child = (Patch*) BSplinePatch::create(alloc,edge,loader); break;
        //case SubdivMesh::IRREGULAR_QUAD_PATCH: child = (Patch*) GregoryPatch::create(alloc,edge,loader); break;
      default: if (MAX_DEPTH > 0) {
          GeneralCatmullClarkPatch patch;
          patch.init2(edge,loader);
          child = (Patch*) Patch::create(alloc,patch,edge,vertices,stride,0);
          break;
        }
      }

      root->child = child;
      return (Patch*) root;
    }

  template<typename Allocator>
  __noinline static Patch* create(const Allocator& alloc, GeneralCatmullClarkPatch& patch, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, size_t depth)
  {
    /* convert into standard quad patch if possible */
    if (likely(patch.isQuadPatch())) {
      CatmullClarkPatch qpatch; patch.init(qpatch);
      return Patch::create(alloc,qpatch,edge,vertices,stride,depth);
    }
    else if (depth >= MAX_DEPTH) {
      return nullptr;
    }
    else 
    {
      size_t N;
      array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
      patch.subdivide(patches,N);

      Patch* ret = nullptr;
      if (N == 3) {
        SubdividedGeneralTrianglePatch* node = SubdividedGeneralTrianglePatch::create(alloc);
        for (size_t i=0; i<3; i++)
          node->child[i] = Patch::create(alloc,patches[i],edge,vertices,stride,depth+1);
        ret = (Patch*) node;
      } 
      else if (N == 4) {
        SubdividedGeneralQuadPatch* node = SubdividedGeneralQuadPatch::create(alloc);
        for (size_t i=0; i<4; i++)
          node->child[i] = Patch::create(alloc,patches[i],edge,vertices,stride,depth+1);
        ret = (Patch*) node;
      }
      else 
        assert(false);
      
      return ret;
    }
  }

  template<typename Allocator>
  __noinline static Patch* create(const Allocator& alloc, CatmullClarkPatch& patch, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, size_t depth)
  {
    if (patch.isRegular()) { assert(depth > 0); return (Patch*) BSplinePatch::create(alloc,patch); }
    //else if (patch.isGregory()) { assert(depth > 0); return (Patch*) GregoryPatch::create(alloc,patch); }
    else if (depth >= MAX_DEPTH) return nullptr;
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

  public:
    Type type;
  };
}
