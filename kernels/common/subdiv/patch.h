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
#include "bilinear_patch.h"
#include "bspline_patch.h"
#include "bezier_patch.h"
#include "gregory_patch.h"
#include "gregory_triangle_patch.h"
#include "tessellation_cache.h"

#if 1
#define PATCH_DEBUG_SUBDIVISION(ptr,x,y,z)
#else
#define PATCH_DEBUG_SUBDIVISION(ptr,x,y,z)            \
  {                                                   \
    size_t hex = (size_t)ptr;                          \
    for (size_t i=0; i<4; i++) hex = hex ^ (hex >> 8);  \
    const float c = (float)(((hex >> 0) ^ (hex >> 4) ^ (hex >> 8) ^ (hex >> 12) ^ (hex >> 16))&0xf)/15.0f; \
    if (P) *P = Vertex(0.5f+0.5f*x,0.5f+0.5f*y,0.5f+0.5f*z,0.0f);         \
    }               
#endif

#define PATCH_MAX_CACHE_DEPTH 2
#define PATCH_MIN_RESOLUTION 1     // FIXME: not yet completely implemented
#define PATCH_MAX_EVAL_DEPTH_IRREGULAR 2     // maximal evaluation depth at irregular vertices (has to be larger or equal than PATCH_MAX_CACHE_DEPTH)
#define PATCH_MAX_EVAL_DEPTH_CREASE 10       // maximal evaluation depth at crease features (has to be larger or equal than PATCH_MAX_CACHE_DEPTH)
#define PATCH_USE_GREGORY 1        // 0 = no gregory, 1 = fill, 2 = as early as possible

#if PATCH_USE_GREGORY==2
#define PATCH_USE_BEZIER_PATCH 1   // enable use of bezier instead of b-spline patches
#else
#define PATCH_USE_BEZIER_PATCH 0   // enable use of bezier instead of b-spline patches
#endif

#if PATCH_USE_BEZIER_PATCH
#  define RegularPatch  BezierPatch
#  define RegularPatchT BezierPatchT<Vertex,Vertex_t>
#else
#  define RegularPatch  BSplinePatch
#  define RegularPatchT BSplinePatchT<Vertex,Vertex_t>
#endif

#if PATCH_USE_GREGORY
#define IrregularFillPatch GregoryPatch
#define IrregularFillPatchT GregoryPatchT<Vertex,Vertex_t>
#else
#define IrregularFillPatch BilinearPatch
#define IrregularFillPatchT BilinearPatchT<Vertex,Vertex_t>
#endif

namespace embree
{
  template<typename vfloat>
  __forceinline Vec2<vfloat> map_tri_to_quad(const Vec2<vfloat>& uv)
  {
    const Vec2<vfloat> a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2<vfloat> ab = vfloat(0.5f)*(a+b), ac = vfloat(0.5f)*(a+c), bc = vfloat(0.5f)*(b+c), abc = vfloat(1.0f/3.0f)*(a+b+c);
    const Vec2<vfloat> A = a, B = ab-a, C = ac-a, D = a-ab-ac+abc;
    const vfloat AA = det(D,C), BB = det(D,A) + det(B,C) + det(uv,D), CC = det(B,A) + det(uv,B);
    const vfloat vv = (-BB+sqrt(BB*BB-4.0f*AA*CC))/(2.0f*AA);
    const vfloat uu = (uv.x - A.x - vv*C.x)/(B.x + vv*D.x);
    return Vec2<vfloat>(uu,vv);
  }
  
  template<typename vfloat>
    __forceinline Vec2<vfloat> map_quad_to_tri_dx(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2<vfloat>& xy) {
    return (1.0f-xy.y)*Vec2<vfloat>(ab-a) + xy.y*Vec2<vfloat>(abc-ac);
  }
  
  template<typename vfloat>
    __forceinline Vec2<vfloat> map_quad_to_tri_dy(const Vec2f& a, const Vec2f& ab, const Vec2f& abc, const Vec2f& ac, const Vec2<vfloat>& xy) {
    return (1.0f-xy.x)*Vec2<vfloat>(ac-a) + xy.x*Vec2<vfloat>(abc-ab);
  }
  
  template<typename vfloat>
    __forceinline auto right_of_line(const Vec2f& A, const Vec2f& B, const Vec2<vfloat>& P) -> decltype(P.x<P.x) {
    return det(Vec2<vfloat>(A)-P,Vec2<vfloat>(B)-P) <= 0.0f;
  }
  
  template<typename vfloat>
    __forceinline auto right_of_line_ab_abc(const Vec2<vfloat>& uv) -> decltype(uv.x<uv.x)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    return right_of_line(ab,abc,uv);
  }
  
  template<typename vfloat>
    __forceinline auto right_of_line_ac_abc(const Vec2<vfloat>& uv) -> decltype(uv.x<uv.x)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    return right_of_line(ac,abc,uv);
  }
  
  template<typename vfloat>
    __forceinline auto right_of_line_bc_abc(const Vec2<vfloat>& uv) -> decltype(uv.x<uv.x)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    return right_of_line(bc,abc,uv);
  }
  
  template<typename vfloat, typename Vertex>
    __forceinline void map_quad0_to_tri(const Vec2<vfloat>& xy, Vertex& dPdu, Vertex& dPdv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    const Vertex dpdx = dPdu, dpdy = dPdv;
    const Vec2<vfloat> duvdx = map_quad_to_tri_dx(a,ab,abc,ac,xy);
    const Vec2<vfloat> duvdy = map_quad_to_tri_dy(a,ab,abc,ac,xy);
    const LinearSpace2<Vec2<vfloat> > J = rcp(LinearSpace2<Vec2<vfloat> >(duvdx,duvdy));
    dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
    dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
  }
  
  template<typename vfloat, typename Vertex>
    __forceinline void map_quad1_to_tri(const Vec2<vfloat>& xy, Vertex& dPdu, Vertex& dPdv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    const Vertex dpdx = dPdu, dpdy = dPdv;
    const Vec2<vfloat> duvdx = map_quad_to_tri_dx(b,bc,abc,ab,xy);
    const Vec2<vfloat> duvdy = map_quad_to_tri_dy(b,bc,abc,ab,xy);
    const LinearSpace2<Vec2<vfloat> > J = rcp(LinearSpace2<Vec2<vfloat> >(duvdx,duvdy));
    dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
    dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
  }
  
  template<typename vfloat, typename Vertex>
    __forceinline void map_quad2_to_tri(const Vec2<vfloat>& xy, Vertex& dPdu, Vertex& dPdv)
  {
    const Vec2f a(0.0f,0.0f), b(1.0f,0.0f), c(0.0f,1.0f);
    const Vec2f ab = 0.5f*(a+b), ac = 0.5f*(a+c), bc = 0.5f*(b+c), abc = (1.0f/3.0f)*(a+b+c);
    const Vertex dpdx = dPdu, dpdy = dPdv;
    const Vec2<vfloat> duvdx = map_quad_to_tri_dx(c,ac,abc,bc,xy);
    const Vec2<vfloat> duvdy = map_quad_to_tri_dy(c,ac,abc,bc,xy);
    const LinearSpace2<Vec2<vfloat> > J = rcp(LinearSpace2<Vec2<vfloat> >(duvdx,duvdy));
    dPdu = dpdx*J.vx.x + dpdy*J.vx.y;
    dPdv = dpdx*J.vy.x + dpdy*J.vy.y;
  }
  
  template<typename vbool, typename vfloat>
    __forceinline void map_quad0_to_tri(const vbool& valid, const Vec2<vfloat>& xy, float* dPdu, float* dPdv, size_t dstride, size_t i)
  {
    vfloat dPdut = vfloat::loadu(dPdu+i*dstride), dPdvt = vfloat::loadu(dPdv+i*dstride);
    map_quad0_to_tri(xy,dPdut,dPdvt); 
    vfloat::store(valid,dPdu+i*dstride,dPdut);
    vfloat::store(valid,dPdv+i*dstride,dPdvt);
  }

  template<typename vbool, typename vfloat>
    __forceinline void map_quad1_to_tri(const vbool& valid, const Vec2<vfloat>& xy, float* dPdu, float* dPdv, size_t dstride, size_t i)
  {
    vfloat dPdut = vfloat::loadu(dPdu+i*dstride), dPdvt = vfloat::loadu(dPdv+i*dstride);
    map_quad1_to_tri(xy,dPdut,dPdvt); 
    vfloat::store(valid,dPdu+i*dstride,dPdut);
    vfloat::store(valid,dPdv+i*dstride,dPdvt);
  }

  template<typename vbool, typename vfloat>
    __forceinline void map_quad2_to_tri(const vbool& valid, const Vec2<vfloat>& xy, float* dPdu, float* dPdv, size_t dstride, size_t i)
  {
    vfloat dPdut = vfloat::loadu(dPdu+i*dstride), dPdvt = vfloat::loadu(dPdv+i*dstride);
    map_quad2_to_tri(xy,dPdut,dPdvt); 
    vfloat::store(valid,dPdu+i*dstride,dPdut);
    vfloat::store(valid,dPdv+i*dstride,dPdvt);
  }

  template<typename Vertex, typename Vertex_t = Vertex>
    struct __aligned(64) PatchT
    {
    public:
    
    typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
    typedef CatmullClark1RingT<Vertex,Vertex_t> CatmullClarkRing;
    typedef BezierCurveT<Vertex> BezierCurve;
    
    enum Type {
      INVALID_PATCH = 0,
      BILINEAR_PATCH = 1,
      BSPLINE_PATCH = 2,  
      BEZIER_PATCH = 3,  
      GREGORY_PATCH = 4,
      SUBDIVIDED_GENERAL_TRIANGLE_PATCH = 5,
      //SUBDIVIDED_GENERAL_QUAD_PATCH = 6,
      SUBDIVIDED_GENERAL_PATCH = 7,
      SUBDIVIDED_QUAD_PATCH = 8,
      EVAL_PATCH = 9,
    };
    
    struct Ref
    {
      __forceinline Ref(void* p = nullptr) 
        : ptr((size_t)p) {}

      __forceinline operator bool() const { return ptr != 0; }
      __forceinline operator size_t() const { return ptr; }

      __forceinline Ref (Type ty, void* in) 
        : ptr(((size_t)in)+ty) { assert((((size_t)in) & 0xF) == 0); }

      __forceinline Type  type  () const { return (Type)(ptr & 0xF); }
      __forceinline void* object() const { return (void*) (ptr & ~0xF); }

      size_t ptr;
    };

    struct EvalPatch 
    {
      /* creates EvalPatch from a CatmullClarkPatch */
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const CatmullClarkPatch& patch) 
      {
        size_t ofs = 0, bytes = patch.bytes();
        void* ptr = alloc(bytes);
        patch.serialize(ptr,ofs);
        assert(ofs == bytes);
        return Ref(EVAL_PATCH, ptr);
      }
    };

    struct BilinearPatch 
    {
      /* creates BilinearPatch from a CatmullClarkPatch */
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const CatmullClarkPatch& patch,
                                   const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) {
        return Ref(BILINEAR_PATCH, new (alloc(sizeof(BilinearPatch))) BilinearPatch(patch,border0,border1,border2,border3));
      }
      
      __forceinline BilinearPatch (const CatmullClarkPatch& patch, const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) 
        : patch(patch,border0,border1,border2,border3) {}
      
    public:
      BilinearPatchT<Vertex,Vertex_t> patch;
    };
    
    struct BSplinePatch 
    {
      /* creates BSplinePatch from a half edge */
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const HalfEdge* edge, const char* vertices, size_t stride) {
        return Ref(BSPLINE_PATCH, new (alloc(sizeof(BSplinePatch))) BSplinePatch(edge,vertices,stride));
      }
      
      __forceinline BSplinePatch (const HalfEdge* edge, const char* vertices, size_t stride) 
        : patch(edge,vertices,stride) {}
      
      /* creates BSplinePatch from a CatmullClarkPatch */
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const CatmullClarkPatch& patch,
                                   const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) {
        return Ref(BSPLINE_PATCH, new (alloc(sizeof(BSplinePatch))) BSplinePatch(patch,border0,border1,border2,border3));
      }
      
      __forceinline BSplinePatch (const CatmullClarkPatch& patch, const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) 
        : patch(patch,border0,border1,border2,border3) {}
      
    public:
      BSplinePatchT<Vertex,Vertex_t> patch;
    };

    struct BezierPatch
    {
      /* creates BezierPatch from a half edge */
      template<typename Allocator>
        __noinline static Ref create(const Allocator& alloc, const HalfEdge* edge, const char* vertices, size_t stride) {
        return Ref(BEZIER_PATCH, new (alloc(sizeof(BezierPatch))) BezierPatch(edge,vertices,stride));
      }
      
      __forceinline BezierPatch (const HalfEdge* edge, const char* vertices, size_t stride) 
        : patch(edge,vertices,stride) {}
      
      /* creates Bezier from a CatmullClarkPatch */
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const CatmullClarkPatch& patch,
                                   const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) {
        return Ref(BEZIER_PATCH, new (alloc(sizeof(BezierPatch))) BezierPatch(patch,border0,border1,border2,border3));
      }
      
      __forceinline BezierPatch (const CatmullClarkPatch& patch, const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) 
        : patch(patch,border0,border1,border2,border3) {}
      
    public:
      BezierPatchT<Vertex,Vertex_t> patch;
    };
    
    struct GregoryPatch
    {
      /* creates GregoryPatch from half edge */
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const HalfEdge* edge, const char* vertices, size_t stride) {
        return Ref(GREGORY_PATCH, new (alloc(sizeof(GregoryPatch))) GregoryPatch(edge,vertices,stride));
      }
      
      __forceinline GregoryPatch (const HalfEdge* edge, const char* vertices, size_t stride) 
        : patch(CatmullClarkPatch(edge,vertices,stride)) {}
       
      /* creates GregoryPatch from CatmullClarkPatch */
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const CatmullClarkPatch& patch,
                                   const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) {
        return Ref(GREGORY_PATCH, new (alloc(sizeof(GregoryPatch))) GregoryPatch(patch,border0,border1,border2,border3));
      }
      
      __forceinline GregoryPatch (const CatmullClarkPatch& patch, const BezierCurve* border0, const BezierCurve* border1, const BezierCurve* border2, const BezierCurve* border3) 
        : patch(patch,border0,border1,border2,border3) {}
      
    public:
      GregoryPatchT<Vertex,Vertex_t> patch;
    };
    
    struct SubdividedGeneralTrianglePatch
    {
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, Ref children[3]) {
        return Ref(SUBDIVIDED_GENERAL_TRIANGLE_PATCH, new (alloc(sizeof(SubdividedGeneralTrianglePatch))) SubdividedGeneralTrianglePatch(children));
      }
      
      __forceinline SubdividedGeneralTrianglePatch(Ref children[3]) {
        for (size_t i=0; i<3; i++) child[i] = children[i];
      }
      
      Ref child[3];
    };
    
    struct SubdividedQuadPatch
    {
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, Ref children[4]) {
        return Ref(SUBDIVIDED_QUAD_PATCH, new (alloc(sizeof(SubdividedQuadPatch))) SubdividedQuadPatch(children));
      }
      
      __forceinline SubdividedQuadPatch(Ref children[4]) {
        for (size_t i=0; i<4; i++) child[i] = children[i];
      }
      
    public:
      Ref child[4];
    };
    
    struct SubdividedGeneralPatch
    {
      template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, Ref* children, const size_t N) {
        return Ref(SUBDIVIDED_GENERAL_PATCH, new (alloc(sizeof(SubdividedGeneralPatch))) SubdividedGeneralPatch(children,N));
      }
      
      __forceinline SubdividedGeneralPatch(Ref* children, const size_t N) : N(N) {
        for (size_t i=0; i<N; i++) child[i] = children[i];
      }
      
      size_t N;
      Ref child[MAX_PATCH_VALENCE];
    };
    
    /*! Default constructor. */
    __forceinline PatchT () {}
    
    template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, const HalfEdge* edge, const char* vertices, size_t stride)
    {
      if (PATCH_MAX_CACHE_DEPTH == 0) 
        return nullptr;

      Ref child(0);
      switch (edge->patch_type) {
      case HalfEdge::REGULAR_QUAD_PATCH:   child = RegularPatch::create(alloc,edge,vertices,stride); break;
#if PATCH_USE_GREGORY == 2
      case HalfEdge::IRREGULAR_QUAD_PATCH: child = GregoryPatch::create(alloc,edge,vertices,stride); break;
#endif
      default: {
        GeneralCatmullClarkPatch patch(edge,vertices,stride);
        child = PatchT::create(alloc,patch,edge,vertices,stride,0);
      }
      }
      return child;
    }

    template<typename Allocator>
    __noinline static Ref create(const Allocator& alloc, GeneralCatmullClarkPatch& patch, const HalfEdge* edge, const char* vertices, size_t stride, size_t depth)
    {
      /* convert into standard quad patch if possible */
      if (likely(patch.isQuadPatch())) 
      {
        CatmullClarkPatch qpatch; patch.init(qpatch);
        return PatchT::create(alloc,qpatch,edge,vertices,stride,depth);
      }
      
      if (depth >= PATCH_MAX_CACHE_DEPTH)
        return nullptr;
      
      /* subdivide patch */
      size_t N;
      array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
      patch.subdivide(patches,N);
      
      if (N == 3) 
      {
        Ref child[3];
#if PATCH_USE_GREGORY == 2
        BezierCurve borders[GeneralCatmullClarkPatch::SIZE]; patch.getLimitBorder(borders);
        BezierCurve border0l,border0r; borders[0].subdivide(border0l,border0r);
        BezierCurve border1l,border1r; borders[1].subdivide(border1l,border1r);
        BezierCurve border2l,border2r; borders[2].subdivide(border2l,border2r);
        child[0] = PatchT::create(alloc,patches[0],edge,vertices,stride,depth+1, &border0l, nullptr, nullptr, &border2r);
        child[1] = PatchT::create(alloc,patches[1],edge,vertices,stride,depth+1, &border1l, nullptr, nullptr, &border0r);
        child[2] = PatchT::create(alloc,patches[2],edge,vertices,stride,depth+1, &border2l, nullptr, nullptr, &border1r);
#else
        for (size_t i=0; i<3; i++)
          child[i] = PatchT::create(alloc,patches[i],edge,vertices,stride,depth+1);
#endif
        return SubdividedGeneralTrianglePatch::create(alloc,child);
      } 
      else if (N == 4) 
      {
        Ref child[4];
#if PATCH_USE_GREGORY == 2
        BezierCurve borders[GeneralCatmullClarkPatch::SIZE]; patch.getLimitBorder(borders);
        BezierCurve border0l,border0r; borders[0].subdivide(border0l,border0r);
        BezierCurve border1l,border1r; borders[1].subdivide(border1l,border1r);
        BezierCurve border2l,border2r; borders[2].subdivide(border2l,border2r);
        BezierCurve border3l,border3r; borders[3].subdivide(border3l,border3r);
        GeneralCatmullClarkPatch::fix_quad_ring_order(patches);
        child[0] = PatchT::create(alloc,patches[0],edge,vertices,stride,depth+1,&border0l,nullptr,nullptr,&border3r);
        child[1] = PatchT::create(alloc,patches[1],edge,vertices,stride,depth+1,&border0r,&border1l,nullptr,nullptr);
        child[2] = PatchT::create(alloc,patches[2],edge,vertices,stride,depth+1,nullptr,&border1r,&border2l,nullptr);
        child[3] = PatchT::create(alloc,patches[3],edge,vertices,stride,depth+1,nullptr,nullptr,&border2r,&border3l);
#else
        GeneralCatmullClarkPatch::fix_quad_ring_order(patches);
        for (size_t i=0; i<4; i++)
          child[i] = PatchT::create(alloc,patches[i],edge,vertices,stride,depth+1);
#endif
        return SubdividedQuadPatch::create(alloc,child);
      }
      else 
      {
        assert(N<MAX_PATCH_VALENCE);
        Ref child[MAX_PATCH_VALENCE];
        
#if PATCH_USE_GREGORY == 2
        BezierCurve borders[GeneralCatmullClarkPatch::SIZE]; 
        patch.getLimitBorder(borders);

        for (size_t i0=0; i0<N; i0++) {
          const size_t i2 = i0==0 ? N-1 : i0-1; 
          BezierCurve border0l,border0r; borders[i0].subdivide(border0l,border0r);
          BezierCurve border2l,border2r; borders[i2].subdivide(border2l,border2r);
          child[i0] = PatchT::create(alloc,patches[i0],edge,vertices,stride,depth+1, &border0l, nullptr, nullptr, &border2r);
        }
#else
        for (size_t i=0; i<N; i++)
          child[i] = PatchT::create(alloc,patches[i],edge,vertices,stride,depth+1);
#endif
        return SubdividedGeneralPatch::create(alloc,child,N);
      }
      
      return nullptr;
    }

    static __forceinline bool final(const CatmullClarkPatch& patch, const typename CatmullClarkRing::Type type, size_t depth) 
    {
      const int max_eval_depth = (type & CatmullClarkRing::TYPE_CREASES) ? PATCH_MAX_EVAL_DEPTH_CREASE : PATCH_MAX_EVAL_DEPTH_IRREGULAR;
//#if PATCH_MIN_RESOLUTION
//      return patch.isFinalResolution(PATCH_MIN_RESOLUTION) || depth>=max_eval_depth;
//#else
      return depth>=max_eval_depth;
//#endif
    }

    template<typename Allocator>
      __noinline static Ref create(const Allocator& alloc, CatmullClarkPatch& patch, const HalfEdge* edge, const char* vertices, size_t stride, size_t depth,
                                   const BezierCurve* border0 = nullptr, const BezierCurve* border1 = nullptr, const BezierCurve* border2 = nullptr, const BezierCurve* border3 = nullptr)
    {
      const typename CatmullClarkPatch::Type ty = patch.type();
      if (unlikely(final(patch,ty,depth))) {
        if (ty & CatmullClarkRing::TYPE_REGULAR) return RegularPatch::create(alloc,patch,border0,border1,border2,border3); 
        else                                     return IrregularFillPatch::create(alloc,patch,border0,border1,border2,border3); 
      }
      else if (ty & CatmullClarkRing::TYPE_REGULAR_CREASES) { 
        assert(depth > 0); return RegularPatch::create(alloc,patch,border0,border1,border2,border3); 
      }
#if PATCH_USE_GREGORY == 2
      else if (ty & CatmullClarkRing::TYPE_GREGORY_CREASES) { 
        assert(depth > 0); return GregoryPatch::create(alloc,patch,border0,border1,border2,border3); 
      }
#endif
      else if (depth >= PATCH_MAX_CACHE_DEPTH) {
        return EvalPatch::create(alloc,patch); 
      }
      
      else 
      {
        Ref child[4];
        array_t<CatmullClarkPatch,4> patches; 
        patch.subdivide(patches);
        
        for (size_t i=0; i<4; i++)
          child[i] = PatchT::create(alloc,patches[i],edge,vertices,stride,depth+1);
        return SubdividedQuadPatch::create(alloc,child);
      }
    }
  };

  typedef PatchT<Vec3fa,Vec3fa_t> Patch3fa;
}
