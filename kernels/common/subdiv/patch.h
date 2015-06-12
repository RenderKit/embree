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
#include "feature_adaptive_eval.h"

#define PATCH_DEBUG_SUBDIVISION 0

namespace embree
{
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
        return child[0]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f);
      }
      else if ( ab_abc && !bc_abc) {
        const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(v,w));
        return child[1]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f);
      }
      else {
        const Vec2f xy = map_tri_to_quad(a,ab,abc,ac,Vec2f(w,u));
        return child[2]->eval(xy.x,xy.y,P,dPdu,dPdv,1.0f);
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
