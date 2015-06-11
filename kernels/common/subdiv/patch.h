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
#include "tessellation_cache.h"

namespace embree
{
  template<typename Vertex, typename Vertex_t = Vertex>
    struct __aligned(64) Patch
  {
  public:

    static const size_t MAX_DEPTH = 0;
    
    typedef GeneralCatmullClarkPatchT<Vertex,Vertex_t> GeneralCatmullClarkPatch;
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;

    enum Type {
      INVALID_PATCH = 0,
      BSPLINE_PATCH = 1,  
      GREGORY_PATCH = 2,
      EVAL_PATCH = 3,
      SUBDIVIDED_QUAD_PATCH = 4,
      SUBDIVIDED_TRIANGLE_PATCH = 5
    };

    struct BSplinePatch 
    {
      template<typename Loader, typename Allocator>
        __forceinline static BSplinePatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const Loader& loader) {
        return new (alloc(sizeof(BSplinePatch))) BSplinePatch(edge,loader);
      }

      template<typename Allocator>
        __forceinline static BSplinePatch* create(const Allocator& alloc, const CatmullClarkPatch& patch) {
        return new (alloc(sizeof(BSplinePatch))) BSplinePatch(patch);
      }

      template<typename Loader>
      __forceinline BSplinePatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
        : type(BSPLINE_PATCH) { data.init(edge,loader); }

      __forceinline BSplinePatch (const CatmullClarkPatch& patch) 
        : type(BSPLINE_PATCH), data(patch) {}
      
      __forceinline void eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
      {
        if (P)    *P    = data.eval(u,v); 
        if (dPdu) *dPdu = data.tangentU(u,v); 
        if (dPdv) *dPdv = data.tangentV(u,v); 
      }

      Type type;
      BSplinePatchT<Vertex,Vertex_t> data;
    };

    struct GregoryPatch
    {
      template<typename Loader, typename Allocator>
        __forceinline static GregoryPatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const Loader& loader) {
        return new (alloc(sizeof(BSplinePatch))) GregoryPatch(edge,loader);
      }

      template<typename Allocator>
        __forceinline static GregoryPatch* create(const Allocator& alloc, const CatmullClarkPatch& patch) {
        return new (alloc(sizeof(BSplinePatch))) GregoryPatch(patch);
      }

      template<typename Loader>
      __forceinline GregoryPatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
        : type(GREGORY_PATCH) { CatmullClarkPatch patch; patch.init2(edge,loader); data.init(patch); }

      __forceinline GregoryPatch (const CatmullClarkPatch& patch) 
        : type(GREGORY_PATCH), data(patch) {}

      __forceinline void eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
      {
        if (P)    *P    = data.eval(u,v); 
        if (dPdu) *dPdu = data.tangentU(u,v); 
        if (dPdv) *dPdv = data.tangentV(u,v); 
      }

      Type type;
      GregoryPatchT<Vertex,Vertex_t> data;
    };

    struct EvalPatch
    {
      template<typename Allocator>
        __forceinline static EvalPatch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride) {
        return new (alloc(sizeof(BSplinePatch))) EvalPatch(edge,vertices,stride);
      }

      __forceinline EvalPatch (const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride)
        : type(EVAL_PATCH), edge(edge), vertices(vertices), stride(stride) {}
      
      __forceinline void eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
      {
        auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
          const unsigned vtx = p->getStartVertexIndex();
          return Vertex_t::loadu((float*)&vertices[vtx*stride]);  // FIXME: reads behind the end of the array
        };
        
        GeneralCatmullClarkPatch patch;
        patch.init2(edge,loader);
        FeatureAdaptivePointEval<Vertex,Vertex_t> eval(patch,u,v,P,dPdu,dPdv); 
      }
      
    public:
      Type type;
      const SubdivMesh::HalfEdge* const edge;
      const char* const vertices;
      const size_t stride;
    };
    
    struct SubdividedTrianglePatch
  {
    template<typename Allocator>
    __forceinline static SubdividedTrianglePatch* create(const Allocator& alloc) {
        return new (alloc(sizeof(SubdividedTrianglePatch))) SubdividedTrianglePatch;
      }

    SharedLazyTessellationCache ::CacheEntry child[3];
    };
  
    struct SubdividedQuadPatch
    {
      template<typename Allocator>
      __forceinline static SubdividedQuadPatch* create(const Allocator& alloc) {
        return new (alloc(sizeof(SubdividedQuadPatch))) SubdividedQuadPatch;
      }

      SharedLazyTessellationCache ::CacheEntry child[4];
    };
   
    /*! Default constructor. */
    __forceinline Patch () {}

    template<typename Allocator>
      __forceinline static Patch* create(const Allocator& alloc, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride)
    {
      auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
        const unsigned vtx = p->getStartVertexIndex();
        return Vertex_t::loadu((float*)&vertices[vtx*stride]);
      };
      
      switch (edge->type) {
      case SubdivMesh::REGULAR_QUAD_PATCH:   return (Patch*) BSplinePatch::create(alloc,edge,loader);
      case SubdivMesh::IRREGULAR_QUAD_PATCH: return (Patch*) GregoryPatch::create(alloc,edge,loader);
      default: {
        GeneralCatmullClarkPatch patch;
        patch.init2(edge,loader);
        return (Patch*) Patch::create(alloc,patch,edge,vertices,stride,0);
      }
      }
    }

  template<typename Allocator>
  __forceinline static Patch* create(const Allocator& alloc, GeneralCatmullClarkPatch& patch, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, size_t depth)
  {
    /* convert into standard quad patch if possible */
    if (likely(patch.isQuadPatch())) {
      CatmullClarkPatch qpatch; patch.init(qpatch);
      return Patch::create(alloc,qpatch,edge,vertices,stride,depth);
    }
    else if (depth >= MAX_DEPTH) 
      return (Patch*) EvalPatch::create(alloc,edge,vertices,stride);
    else 
    {
      size_t N;
      array_t<CatmullClarkPatch,GeneralCatmullClarkPatch::SIZE> patches; 
      patch.subdivide(patches,N);

      Patch* ret = nullptr;
      if (N == 3) {
        SubdividedTrianglePatch* node = SubdividedTrianglePatch::create(alloc);
        for (size_t i=0; i<3; i++)
          node->child[i].tag = SharedLazyTessellationCache::Tag(Patch::create(alloc,patches[i],edge,vertices,stride,depth));
        ret = (Patch*) node;
      } 
      else if (N == 4) {
        SubdividedQuadPatch* node = SubdividedQuadPatch::create(alloc);
        for (size_t i=0; i<4; i++)
          node->child[i].tag = SharedLazyTessellationCache::Tag(Patch::create(alloc,patches[i],edge,vertices,stride,depth));
        ret = (Patch*) node;
      }
      else 
        assert(false);
      
      return ret;
    }
  }

  template<typename Allocator>
  __forceinline static Patch* create(const Allocator& alloc, CatmullClarkPatch& patch, const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride, size_t depth)
  {
    if (patch.isRegular()) return (Patch*) BSplinePatch::create(alloc,patch);
    else if (patch.isGregory()) return (Patch*) GregoryPatch::create(alloc,patch);
    else if (depth >= MAX_DEPTH) return (Patch*) EvalPatch::create(alloc,edge,vertices,stride);
    else {
      SubdividedQuadPatch* node = SubdividedQuadPatch::create(alloc);
      array_t<CatmullClarkPatch,4> patches; 
      patch.subdivide(patches);
      for (size_t i=0; i<4; i++)
        node->child[i].tag = SharedLazyTessellationCache::Tag(Patch::create(alloc,patches[i],edge,vertices,stride,depth+1));
      return (Patch*) node;
    }
  }

    __forceinline void eval(const float& u, const float& v, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
    {
      switch (type) {
      case BSPLINE_PATCH: ((BSplinePatch*)this)->eval(u,v,P,dPdu,dPdv); break;
      case GREGORY_PATCH: ((GregoryPatch*)this)->eval(u,v,P,dPdu,dPdv); break; 
      case EVAL_PATCH   : ((EvalPatch*)   this)->eval(u,v,P,dPdu,dPdv); break;
      //case SUBDIVIDED_QUAD_PATCH: 
      //((SubdividedQuadPatch*)data)->eval(u,v,P,dPdu,dPdv); 
      //break;
      //case SUBDIVIDED_TRIANGLE_PATCH: 
      //((SubdividedTrianglePatch*)data)->eval(u,v,P,dPdu,dPdv);
      //break;
      }
    }

  public:
    Type type;
  };
}
