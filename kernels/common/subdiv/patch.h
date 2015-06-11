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

namespace embree
{
  template<typename Vertex, typename Vertex_t = Vertex>
    struct __aligned(64) Patch
  {
  public:
    
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

      template<typename Loader>
      __forceinline BSplinePatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
        : type(BSPLINE_PATCH) { data.init(edge,loader); }

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

      template<typename Loader>
      __forceinline GregoryPatch (const SubdivMesh::HalfEdge* edge, const Loader& loader) 
        : type(GREGORY_PATCH) { CatmullClarkPatchT<Vertex,Vertex_t> patch; patch.init2(edge,loader); data.init(patch); }

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
        
        GeneralCatmullClarkPatchT<Vertex,Vertex_t> patch;
        patch.init2(edge,loader);
        FeatureAdaptivePointEval<Vertex,Vertex_t> eval(patch,u,v,P,dPdu,dPdv); 
      }
      
    public:
      Type type;
      const SubdivMesh::HalfEdge* const edge;
      const char* const vertices;
      const size_t stride;
    };
    
    /*struct SubdividedTrianglePatch
    {
      LocalTessellationCacheThreadInfo::CacheEntry patches[3];
    };
    
    struct SubdividedQuadPatch
    {
      LocalTessellationCacheThreadInfo::CacheEntry patches[4];
      };*/
    
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
      default:                               return (Patch*) EvalPatch   ::create(alloc,edge,vertices,stride);
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
