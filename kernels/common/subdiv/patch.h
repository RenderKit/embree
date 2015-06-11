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
    
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
    typedef BSplinePatchT<Vertex,Vertex_t> BSplinePatch;
    typedef GregoryPatchT<Vertex,Vertex_t> GregoryPatch;
    typedef EvalPatchT<Vertex,Vertex_t> EvalPatch;

    enum Type {
      INVALID_PATCH = 0,
      BSPLINE_PATCH = 1,  
      GREGORY_PATCH = 2,
      EVAL_PATCH = 3
    };

    /*! Default constructor. */
    __forceinline Patch () {}

    __forceinline Patch (const SubdivMesh::HalfEdge* edge, const char* vertices, size_t stride) 
      : type(INVALID_PATCH)
    {
      auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
        const unsigned vtx = p->getStartVertexIndex();
        return Vertex_t::loadu((float*)&vertices[vtx*stride]);
      };

      switch (edge->type) 
      {
      case SubdivMesh::REGULAR_QUAD_PATCH: 
      {
        ((BSplinePatch*)data)->init(edge,loader);
        type = BSPLINE_PATCH;
        break;
      }
  
      case SubdivMesh::IRREGULAR_QUAD_PATCH:
      {
        CatmullClarkPatch patch;
        patch.init2(edge,loader);
        ((GregoryPatch*)data)->init(patch);
        type = GREGORY_PATCH;
        break;
      }
      default: 
      {
        new (data) EvalPatch(edge,vertices,stride);
        type = EVAL_PATCH;
        break;
      }
      }
    }

    __forceinline Vertex eval(const float uu, const float vv) const
    {
      switch (type) {
      case BSPLINE_PATCH: return ((BSplinePatch*)data)->eval(uu,vv);
      case GREGORY_PATCH: return ((GregoryPatch*)data)->eval(uu,vv);
      case EVAL_PATCH   : { Vertex P; return ((EvalPatch*)data)->eval(uu,vv,&P,nullptr,nullptr); return P; }
      }
      return zero;
    }

    __forceinline void tangentUV(const float& uu, const float& vv, Vertex* dPdu, Vertex* dPdv) const
    {
      switch (type) {
      case BSPLINE_PATCH: 
        *dPdu = ((BSplinePatch*)data)->tangentU(uu,vv); 
        *dPdv = ((BSplinePatch*)data)->tangentV(uu,vv); 
        break;
      case GREGORY_PATCH: 
        *dPdu = ((GregoryPatch*)data)->tangentU(uu,vv); 
        *dPdv = ((GregoryPatch*)data)->tangentV(uu,vv); 
        break;
      case EVAL_PATCH   : 
        ((EvalPatch*)data)->eval(uu,vv,nullptr,dPdu,dPdv); 
        break;
      }
    }

    __forceinline void eval(const float& uu, const float& vv, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
    {
      switch (type) {
      case BSPLINE_PATCH: 
        if (P)    *P    = ((BSplinePatch*)data)->eval(uu,vv); 
        if (dPdu) *dPdu = ((BSplinePatch*)data)->tangentU(uu,vv); 
        if (dPdv) *dPdv = ((BSplinePatch*)data)->tangentV(uu,vv); 
        break;
      case GREGORY_PATCH: 
        if (P)    *P    = ((GregoryPatch*)data)->eval(uu,vv); 
        if (dPdu) *dPdu = ((GregoryPatch*)data)->tangentU(uu,vv); 
        if (dPdv) *dPdv = ((GregoryPatch*)data)->tangentV(uu,vv); 
        break;
      case EVAL_PATCH:
        ((EvalPatch*)data)->eval(uu,vv,P,dPdu,dPdv); 
        break;
      }
    }

  public:
    Type type;
    Vertex data[24];
  };
}
