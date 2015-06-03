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
  template<typename Vertex, typename Vertex_t>
    struct __aligned(64) Patch
  {
  public:
    
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;

    enum Type {
      BSPLINE_PATCH = 1,  
      GREGORY_PATCH = 2,
      EVAL_PATCH = 3
    };

    /*! Default constructor. */
    __forceinline Patch () {}

    __forceinline Patch (const SubdivMesh::HalfEdge* edge, const void* vertices, size_t stride) 
      : flags(0)
    {
      auto loader = [&](const SubdivMesh::HalfEdge* p) -> Vertex { 
        const unsigned vtx = p->getStartVertexIndex();
        return Vertex::loadu((float*)&src[vtx*stride]);  // FIXME: reads behind the end of the array
      };

      switch (edge->type) 
      {
      case SubdivMesh::REGULAR_QUAD_PATCH: 
      {
        ((BSplinePatchT<Vertex>*)data)->init(edge,loader);
        type = BSPLINE_PATCH;
        break;
      }
  
      case SubdivMesh::IRREGULAR_QUAD_PATCH:
      {
        CatmullClarkPatchT<Vertex> patch;
        patch.init2(edge,loader);
        ((GregoryPatchT<Vertex>*)data)->init(patch);
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
      case BSPLINE_PATCH: return ((BsplinePatch*)data)->eval(uu,vv);
      case GREGORY_PATCH: return ((GregoryPatch*)data)->eval(uu,vv);
      case EVAL_PATCH   : { Vertex P; return ((EvalPatch*)data)->eval(uu,vv,&p,nullptr,nullptr); return P; }
      }
      return zero;
    }

    __forceinline void tangentUV(const float& uu, const float& vv, Vertex* dPdu, Vertex* dPdv) const
    {
      switch (type) {
      case BSPLINE_PATCH: 
        *dPdu = ((BsplinePatch*)data)->tangentU(uu,vv); 
        *dPdv = ((BsplinePatch*)data)->tangentV(uu,vv); 
        break;
      case GREGORY_PATCH: 
        *dPdu = ((GregoryPatch*)data)->tangentU(uu,vv); 
        *dPdv = ((GregoryPatch*)data)->tangentV(uu,vv); 
        break;
      case EVAL_PATCH   : 
        ((EvalPatch*)data)->eval(uu,vv,nullptr,dPdu,dPdv); 
        break;
      }
      return zero;
    }

    __forceinline void eval(const float& uu, const float& vv, Vertex* P, Vertex* dPdu, Vertex* dPdv) const
    {
      switch (type) {
      case BSPLINE_PATCH: 
        if (P)    *P    = ((BsplinePatch*)data)->eval(uu,vv); 
        if (dPdu) *dPdu = ((BsplinePatch*)data)->tangentU(uu,vv); 
        if (dPdv) *dPdv = ((BsplinePatch*)data)->tangentV(uu,vv); 
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
      return zero;
    }

  public:
    Type type;
    Vertex data[24];
  };
}
