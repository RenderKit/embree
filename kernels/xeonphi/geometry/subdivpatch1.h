// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "primitive.h"
#include "common/scene_subdiv_mesh.h"
#include "common/scene_subdivision.h"

namespace embree
{

  class RegularCatmullClarkPatch : public RegularCatmullClarkPatchT<Vec3fa> 
  {
  public:
    
  };


  struct SubdivPatch1
  {
  public:

    static int regular1RingOffsets[8][2];

    /*! Default constructor. */
    __forceinline SubdivPatch1 (const SubdivMesh::HalfEdge * first_half_edge,
				const Vec3fa *vertices,
				unsigned int geomID,
				unsigned int primID,
				size_t flags = 0) 
    : first_half_edge(first_half_edge),
      vertices(vertices),
      geomID(geomID),
      primID(primID),
      flags(flags)
  
    {}

    __forceinline const Vec3fa &getQuadVertex(const unsigned int i=0) const { 
      const SubdivMesh::HalfEdge *const h = first_half_edge + i;
      return vertices[h->vtx_index];
    }

    __forceinline unsigned int getStartVertexIndexFromHalfEdge(const SubdivMesh::HalfEdge * edge) const { 
      return edge->vtx_index;
    }

    __forceinline const Vec3fa &getStartVertexFromHalfEdge(const SubdivMesh::HalfEdge * edge) const { 
      return vertices[ getStartVertexIndexFromHalfEdge(edge) ];
    }

    __forceinline unsigned int getEndVertexIndexFromHalfEdge(const SubdivMesh::HalfEdge * edge) const { 
      SubdivMesh::HalfEdge *h_base = edge->base();
      return edge->next(h_base)->vtx_index;
    }

    __forceinline const Vec3fa &getEndVertexFromHalfEdge(const SubdivMesh::HalfEdge * edge) const { 
      return vertices[ getEndVertexIndexFromHalfEdge(edge) ];
    }

    __forceinline void init( RegularCatmullClarkPatch& cc_patch)
    {
      // quad(1,1)
      cc_patch.v[1][1] = getQuadVertex(0);
      cc_patch.v[1][2] = getQuadVertex(1);
      cc_patch.v[2][2] = getQuadVertex(2);
      cc_patch.v[2][1] = getQuadVertex(3);

    }
   
    const SubdivMesh::HalfEdge * first_half_edge; //!< pointer to first half edge of corresponding quad in the subdivision mesh
    const Vec3fa *vertices;                       //!< pointer to the vertex positions in the subdivison mesh
    size_t flags;
    unsigned int geomID;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int primID;                          //!< primitive ID of this subdivision patch
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1 &p)
    {
      o << "first_half_edge " << p.first_half_edge << " vertices " << p.vertices << " flags " << p.flags << " geomID " << p.geomID << " primID " << p.primID;

      return o;
    } 

  

}
