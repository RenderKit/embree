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

  #define MAX_VALENCE 16

  struct __aligned(64) FinalQuad
  {
    Vec3fa vtx[4];
    Vec2f uv[2];
    unsigned int geomID;
    unsigned int primID;
  };

  struct __aligned(64) CatmullClark1Ring
  {
    Vec3fa vtx;
    Vec3fa ring[2*MAX_VALENCE]; // two vertices per face
    unsigned int valence;
    unsigned int num_vtx;

    CatmullClark1Ring() {}
    
    __forceinline void init(const SubdivMesh::HalfEdge *const h,
			    const Vec3fa *const vertices)
    {
      size_t i=0;
      vtx = vertices[ h->getStartVertexIndex() ];
      DBG_PRINT( h->getStartVertexIndex() );
      SubdivMesh::HalfEdge *p = (SubdivMesh::HalfEdge*)h;
      do {
	p = p->opposite();
	ring[i++] = vertices[ p->getStartVertexIndex() ];
	ring[i++] = vertices[ p->prev()->getStartVertexIndex() ];

	DBG_PRINT( p->getStartVertexIndex() );
	DBG_PRINT( p->prev()->getStartVertexIndex() );

	/*! continue with next adjacent edge. */
	p = p->next();
      } while( p != h);
      num_vtx = i;
      valence = i >> 1;
      // copy first vertex to last position
      ring[num_vtx] = ring[0];
      assert( i+1 < MAX_VALENCE );
    }

    __forceinline void CatmullClarkSubdivision(CatmullClark1Ring &dest)
    {
      dest.valence = valence;
      dest.num_vtx = num_vtx;
      Vec3fa avg_faces(0.0f,0.0f,0.0f);
      // new face vtx
      for (size_t i=0;i<valence;i++)
	{
	  const Vec3fa new_face = (vtx + ring[2*i] + ring[2*i+1] + ring[2*i+2]) * 0.25f;
	  dest.ring[2*i + 1] = new_face;
	  avg_faces += new_face;
	}
      // new edge vertices
      Vec3fa avg_edges(0.0f,0.0f,0.0f);
      for (size_t i=1;i<valence;i++)
	{
	  const Vec3fa new_edge = (vtx + ring[2*i] + dest.ring[2*i-1] + dest.ring[2*i+1]) * 0.25f;
	  dest.ring[2*i + 0] = new_edge;
	  avg_edges += new_edge;
	}
      dest.ring[0] = (vtx + ring[0] + dest.ring[num_vtx-1] + dest.ring[1]) * 0.25f;
      dest.ring[num_vtx] = dest.ring[0]; // copy to last position
      avg_edges += dest.ring[0];
      // new vtx
      const float inv_valence = 1.0f / valence;
      avg_faces *= inv_valence;
      avg_edges *= inv_valence;

      dest.vtx = (vtx + 2.0f * avg_edges + (float)(valence-3)*vtx) * inv_valence;
    }

  };

  class RegularCatmullClarkPatch : public RegularCatmullClarkPatchT<Vec3fa> 
  {
  public:
    
  };

  class __aligned(64) IrregularCatmullClarkPatch
  {
  public:
    CatmullClark1Ring ring[4];
  };



  struct SubdivPatch1
  {
  public:

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

    __forceinline void init( IrregularCatmullClarkPatch& patch) const
    {
      for (size_t i=0;i<4;i++)
	patch.ring[0].init(first_half_edge + i,vertices);
    }

    __forceinline void init( FinalQuad& quad ) const
    {
      quad.vtx[0] = getQuadVertex(0);
      quad.vtx[1] = getQuadVertex(1);
      quad.vtx[2] = getQuadVertex(2);
      quad.vtx[3] = getQuadVertex(3);
      // uv[0] = 
      // uv[1] = 
      quad.geomID = geomID;
      quad.primID = primID;
    };

    __forceinline void init( RegularCatmullClarkPatch& cc_patch) const
    {
      DBG_PRINT( first_half_edge->halfedge_id );
      DBG_PRINT( first_half_edge->next()->halfedge_id );
      DBG_PRINT( first_half_edge->next()->next()->halfedge_id );
      DBG_PRINT( first_half_edge->next()->next()->next()->halfedge_id );
      exit(0);

      // quad(0,0)
      const SubdivMesh::HalfEdge *e11 = first_half_edge->half_circle();
      const SubdivMesh::HalfEdge *e10 = e11->next();
      const SubdivMesh::HalfEdge *e00 = e10->next();
      const SubdivMesh::HalfEdge *e01 = e10->next();

      cc_patch.v[1][1] = vertices[e11->getStartVertexIndex()];
      cc_patch.v[1][0] = vertices[e10->getStartVertexIndex()];
      cc_patch.v[0][0] = vertices[e00->getStartVertexIndex()];
      cc_patch.v[0][1] = vertices[e01->getStartVertexIndex()];

      // quad(0,2)
      const SubdivMesh::HalfEdge *e12 = first_half_edge->opposite()->half_circle();
      const SubdivMesh::HalfEdge *e13 = e12->next();
      const SubdivMesh::HalfEdge *e03 = e13->next();
      const SubdivMesh::HalfEdge *e02 = e03->next();

      cc_patch.v[1][2] = vertices[e12->getStartVertexIndex()];
      cc_patch.v[1][3] = vertices[e13->getStartVertexIndex()];
      cc_patch.v[0][3] = vertices[e03->getStartVertexIndex()];
      cc_patch.v[0][2] = vertices[e02->getStartVertexIndex()];

      // quad(2,0)
      const SubdivMesh::HalfEdge *e21 = first_half_edge->prev()->half_circle();
      const SubdivMesh::HalfEdge *e31 = e21->next();
      const SubdivMesh::HalfEdge *e30 = e31->next();
      const SubdivMesh::HalfEdge *e20 = e30->next();

      cc_patch.v[2][1] = vertices[e21->getStartVertexIndex()];
      cc_patch.v[3][1] = vertices[e31->getStartVertexIndex()];
      cc_patch.v[3][0] = vertices[e30->getStartVertexIndex()];
      cc_patch.v[2][0] = vertices[e20->getStartVertexIndex()];

      // quad(2,2)
      const SubdivMesh::HalfEdge *e22 = first_half_edge->next()->opposite()->half_circle();
      const SubdivMesh::HalfEdge *e32 = e22->next();
      const SubdivMesh::HalfEdge *e33 = e32->next();
      const SubdivMesh::HalfEdge *e23 = e33->next();

      cc_patch.v[2][2] = vertices[e22->getStartVertexIndex()];
      cc_patch.v[3][2] = vertices[e32->getStartVertexIndex()];
      cc_patch.v[3][3] = vertices[e33->getStartVertexIndex()];
      cc_patch.v[2][3] = vertices[e23->getStartVertexIndex()];      
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
