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

#include "common/geometry.h"

// lots of code shared between xeon and xeon phi right now

namespace embree
{

  /* int->float lookup table for single axis of a regular grid, grid dimension would be (2^l+1)*(2^l+1) */

  struct __aligned(64) RegularGridLookUpTables
  {
  private:
    const static size_t MAX_SUBDIVISION_LEVEL = 8;
    const static size_t MAX_TABLE_ENTRIES = 2*(((unsigned int)1 << MAX_SUBDIVISION_LEVEL)+1);
    unsigned int offset[MAX_SUBDIVISION_LEVEL];

    float table[ MAX_TABLE_ENTRIES ];

  public:

    __forceinline float lookUp(const size_t level,
			       const size_t index) const
    {
      assert(level < MAX_SUBDIVISION_LEVEL);
      assert(offset[level]+index < MAX_TABLE_ENTRIES); 
      return table[offset[level]+index];
    }

    RegularGridLookUpTables()
      {
	size_t index = 0;
	for (size_t l=0;l<MAX_SUBDIVISION_LEVEL;l++)
	  {
	    const unsigned int grid_size = (1 << l) + 1;
	    offset[l] = index;
	    for (size_t i=0;i<grid_size;i++)
	      table[index++] = (float)i / (float)(grid_size-1);
	  }	
	assert(index < MAX_TABLE_ENTRIES );

	/* for (size_t l=0;l<MAX_SUBDIVISION_LEVEL;l++) */
	/*   { */
	/*     DBG_PRINT(l); */
	/*     const unsigned int grid_size = (1 << l) + 1; */
	/*     DBG_PRINT(grid_size); */

	/*     for (size_t i=0;i<grid_size;i++) */
	/*       { */
	/* 	DBG_PRINT(i); */
	/* 	DBG_PRINT(lookUp(l,i)); */
	/*       } */
	/*   }	 */
      }
  };


#define MAX_VALENCE 16

  struct __aligned(64) FinalQuad
  {
    Vec3fa vtx[4];
  };

  struct __aligned(64) CatmullClark1Ring
  {

    Vec3fa ring[2*MAX_VALENCE]; // two vertices per face
    float crease_weight[MAX_VALENCE]; // FIXME: move into 4th component of ring entries

    int hard_edge_index;
    Vec3fa vtx;
    unsigned int valence;
    unsigned int num_vtx;
    float vertex_crease_weight;

    __forceinline       Vec3fa& get_ring(int i)       { if (i == 0) return ring[i]; else return ring[num_vtx-i]; }
    __forceinline const Vec3fa& get_ring(int i) const { if (i == 0) return ring[i]; else return ring[num_vtx-i]; }

    __forceinline       float& get_crease_weight(int i)       { if (i == 0) return crease_weight[i]; else return crease_weight[valence-i]; }
    __forceinline const float& get_crease_weight(int i) const { if (i == 0) return crease_weight[i]; else return crease_weight[valence-i]; }

  public:
    CatmullClark1Ring() {}

    __forceinline const Vec3fa& front(size_t i) const {
      assert(num_vtx>i);
      return ring[i];
    }

    __forceinline const Vec3fa& back(size_t i) const {
      assert(i>0 && num_vtx>=i);
      return ring[num_vtx-i];
    }

    __forceinline bool has_last_face() const {
      return hard_edge_index != num_vtx-2;
    }

    __forceinline bool has_second_face() const {
      return (hard_edge_index == -1) || (hard_edge_index >= 4);
    }

    __forceinline BBox3fa bounds() const
    {
      BBox3fa bounds ( vtx );
      for (size_t i = 0; i<num_vtx ; i++)
	bounds.extend( ring[i] );
      return bounds;
    }

    __forceinline void init(const SubdivMesh::HalfEdge* const h, const Vec3fa* const vertices) // FIXME: should get buffer as vertex array input!!!!
    {
      for (size_t i=0; i<MAX_VALENCE; i++) crease_weight[i] = nan; // FIXME: remove

      hard_edge_index = -1;
      vtx = (Vec3fa_t)vertices[ h->getStartVertexIndex() ];
      vertex_crease_weight = h->vertex_crease_weight;
      SubdivMesh::HalfEdge* p = (SubdivMesh::HalfEdge*) h;

      size_t i=0;
      crease_weight[i/2] = p->edge_crease_weight;
      if (!p->hasOpposite()) crease_weight[i/2] = inf;

      do 
      {
        /* store first two vertices of face */
        p = p->next();
        ring[i++] = (Vec3fa_t) vertices[ p->getStartVertexIndex() ];
        p = p->next();
        ring[i++] = (Vec3fa_t) vertices[ p->getStartVertexIndex() ];
        p = p->next();
        crease_weight[i/2] = p->edge_crease_weight;

        /* continue with next face */
        if (likely(p->hasOpposite())) 
          p = p->opposite();
        
        /* if there is no opposite go the long way to the other side of the border */
        else
        {
          /*! mark first border edge and store dummy vertex for face between the two border edges */
          hard_edge_index = i;
          crease_weight[i/2] = inf; 
          ring[i++] = (Vec3fa_t) vertices[ p->getStartVertexIndex() ];
          ring[i++] = vtx; //Vec3fa(nan);
          crease_weight[i/2] = inf;
  
          /*! goto other side of border */
          p = (SubdivMesh::HalfEdge*) h;
          while (p->hasOpposite()) 
            p = p->opposite()->next();
        }

      } while (p != h); 

      num_vtx = i;
      valence = i >> 1;
    }


    __forceinline void update(CatmullClark1Ring& dest) const
    {
      dest.valence         = valence;
      dest.num_vtx         = num_vtx;
      dest.hard_edge_index = hard_edge_index;
      dest.vertex_crease_weight   = max(0.0f,vertex_crease_weight-1.0f);

      /* calculate face points */
      Vec3fa_t S = Vec3fa_t(0.0f);
      for (size_t i=0; i<valence-1; i++) {
        S += dest.ring[2*i+1] = ((vtx + ring[2*i]) + (ring[2*i+1] + ring[2*i+2])) * 0.25f;
      }
      S += dest.ring[num_vtx-1] = ((vtx + ring[num_vtx-2]) + (ring[num_vtx-1] + ring[0])) * 0.25f;
      
      /* calculate new edge points */
      size_t num_creases = 0;
      size_t crease_id[MAX_VALENCE];
      Vec3fa_t C = Vec3fa_t(0.0f);
      for (size_t i=1; i<valence; i++)
      {
        const Vec3fa_t v = vtx + ring[2*i];
        const Vec3fa_t f = dest.ring[2*i-1] + dest.ring[2*i+1];
        S += ring[2*i];
        dest.crease_weight[i] = max(crease_weight[i]-1.0f,0.0f);
        //dest.crease_weight[i] = crease_weight[i] < 1.0f ? 0.0f : 0.5f*crease_weight[i];
        
        /* fast path for regular edge points */
        if (likely(crease_weight[i] <= 0.0f)) {
          dest.ring[2*i] = (v+f) * 0.25f;
        }
        
        /* slower path for hard edge rule */
        else {
          C += ring[2*i]; crease_id[num_creases++] = i;
          dest.ring[2*i] = v*0.5f;

          /* even slower path for blended edge rule */
          if (unlikely(crease_weight[i] < 1.0f)) {
            const float w0 = crease_weight[i], w1 = 1.0f-w0;
            dest.ring[2*i] = w1*((v+f)*0.25f) + w0*(v*0.5f);
          }
        }
      }
      {
        const size_t i=0;
        const Vec3fa_t v = vtx + ring[2*i];
        const Vec3fa_t f = dest.ring[num_vtx-1] + dest.ring[2*i+1];
        S += ring[2*i];
        dest.crease_weight[i] = max(crease_weight[i]-1.0f,0.0f);
        //dest.crease_weight[i] = crease_weight[i] < 1.0f ? 0.0f : 0.5f*crease_weight[i];

        /* fast path for regular edge points */
        if (likely(crease_weight[i] <= 0.0f)) {
          dest.ring[2*i] = (v+f) * 0.25f;
        }
        
        /* slower path for hard edge rule */
        else {
          C += ring[2*i]; crease_id[num_creases++] = i;
          dest.ring[2*i] = v*0.5f;

          /* even slower path for blended edge rule */
          if (unlikely(crease_weight[i] < 1.0f)) {
            const float w0 = crease_weight[i], w1 = 1.0f-w0;
            dest.ring[2*i] = w1*((v+f)*0.25f) + w0*(v*0.5f);
          }
        }
      }

      /* compute new vertex using smooth rule */
      const float inv_valence = 1.0f / (float)valence;
      const Vec3fa_t v_smooth = (Vec3fa_t)(S*inv_valence + (float(valence)-2.0f)*vtx)*inv_valence;
      dest.vtx = v_smooth;

      /* compute new vertex using vertex_crease_weight rule */
      if (unlikely(vertex_crease_weight > 0.0f)) 
      {
        if (vertex_crease_weight >= 1.0f) {
          dest.vtx = vtx;
        } else {
          const float t0 = vertex_crease_weight, t1 = 1.0f-t0;
          dest.vtx = t0*vtx + t1*v_smooth;;
        }
        return;
      }

      if (likely(num_creases <= 1))
        return;
      
      /* compute new vertex using crease rule */
      if (likely(num_creases == 2)) {
        const Vec3fa_t v_sharp = (Vec3fa_t)(C + 6.0f * vtx) * (1.0f / 8.0f);
        const float crease_weight0 = crease_weight[crease_id[0]];
        const float crease_weight1 = crease_weight[crease_id[1]];
        dest.vtx = v_sharp;
        dest.crease_weight[crease_id[0]] = max(0.25f*(3.0f*crease_weight0 + crease_weight1)-1.0f,0.0f);
        dest.crease_weight[crease_id[1]] = max(0.25f*(3.0f*crease_weight1 + crease_weight0)-1.0f,0.0f);
        //dest.crease_weight[crease_id[0]] = max(0.5f*(crease_weight0 + crease_weight1)-1.0f,0.0f);
        //dest.crease_weight[crease_id[1]] = max(0.5f*(crease_weight1 + crease_weight0)-1.0f,0.0f);
        const float t0 = 0.5f*(crease_weight0+crease_weight1), t1 = 1.0f-t0;
        //dest.crease_weight[crease_id[0]] = t0 < 1.0f ? 0.0f : 0.5f*t0;
        //dest.crease_weight[crease_id[1]] = t0 < 1.0f ? 0.0f : 0.5f*t0;
        if (unlikely(t0 < 1.0f)) {
          dest.vtx = t0*v_sharp + t1*v_smooth;
        }
      }
      
      /* compute new vertex using corner rule */
      else {
        dest.vtx = vtx;
      }
    }

    /* returns true if the vertex can be part of B-spline patch */
    __forceinline bool dicable() const 
    {
      if (valence != 4) 
        return false;

      if (vertex_crease_weight > 0.0f)
        return false;
      
      for (size_t i=0; i<valence; i++) {
        if (crease_weight[i] > 0.0f) return false;
      }
      return true;
    }

    /* computes the limit vertex */
    __forceinline Vec3fa getLimitVertex() const
    {
      /* border vertex rule */
      if (unlikely(hard_edge_index != -1))
	{
	  const unsigned int second_hard_edge_index = hard_edge_index+2 >= num_vtx ? 0 : hard_edge_index+2;
	  return (4.0f * vtx + ring[hard_edge_index] + ring[second_hard_edge_index]) * 1.0f/6.0f;
	}

      Vec3fa_t F( 0.0f );
      Vec3fa_t E( 0.0f );

      for (size_t i=0; i<valence; i++) {
        F += ring[2*i+1];
        E += ring[2*i];
      }

      const float n = (float)valence;
      return (Vec3fa_t)(n*n*vtx+4*E+F) / ((n+5.0f)*n);      
    }

    /* gets limit tangent in the direction of egde vtx -> ring[0] */
    __forceinline Vec3fa getLimitTangent() const 
    {
      /* border vertex rule */
      if (unlikely(hard_edge_index != -1))
	{
	  //if (hard_edge_index != 0 && valence != 2) { 
          if (has_last_face() && valence != 2) { // FIXME: probably wrong
	    return ring[0] - vtx; 
          }
	  else
	    {
              const unsigned int second_hard_edge_index = hard_edge_index+2 >= num_vtx ? 0 : hard_edge_index+2;
	      assert(second_hard_edge_index < num_vtx);
	      return (ring[second_hard_edge_index] - ring[hard_edge_index]) * 0.5f;
	    }
	}

      Vec3fa_t alpha( 0.0f );
      Vec3fa_t beta ( 0.0f );

      const float n = (float)valence;
      const float c0 = 1.0f/n * 1.0f / sqrtf(4.0f + cos(M_PI/n)*cos(M_PI/n));  
      const float c1 = (1.0f/n + cosf(M_PI/n) * c0); // FIXME: plus or minus
      for (size_t i=0; i<valence; i++)
	{
	  const float a = c1 * cosf(2.0f*M_PI*i/n);
	  const float b = c0 * cosf((2.0f*M_PI*i+M_PI)/n);
	  alpha +=  a * ring[2*i];
          beta  +=  b * ring[2*i+1];
	}
      return alpha +  beta;      // FIXME: scaling inconsistent with border case
    }

    /* gets limit tangent in the direction of egde vtx -> ring[num_vtx-2] */
    __forceinline Vec3fa getSecondLimitTangent() const 
    {
      /* border vertex rule */
      if (unlikely(hard_edge_index != -1))
	{
	  //if (hard_edge_index == 0 && valence != 2) {
	  if (!has_last_face() && valence != 2) {
	    return get_ring(num_vtx-2) - vtx;
          }
	  else
	    {
              const unsigned int second_hard_edge_index = hard_edge_index+2 >= num_vtx ? 0 : hard_edge_index+2;
	      return (ring[hard_edge_index] - ring[second_hard_edge_index]) * 0.5f;
	    }
	}

      Vec3fa_t alpha( 0.0f );
      Vec3fa_t beta ( 0.0f );
      const float n = (float)valence;
      const float c0 = 1.0f/n * 1.0f / sqrtf(4.0f + cos(M_PI/n)*cos(M_PI/n));  
      const float c1 = (1.0f/n + cosf(M_PI/n) * c0);
      for (size_t i=0; i<valence; i++)
	{
	  const float a = c1 * cosf(2.0f*M_PI*(float(i)-1.0f)/n);
	  const float b = c0 * cosf((2.0f*M_PI*(float(i)-1.0f)+M_PI)/n);
	  alpha += a * ring[2*i];
          beta  += b * ring[2*i+1];
	}
      return alpha +  beta;      
    }

    /* returns center of the n-th quad in the 1-ring */
    __forceinline Vec3fa getQuadCenter(const size_t index) const
    {
      const Vec3fa_t &p0 = vtx;
      const Vec3fa_t &p1 = ring[2*index+0];
      const Vec3fa_t &p2 = ring[2*index+1];
      const Vec3fa_t &p3 = index == valence-1 ? ring[0] : ring[2*index+2];
      const Vec3fa p = (p0+p1+p2+p3) * 0.25f;
      return p;
    }

    /* returns center of the n-th edge in the 1-ring */
    __forceinline Vec3fa getEdgeCenter(const size_t index) const {
      return (vtx + ring[index*2]) * 0.5f;
    }

    friend __forceinline std::ostream &operator<<(std::ostream &o, const CatmullClark1Ring &c)
    {
      o << "vtx " << c.vtx << " size = " << c.num_vtx << ", hard_edge = " << c.hard_edge_index << ", valence " << c.valence << ", ring: " << std::endl;
      for (size_t i=0; i<c.num_vtx; i++) {
        o << i << " -> " << c.ring[i];
        if (i % 2 == 0) o << " crease = " << c.crease_weight[i/2];
        o << std::endl;
      }
      return o;
    } 
  };

  struct __aligned(64) GeneralCatmullClark1Ring
  {
    Vec3fa vtx;
    Vec3fa ring[2*MAX_VALENCE]; 
    int face_size[MAX_VALENCE];       // number of vertices-2 of nth face in ring
    float crease_weight[MAX_VALENCE]; // FIXME: move into 4th component of ring entries
    unsigned int valence;
    unsigned int num_vtx;
    int hard_edge_face;
    float vertex_crease_weight;

    GeneralCatmullClark1Ring() {}

    void flip() 
    {
      Vec3fa ring_o[2*MAX_VALENCE]; // two vertices per face
      int face_size_o[MAX_VALENCE];       // number of vertices-2 of nth face in ring
      float crease_weight_o[MAX_VALENCE]; // FIXME: move into 4th component of ring entries
      for (size_t i=0; i<valence; i++) face_size_o[i] = face_size[i];
      for (size_t i=0; i<valence; i++) crease_weight_o[i] = crease_weight[i];
      for (size_t i=0; i<num_vtx; i++) ring_o[i] = ring[i];

      for (size_t i=0; i<valence; i++) face_size[i] = face_size_o[valence-1-i];
      for (size_t i=1; i<valence; i++) crease_weight[i] = crease_weight_o[valence-i];
      for (size_t i=1; i<num_vtx; i++) ring[i] = ring_o[num_vtx-i];
      if (hard_edge_face != -1)
        hard_edge_face = valence-1-hard_edge_face;
    }

    __forceinline bool has_last_face() const {
      //return hard_edge_face != 0;
      return hard_edge_face != valence-1;
    }

    __forceinline bool has_second_face() const {
      return (hard_edge_face == -1) || (hard_edge_face >= 2);
    }

    __forceinline void init(const SubdivMesh::HalfEdge* const h, const Vec3fa* const vertices) // FIXME: should get buffer as vertex array input!!!!
    {
      for (size_t i=0; i<MAX_VALENCE; i++) crease_weight[i] = nan; // FIXME: remove

      hard_edge_face = -1;
      vtx = (Vec3fa_t)vertices[ h->getStartVertexIndex() ];
      vertex_crease_weight = h->vertex_crease_weight;
      SubdivMesh::HalfEdge* p = (SubdivMesh::HalfEdge*) h;

      size_t e=0, f=0;
      do {
        assert(f < MAX_VALENCE);
        crease_weight[f] = p->edge_crease_weight;

        size_t vn = 0;
        assert(e < 2*MAX_VALENCE);
	ring[e++] = (Vec3fa_t) vertices[ p->getEndVertexIndex() ];
        vn++;
        
	if (unlikely(!p->hasOpposite())) { 
          init_secondhalf(h,vertices,e,f);
          return;
        }
	p = p->opposite();

        SubdivMesh::HalfEdge* pnext2 = p->next()->next();
        for (SubdivMesh::HalfEdge* v = p->prev(); v!=pnext2; v=v->prev()) {
          assert(e < 2*MAX_VALENCE);
          ring[e++] = (Vec3fa_t) vertices[ v->getStartVertexIndex() ];
          vn++;
        }
        face_size[f++] = vn;
        p = p->next();
        
      } while (p != h);

      num_vtx = e;
      valence = e >> 1;

      flip();
    }

    __forceinline void init_secondhalf(const SubdivMesh::HalfEdge* const h, const Vec3fa* const vertices, size_t e, size_t f)
    {
      /*! mark first hard edge and store dummy vertex for face between the two hard edges */
      hard_edge_face = f;
      assert(e < 2*MAX_VALENCE);
      ring[e++] = vtx; //Vec3fa_t(nan);
      crease_weight[f] = inf; 
      face_size[f++] = 2;
      
      /*! first cycle clock-wise until we found the second edge */	  
      SubdivMesh::HalfEdge* p = (SubdivMesh::HalfEdge*) h;
      p = p->prev();	  
      while(p->hasOpposite()) {
        p = p->opposite();
        p = p->prev();	      
      }
      
      /*! store second hard edge and diagonal vertex */
      assert(f < MAX_VALENCE);
      crease_weight[f] = inf;
      
      size_t vn = 0;
      SubdivMesh::HalfEdge* pnext2 = p->next()->next();
      for (SubdivMesh::HalfEdge* v = p; v!=pnext2; v=v->prev()) {
        assert(e < 2*MAX_VALENCE);
        ring[e++] = (Vec3fa_t) vertices[ v->getStartVertexIndex() ];
        vn++;
      }
      face_size[f++] = vn;
      p = p->next();
      
      /*! continue counter-clockwise */	  
      while (p != h) 
      {
	p = p->opposite();

        assert(f < MAX_VALENCE);
        crease_weight[f] = p->edge_crease_weight;

        size_t vn = 0;
        SubdivMesh::HalfEdge* pnext2 = p->next()->next();
        for (SubdivMesh::HalfEdge* v = p; v!=pnext2; v=v->prev()) {
          assert(e < 2*MAX_VALENCE);
          ring[e++] = (Vec3fa_t) vertices[ v->getStartVertexIndex() ];
          vn++;
        }
        face_size[f++] = vn;
        p = p->next();
      }

      num_vtx = e;
      valence = f;

      flip();
    }

    __forceinline void update(CatmullClark1Ring& dest) const
    {
      dest.valence         = valence;
      dest.num_vtx         = 2*valence;
      dest.hard_edge_index = hard_edge_face == -1 ? -1 : 2*hard_edge_face; // FIXME:
      dest.vertex_crease_weight   = max(0.0f,vertex_crease_weight-1.0f);

      /* calculate face points */
      Vec3fa_t S = Vec3fa_t(0.0f);
      for (size_t f=0, v=0; f<valence; v+=face_size[f++]) {
        Vec3fa_t F = vtx;
        for (size_t k=v; k<=v+face_size[f]; k++) F += ring[k%num_vtx]; // FIXME: optimize
        S += dest.ring[2*f+1] = F/float(face_size[f]+2);
      }
      
      /* calculate new edge points */
      size_t num_creases = 0;
      size_t crease_id[MAX_VALENCE];
      Vec3fa_t C = Vec3fa_t(0.0f);
      for (size_t i=0, j=0; i<valence; j+=face_size[i++])
      {
        const Vec3fa_t v = vtx + ring[j];
        Vec3fa_t f = dest.ring[2*i+1];
        if (i == 0) f += dest.ring[dest.num_vtx-1]; 
        else        f += dest.ring[2*i-1];
        S += ring[j];
        dest.crease_weight[i] = max(crease_weight[i]-1.0f,0.0f);
        
        /* fast path for regular edge points */
        if (likely(crease_weight[i] <= 0.0f)) {
          dest.ring[2*i] = (v+f) * 0.25f;
        }
        
        /* slower path for hard edge rule */
        else {
          C += ring[j]; crease_id[num_creases++] = i;
          dest.ring[2*i] = v*0.5f;

          /* even slower path for blended edge rule */
          if (unlikely(crease_weight[i] < 1.0f)) {
            const float w0 = crease_weight[i], w1 = 1.0f-w0;
            dest.ring[2*i] = w1*((v+f)*0.25f) + w0*(v*0.5f);
          }
        }
      }

      /* compute new vertex using smooth rule */
      const float inv_valence = 1.0f / (float)valence;
      const Vec3fa_t v_smooth = (Vec3fa_t)(S*inv_valence + (float(valence)-2.0f)*vtx)*inv_valence;
      dest.vtx = v_smooth;

      /* compute new vertex using vertex_crease_weight rule */
      if (unlikely(vertex_crease_weight > 0.0f)) 
      {
        if (vertex_crease_weight >= 1.0f) {
          dest.vtx = vtx;
        } else {
          const float t0 = vertex_crease_weight, t1 = 1.0f-t0;
          dest.vtx = t0*vtx + t1*v_smooth;;
        }
        return;
      }

      if (likely(num_creases <= 1))
        return;
      
      /* compute new vertex using crease rule */
      if (likely(num_creases == 2)) {
        const Vec3fa_t v_sharp = (Vec3fa_t)(C + 6.0f * vtx) * (1.0f / 8.0f);
        const float crease_weight0 = crease_weight[crease_id[0]];
        const float crease_weight1 = crease_weight[crease_id[1]];
        dest.vtx = v_sharp;
        dest.crease_weight[crease_id[0]] = max(0.25f*(3.0f*crease_weight0 + crease_weight1)-1.0f,0.0f);
        dest.crease_weight[crease_id[1]] = max(0.25f*(3.0f*crease_weight1 + crease_weight0)-1.0f,0.0f);
        //dest.crease_weight[crease_id[0]] = max(0.5f*(crease_weight0 + crease_weight1)-1.0f,0.0f);
        //dest.crease_weight[crease_id[1]] = max(0.5f*(crease_weight1 + crease_weight0)-1.0f,0.0f);
        const float t0 = 0.5f*(crease_weight0+crease_weight1), t1 = 1.0f-t0;
        //dest.crease_weight[crease_id[0]] = t0 < 1.0f ? 0.0f : 0.5f*t0;
        //dest.crease_weight[crease_id[1]] = t0 < 1.0f ? 0.0f : 0.5f*t0;
        if (unlikely(t0 < 1.0f)) {
          dest.vtx = t0*v_sharp + t1*v_smooth;
        }
      }
      
      /* compute new vertex using corner rule */
      else {
        dest.vtx = vtx;
      }
    }

    friend __forceinline std::ostream &operator<<(std::ostream &o, const GeneralCatmullClark1Ring &c)
    {
      o << "vtx " << c.vtx << " size = " << c.num_vtx << ", hard_edge_face = " << c.hard_edge_face << ", " << " valence = " << c.valence << ", ring: " << std::endl;
      for (size_t v=0, f=0; f<c.valence; v+=c.face_size[f++]) {
        for (size_t i=v; i<v+c.face_size[f]; i++) {
          o << i << " -> " << c.ring[i];
          if (i == v) o << " crease = " << c.crease_weight[f];
          o << std::endl;
        }
      }
      return o;
    } 
  };
  
  class __aligned(64) IrregularCatmullClarkPatch
  {
  public:
    CatmullClark1Ring ring[4];
    float level[4];

    __forceinline IrregularCatmullClarkPatch () {}

    __forceinline IrregularCatmullClarkPatch (const SubdivMesh::HalfEdge* first_half_edge, const Vec3fa* vertices) 
    {
      for (size_t i=0; i<4; i++) {
        ring[i].init(first_half_edge+i,vertices);
        level[i] = first_half_edge[i].edge_level;
      }
    }

    __forceinline Vec3fa getLimitVertex(const size_t index) const {
      return ring[index].getLimitVertex();
    }

    __forceinline Vec3fa getLimitTangent(const size_t index) const {
      return ring[index].getLimitTangent();
    }

    __forceinline Vec3fa getSecondLimitTangent(const size_t index) const {
      return ring[index].getSecondLimitTangent();
    }

    __forceinline BBox3fa bounds() const
    {
      BBox3fa bounds (ring[0].bounds());
      for (size_t i=1; i<4; i++)
	bounds.extend(ring[i].bounds());
      return bounds;
    }

    /* returns true if the patch is a B-spline patch */
    __forceinline bool dicable() const {
      return ring[0].dicable() && ring[1].dicable() && ring[2].dicable() && ring[3].dicable();
    }

    static __forceinline void init_regular(const CatmullClark1Ring& p0,
					   const CatmullClark1Ring& p1,
					   CatmullClark1Ring& dest0,
					   CatmullClark1Ring& dest1) 
    {
      dest1.valence = dest0.valence = 4;
      dest1.num_vtx = dest0.num_vtx = 8;
      dest1.hard_edge_index = dest0.hard_edge_index = -1;
      dest1.vtx = dest0.vtx = (Vec3fa_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;

      dest1.ring[2] = dest0.ring[0] = (Vec3fa_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[7] = (Vec3fa_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[6] = (Vec3fa_t)p1.vtx;
      dest1.ring[7] = dest0.ring[5] = (Vec3fa_t)p1.ring[4];
      dest1.ring[6] = dest0.ring[4] = (Vec3fa_t)p0.ring[p0.num_vtx-1];
      dest1.ring[5] = dest0.ring[3] = (Vec3fa_t)p0.ring[p0.num_vtx-2];
      dest1.ring[4] = dest0.ring[2] = (Vec3fa_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vec3fa_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[3] = p1.crease_weight[1];
      dest1.crease_weight[3] = dest0.crease_weight[2] = 0.0f;
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];
    }


    static __forceinline void init_border(const CatmullClark1Ring &p0,
                                          const CatmullClark1Ring &p1,
                                          CatmullClark1Ring &dest0,
                                          CatmullClark1Ring &dest1) 
    {
      dest1.valence = dest0.valence = 3;
      dest1.num_vtx = dest0.num_vtx = 6;
      dest0.hard_edge_index = 2;
      dest1.hard_edge_index = 4;
      dest1.vtx  = dest0.vtx = (Vec3fa_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;

      dest1.ring[2] = dest0.ring[0] = (Vec3fa_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[5] = (Vec3fa_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[4] = (Vec3fa_t)p1.vtx;
      dest1.ring[5] = dest0.ring[3] = (Vec3fa_t)p0.ring[p0.hard_edge_index+1]; // dummy
      dest1.ring[4] = dest0.ring[2] = (Vec3fa_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vec3fa_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[2] = p1.crease_weight[1];
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];
    }

    static __forceinline void init_regular(const Vec3fa_t &center, const Vec3fa_t center_ring[8], const size_t offset, CatmullClark1Ring &dest)
    {
      dest.valence = 4;
      dest.num_vtx = 8;
      dest.hard_edge_index = -1;
      dest.vtx     = (Vec3fa_t)center;
      dest.vertex_crease_weight = 0.0f;
      for (size_t i=0; i<8; i++) 
	dest.ring[i] = (Vec3fa_t)center_ring[(offset+i)%8];
      for (size_t i=0; i<4; i++) 
        dest.crease_weight[i] = 0.0f;
    }
 

    __forceinline void subdivide(IrregularCatmullClarkPatch patch[4]) const
    {
      ring[0].update(patch[0].ring[0]);
      ring[1].update(patch[1].ring[1]);
      ring[2].update(patch[2].ring[2]);
      ring[3].update(patch[3].ring[3]);

      patch[0].level[0] = 0.5f*level[0];
      patch[0].level[1] = 0.25f*(level[1]+level[3]);
      patch[0].level[2] = 0.25f*(level[0]+level[2]);
      patch[0].level[3] = 0.5f*level[3];

      patch[1].level[0] = 0.5f*level[0];
      patch[1].level[1] = 0.5f*level[1];
      patch[1].level[2] = 0.25f*(level[0]+level[2]);
      patch[1].level[3] = 0.25f*(level[1]+level[3]);

      patch[2].level[0] = 0.25f*(level[0]+level[2]);
      patch[2].level[1] = 0.5f*level[1];
      patch[2].level[2] = 0.5f*level[2];
      patch[2].level[3] = 0.25f*(level[1]+level[3]);

      patch[3].level[0] = 0.25f*(level[0]+level[2]);
      patch[3].level[1] = 0.25f*(level[1]+level[3]);
      patch[3].level[2] = 0.5f*level[2];
      patch[3].level[3] = 0.5f*level[3];
      
      if (likely(ring[0].has_last_face()))
        init_regular(patch[0].ring[0],patch[1].ring[1],patch[0].ring[1],patch[1].ring[0]);
      else
        init_border(patch[0].ring[0],patch[1].ring[1],patch[0].ring[1],patch[1].ring[0]);

      if (likely(ring[1].has_last_face()))
        init_regular(patch[1].ring[1],patch[2].ring[2],patch[1].ring[2],patch[2].ring[1]);
      else
        init_border(patch[1].ring[1],patch[2].ring[2],patch[1].ring[2],patch[2].ring[1]);

      if (likely(ring[2].has_last_face()))
        init_regular(patch[2].ring[2],patch[3].ring[3],patch[2].ring[3],patch[3].ring[2]);
      else
        init_border(patch[2].ring[2],patch[3].ring[3],patch[2].ring[3],patch[3].ring[2]);

      if (likely(ring[3].has_last_face()))
        init_regular(patch[3].ring[3],patch[0].ring[0],patch[3].ring[0],patch[0].ring[3]);
      else
        init_border(patch[3].ring[3],patch[0].ring[0],patch[3].ring[0],patch[0].ring[3]);

      Vec3fa_t center = (ring[0].vtx + ring[1].vtx + ring[2].vtx + ring[3].vtx) * 0.25f;
      Vec3fa_t center_ring[8];

      // counter-clockwise
      center_ring[0] = (Vec3fa_t)patch[3].ring[3].ring[0];
      center_ring[7] = (Vec3fa_t)patch[3].ring[3].vtx;
      center_ring[6] = (Vec3fa_t)patch[2].ring[2].ring[0];
      center_ring[5] = (Vec3fa_t)patch[2].ring[2].vtx;
      center_ring[4] = (Vec3fa_t)patch[1].ring[1].ring[0];
      center_ring[3] = (Vec3fa_t)patch[1].ring[1].vtx;
      center_ring[2] = (Vec3fa_t)patch[0].ring[0].ring[0];
      center_ring[1] = (Vec3fa_t)patch[0].ring[0].vtx;

      init_regular(center,center_ring,0,patch[0].ring[2]);
      init_regular(center,center_ring,2,patch[1].ring[3]);
      init_regular(center,center_ring,4,patch[2].ring[0]);
      init_regular(center,center_ring,6,patch[3].ring[1]);
    }

    __forceinline void init( FinalQuad& quad ) const
    {
      quad.vtx[0] = (Vec3fa_t)ring[0].vtx;
      quad.vtx[1] = (Vec3fa_t)ring[1].vtx;
      quad.vtx[2] = (Vec3fa_t)ring[2].vtx;
      quad.vtx[3] = (Vec3fa_t)ring[3].vtx;
    };

    friend __forceinline std::ostream &operator<<(std::ostream &o, const IrregularCatmullClarkPatch &p)
    {
      o << "IrregularCatmullClarkPatch { " << std::endl;
      for (size_t i=0; i<4; i++)
	o << "level" << i << ": " << p.level[i] << std::endl;
      for (size_t i=0; i<4; i++)
	o << "ring" << i << ": " << p.ring[i] << std::endl;
      o << "}" << std::endl;
      return o;
    }
  };



  class __aligned(64) GeneralIrregularCatmullClarkPatch
  {
  public:
    enum { SIZE = 10 };
    GeneralCatmullClark1Ring ring[SIZE];
    float level[SIZE];
    size_t N;

    __forceinline GeneralIrregularCatmullClarkPatch () 
      : N(0) {}

    __forceinline GeneralIrregularCatmullClarkPatch (const SubdivMesh::HalfEdge* h, const Vec3fa* vertices) 
    {
      size_t i = 0;
      const SubdivMesh::HalfEdge* edge = h; 
      do {
        ring[i].init(edge,vertices);
        level[i] = edge->edge_level;
        edge = edge->next();
        i++;
      } while ((edge != h) && (i < SIZE));
      N = i;
    }

    static __forceinline void init_regular(const CatmullClark1Ring& p0,
					   const CatmullClark1Ring& p1,
					   CatmullClark1Ring& dest0,
					   CatmullClark1Ring& dest1) 
    {
      dest1.valence = dest0.valence = 4;
      dest1.num_vtx = dest0.num_vtx = 8;
      dest1.hard_edge_index = dest0.hard_edge_index = -1;
      dest1.vtx = dest0.vtx = (Vec3fa_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;

      dest1.ring[2] = dest0.ring[0] = (Vec3fa_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[7] = (Vec3fa_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[6] = (Vec3fa_t)p1.vtx;
      dest1.ring[7] = dest0.ring[5] = (Vec3fa_t)p1.ring[4];
      dest1.ring[6] = dest0.ring[4] = (Vec3fa_t)p0.ring[p0.num_vtx-1];
      dest1.ring[5] = dest0.ring[3] = (Vec3fa_t)p0.ring[p0.num_vtx-2];
      dest1.ring[4] = dest0.ring[2] = (Vec3fa_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vec3fa_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[3] = p1.crease_weight[1];
      dest1.crease_weight[3] = dest0.crease_weight[2] = 0.0f;
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];
    }


    static __forceinline void init_border(const CatmullClark1Ring &p0,
                                          const CatmullClark1Ring &p1,
                                          CatmullClark1Ring &dest0,
                                          CatmullClark1Ring &dest1) 
    {
      dest1.valence = dest0.valence = 3;
      dest1.num_vtx = dest0.num_vtx = 6;
      dest0.hard_edge_index = 2;
      dest1.hard_edge_index = 4;
      dest1.vtx  = dest0.vtx = (Vec3fa_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;

      dest1.ring[2] = dest0.ring[0] = (Vec3fa_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[5] = (Vec3fa_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[4] = (Vec3fa_t)p1.vtx;
      dest1.ring[5] = dest0.ring[3] = (Vec3fa_t)p0.ring[p0.hard_edge_index+1]; // dummy
      dest1.ring[4] = dest0.ring[2] = (Vec3fa_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vec3fa_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[2] = p1.crease_weight[1];
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];
    }

    static __forceinline void init_regular(const Vec3fa_t &center, const Vec3fa_t center_ring[2*SIZE], const size_t N, const size_t offset, CatmullClark1Ring &dest)
    {
      assert(N<MAX_VALENCE);
      dest.valence = N;
      dest.num_vtx = 2*N;
      dest.hard_edge_index = -1;
      dest.vtx     = (Vec3fa_t)center;
      dest.vertex_crease_weight = 0.0f;
      for (size_t i=0; i<2*N; i++) 
	dest.ring[i] = (Vec3fa_t)center_ring[(2*N+offset+i-1)%(2*N)];
      for (size_t i=0; i<N; i++) 
        dest.crease_weight[i] = 0.0f;
    }
 
    __forceinline void subdivide(IrregularCatmullClarkPatch patch[SIZE], size_t& N_o) const
    {
      N_o = N;

      for (size_t i=0; i<N; i++) {
        size_t ip1 = (i+1)%N; // FIXME: %
        ring[i].update(patch[i].ring[0]);
        patch[i]  .level[0] = 0.5f*level[i];
        patch[ip1].level[3] = 0.5f*level[i];
      }

      Vec3fa_t center = Vec3fa_t(0.0f);
      Vec3fa_t center_ring[2*SIZE];

      for (size_t i=0; i<N; i++)
      {
        size_t ip1 = (i+1)%N; // FIXME: %
        size_t im1 = (i+N-1)%N; // FIXME: %
        if (likely(ring[i].has_last_face())) init_regular(patch[i].ring[0],patch[ip1].ring[0],patch[i].ring[1],patch[ip1].ring[3]); 
        else                                   init_border (patch[i].ring[0],patch[ip1].ring[0],patch[i].ring[1],patch[ip1].ring[3]);

        patch[i].level[1] = patch[ip1].level[2] = 0.25f*(level[im1]+level[ip1]);

        center += ring[i].vtx;
        center_ring[2*i+0] = (Vec3fa_t)patch[i].ring[0].vtx;
        center_ring[2*i+1] = (Vec3fa_t)patch[i].ring[0].ring[0];
      }
      center /= float(N);

      for (size_t i=0; i<N; i++) {
        init_regular(center,center_ring,N,2*i,patch[i].ring[2]);
      }
    }

    friend __forceinline std::ostream &operator<<(std::ostream &o, const GeneralIrregularCatmullClarkPatch &p)
    {
      o << "GeneralIrregularCatmullClarkPatch { " << std::endl;
      for (size_t i=0; i<p.N; i++)
	o << "level" << i << ": " << p.level[i] << std::endl;
      for (size_t i=0; i<p.N; i++)
	o << "ring" << i << ": " << p.ring[i] << std::endl;
      o << "}" << std::endl;
      return o;
    }
  };




  class CubicBSplineCurve
  {
  public:

    static __forceinline Vec4f eval(const float u)
    {
      // FIXME: lookup
      const float t  = u;
      const float s  = 1.0f - u;
      const float n0 = s*s*s;
      const float n1 = (4.0f*s*s*s+t*t*t) + (12.0f*s*t*s + 6.0*t*s*t);
      const float n2 = (4.0f*t*t*t+s*s*s) + (12.0f*t*s*t + 6.0*s*t*s);
      const float n3 = t*t*t;
      //const float c  = 1.0f/6.0f; // do this later
      return Vec4f(n0,n1,n2,n3);
    }

#if defined(__MIC__)

    static __forceinline mic4f eval(const mic_f u)
    {
      // FIXME: lookup
      const mic_f t  = u;
      const mic_f s  = 1.0f - u;
      const mic_f n0 = s*s*s;
      const mic_f n1 = (4.0f*s*s*s+t*t*t) + (12.0f*s*t*s + 6.0*t*s*t);
      const mic_f n2 = (4.0f*t*t*t+s*s*s) + (12.0f*t*s*t + 6.0*s*t*s);
      const mic_f n3 = t*t*t;
      //const mic_f c  = 1.0f/6.0f; // do this later
      return mic4f(n0,n1,n2,n3);
    }


#endif
    
  };

  class CubicBezierCurve
  {
  public:

    static __forceinline Vec3fa_t eval(const float u, const Vec3fa_t &p0, const Vec3fa_t &p1, const Vec3fa_t &p2, const Vec3fa_t &p3)
    {
      // FIXME: lookup
      const float t  = u;
      const float s  = 1.0f - u;
      const float n0 = s*s*s;
      const float n1 = 3.0f*t*s*t;
      const float n2 = 3.0f*s*t*s;
      const float n3 = t*t*t;
      const Vec3fa_t n = p0 * n0 + p1 * n1 + p2 * n2 + p3 * n3;
      return n;
    }
    
  };




  template<typename T>
    class RegularCatmullClarkPatchT
    {
    public:
      T v[4][4];

      __forceinline T computeFaceVertex(const unsigned int y,const unsigned int x) const
      {
	return (v[y][x] + v[y][x+1] + v[y+1][x+1] + v[y+1][x]) * 0.25f;
      }

      __forceinline T computeQuadVertex(const unsigned int y,
					const unsigned int x,
					const T face[3][3]) const
      {
	const T P = v[y][x]; 
	const T Q = face[y-1][x-1] + face[y-1][x] + face[y][x] + face[y][x-1];
	const T R = v[y-1][x] + v[y+1][x] + v[y][x-1] + v[y][x+1];
	const T res = (Q + R) * 0.0625f + P * 0.5f;
	return res;
      }

      __forceinline T computeLimitVertex(const int y,
					 const int x) const
      {
	const T P = v[y][x];
	const T Q = v[y-1][x-1] + v[y-1][x+1] + v[y+1][x-1] + v[y+1][x+1];
	const T R = v[y-1][x] + v[y+1][x] + v[y][x-1] + v[y][x+1];
	const T res = (P * 16.0f + R * 4.0f + Q) * 1.0f / 36.0f;
	return res;
      }

      __forceinline T computeLimitTangentX(const int y,
                                           const int x) const
      {
	/* --- tangent X --- */
	const T Qx = v[y-1][x+1] - v[y-1][x-1] + v[y+1][x+1] - v[y+1][x-1];
	const T Rx = v[y][x-1] - v[y][x+1];
	const T tangentX = (Rx * 4.0f + Qx) * 1.0f / 12.0f;
        return tangentX;
      };

      __forceinline T computeLimitTangentY(const int y,
                                           const int x) const
      {
	const T Qy = v[y-1][x-1] - v[y+1][x-1] + v[y-1][x+1] - v[y+1][x+1];
	const T Ry = v[y-1][x] - v[y+1][x];
	const T tangentY = (Ry * 4.0f + Qy) * 1.0f / 12.0f;
    
	return tangentY;
      }

      __forceinline T computeLimitNormal(const int y,
					 const int x) const
      {
	/* --- tangent X --- */
	const T Qx = v[y-1][x+1] - v[y-1][x-1] + v[y+1][x+1] - v[y+1][x-1];
	const T Rx = v[y][x-1] - v[y][x+1];
	const T tangentX = (Rx * 4.0f + Qx) * 1.0f / 12.0f;

	/* --- tangent Y --- */
	const T Qy = v[y-1][x-1] - v[y+1][x-1] + v[y-1][x+1] - v[y+1][x+1];
	const T Ry = v[y-1][x] - v[y+1][x];
	const T tangentY = (Ry * 4.0f + Qy) * 1.0f / 12.0f;
    
	return cross(tangentX,tangentY);
      }

      __forceinline void initSubPatches(const T edge[12],
					const T face[3][3],
					const T newQuadVertex[2][2],
					RegularCatmullClarkPatchT child[4]) const
      {
	RegularCatmullClarkPatchT &subTL = child[0];
	RegularCatmullClarkPatchT &subTR = child[1];
	RegularCatmullClarkPatchT &subBR = child[2];
	RegularCatmullClarkPatchT &subBL = child[3];

	// top-left
	subTL.v[0][0] = face[0][0];
	subTL.v[0][1] = edge[0];
	subTL.v[0][2] = face[0][1];
	subTL.v[0][3] = edge[1];

	subTL.v[1][0] = edge[2];
	subTL.v[1][1] = newQuadVertex[0][0];
	subTL.v[1][2] = edge[3];
	subTL.v[1][3] = newQuadVertex[0][1];

	subTL.v[2][0] = face[1][0];
	subTL.v[2][1] = edge[5];
	subTL.v[2][2] = face[1][1];
	subTL.v[2][3] = edge[6];

	subTL.v[3][0] = edge[7];
	subTL.v[3][1] = newQuadVertex[1][0];
	subTL.v[3][2] = edge[8];
	subTL.v[3][3] = newQuadVertex[1][1];

	// top-right
	subTR.v[0][0] = edge[0];
	subTR.v[0][1] = face[0][1];
	subTR.v[0][2] = edge[1];
	subTR.v[0][3] = face[0][2];

	subTR.v[1][0] = newQuadVertex[0][0];
	subTR.v[1][1] = edge[3];
	subTR.v[1][2] = newQuadVertex[0][1];
	subTR.v[1][3] = edge[4];

	subTR.v[2][0] = edge[5];
	subTR.v[2][1] = face[1][1];
	subTR.v[2][2] = edge[6];
	subTR.v[2][3] = face[1][2];

	subTR.v[3][0] = newQuadVertex[1][0];
	subTR.v[3][1] = edge[8];
	subTR.v[3][2] = newQuadVertex[1][1];
	subTR.v[3][3] = edge[9];

	// buttom-right
	subBR.v[0][0] = newQuadVertex[0][0];
	subBR.v[0][1] = edge[3];
	subBR.v[0][2] = newQuadVertex[0][1];
	subBR.v[0][3] = edge[4];

	subBR.v[1][0] = edge[5];
	subBR.v[1][1] = face[1][1];
	subBR.v[1][2] = edge[6];
	subBR.v[1][3] = face[1][2];

	subBR.v[2][0] = newQuadVertex[1][0];
	subBR.v[2][1] = edge[8];
	subBR.v[2][2] = newQuadVertex[1][1];
	subBR.v[2][3] = edge[9];

	subBR.v[3][0] = edge[10];
	subBR.v[3][1] = face[2][1];
	subBR.v[3][2] = edge[11];
	subBR.v[3][3] = face[2][2];

	// buttom-left
	subBL.v[0][0] = edge[2];
	subBL.v[0][1] = newQuadVertex[0][0];
	subBL.v[0][2] = edge[3];
	subBL.v[0][3] = newQuadVertex[0][1];

	subBL.v[1][0] = face[1][0];
	subBL.v[1][1] = edge[5];
	subBL.v[1][2] = face[1][1];
	subBL.v[1][3] = edge[6];

	subBL.v[2][0] = edge[7];
	subBL.v[2][1] = newQuadVertex[1][0];
	subBL.v[2][2] = edge[8];
	subBL.v[2][3] = newQuadVertex[1][1];

	subBL.v[3][0] = face[2][0];
	subBL.v[3][1] = edge[10];
	subBL.v[3][2] = face[2][1];
	subBL.v[3][3] = edge[11];
      }

      __forceinline void subdivide(RegularCatmullClarkPatchT child[4]) const
      {
	T face[3][3];
	face[0][0] = computeFaceVertex(0,0);
	face[0][1] = computeFaceVertex(0,1);
	face[0][2] = computeFaceVertex(0,2);
	face[1][0] = computeFaceVertex(1,0);
	face[1][1] = computeFaceVertex(1,1);
	face[1][2] = computeFaceVertex(1,2);
	face[2][0] = computeFaceVertex(2,0);
	face[2][1] = computeFaceVertex(2,1);
	face[2][2] = computeFaceVertex(2,2);

	T edge[12];
	edge[0]  = 0.25f * (v[0][1] + v[1][1] + face[0][0] + face[0][1]);
	edge[1]  = 0.25f * (v[0][2] + v[1][2] + face[0][1] + face[0][2]);
	edge[2]  = 0.25f * (v[1][0] + v[1][1] + face[0][0] + face[1][0]);
	edge[3]  = 0.25f * (v[1][1] + v[1][2] + face[0][1] + face[1][1]);
	edge[4]  = 0.25f * (v[1][2] + v[1][3] + face[0][2] + face[1][2]);
	edge[5]  = 0.25f * (v[1][1] + v[2][1] + face[1][0] + face[1][1]);
	edge[6]  = 0.25f * (v[1][2] + v[2][2] + face[1][1] + face[1][2]);
	edge[7]  = 0.25f * (v[2][0] + v[2][1] + face[1][0] + face[2][0]);
	edge[8]  = 0.25f * (v[2][1] + v[2][2] + face[1][1] + face[2][1]);
	edge[9]  = 0.25f * (v[2][2] + v[2][3] + face[1][2] + face[2][2]);
	edge[10] = 0.25f * (v[2][1] + v[3][1] + face[2][0] + face[2][1]);
	edge[11] = 0.25f * (v[2][2] + v[3][2] + face[2][1] + face[2][2]);

	T newQuadVertex[2][2];
	newQuadVertex[0][0] = computeQuadVertex(1,1,face);
	newQuadVertex[0][1] = computeQuadVertex(1,2,face);
	newQuadVertex[1][1] = computeQuadVertex(2,2,face);
	newQuadVertex[1][0] = computeQuadVertex(2,1,face);

	initSubPatches(edge,face,newQuadVertex,child);
      }
    };


  class __aligned(64) RegularCatmullClarkPatch : public RegularCatmullClarkPatchT<Vec3fa> 
  {
  public:

    RegularCatmullClarkPatch () {}

    __forceinline void init( FinalQuad& quad ) const
    {
      quad.vtx[0] = v[1][1];
      quad.vtx[1] = v[1][2];
      quad.vtx[2] = v[2][2];
      quad.vtx[3] = v[2][1];
    };

    __forceinline Vec3fa limitVtx0() const { return computeLimitVertex(1,1); }
    __forceinline Vec3fa limitVtx1() const { return computeLimitVertex(1,2); }
    __forceinline Vec3fa limitVtx2() const { return computeLimitVertex(2,2); }
    __forceinline Vec3fa limitVtx3() const { return computeLimitVertex(2,1); }

    __forceinline void init_limit( FinalQuad& quad ) const
    {

      const Vec3fa limit_v0 = computeLimitVertex(1,1);
      const Vec3fa limit_v1 = computeLimitVertex(1,2);
      const Vec3fa limit_v2 = computeLimitVertex(2,2);
      const Vec3fa limit_v3 = computeLimitVertex(2,1);
      
      /* const Vec3fa limit_normal0 = computeLimitNormal(1,1); */
      /* const Vec3fa limit_normal1 = computeLimitNormal(1,2); */
      /* const Vec3fa limit_normal2 = computeLimitNormal(2,2); */
      /* const Vec3fa limit_normal3 = computeLimitNormal(2,1); */

      quad.vtx[0] = limit_v0;
      quad.vtx[1] = limit_v1;
      quad.vtx[2] = limit_v2;
      quad.vtx[3] = limit_v3;
    };

    __forceinline void init(const IrregularCatmullClarkPatch &irreg_patch)
    {
      assert( irreg_patch.dicable() );

      v[1][1] = irreg_patch.ring[0].vtx;
      v[0][1] = irreg_patch.ring[0].get_ring(2);
      v[0][0] = irreg_patch.ring[0].get_ring(3);
      v[1][0] = irreg_patch.ring[0].get_ring(4);

      v[1][2] = irreg_patch.ring[1].vtx;
      v[1][3] = irreg_patch.ring[1].get_ring(2);
      v[0][3] = irreg_patch.ring[1].get_ring(3);
      v[0][2] = irreg_patch.ring[1].get_ring(4);

      v[2][2] = irreg_patch.ring[2].vtx;
      v[3][2] = irreg_patch.ring[2].get_ring(2);
      v[3][3] = irreg_patch.ring[2].get_ring(3);
      v[2][3] = irreg_patch.ring[2].get_ring(4);

      v[2][1] = irreg_patch.ring[3].vtx;
      v[2][0] = irreg_patch.ring[3].get_ring(2);
      v[3][0] = irreg_patch.ring[3].get_ring(3);      
      v[3][1] = irreg_patch.ring[3].get_ring(4);
    }

    __forceinline void init(const SubdivMesh::HalfEdge *const first_half_edge,
			    const Vec3fa *const vertices)
    {
      IrregularCatmullClarkPatch ipatch( first_half_edge, vertices );
      init( ipatch );
    }

    __forceinline BBox3fa bounds() const
    {
      const Vec3fa *const cv = &v[0][0];
      BBox3fa bounds ( cv[0] );
      for (size_t i = 1; i<16 ; i++)
	bounds.extend( cv[i] );
      return bounds;
    }

#if defined(__MIC__)

    __forceinline mic_f getRow(const size_t i) const
    {
      return load16f(&v[i][0]);
    }

    __forceinline void prefetchData() const
    {
      prefetch<PFHINT_L1>(&v[0][0]);
      prefetch<PFHINT_L1>(&v[1][0]);
      prefetch<PFHINT_L1>(&v[2][0]);
      prefetch<PFHINT_L1>(&v[3][0]);
    }
#endif

    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      // FIXME: merge u,v and extract after computation

      const Vec4f v_n = CubicBSplineCurve::eval(vv);

      const Vec3fa_t curve0 = v_n[0] * v[0][0] + v_n[1] * v[1][0] + v_n[2] * v[2][0] + v_n[3] * v[3][0];
      const Vec3fa_t curve1 = v_n[0] * v[0][1] + v_n[1] * v[1][1] + v_n[2] * v[2][1] + v_n[3] * v[3][1];
      const Vec3fa_t curve2 = v_n[0] * v[0][2] + v_n[1] * v[1][2] + v_n[2] * v[2][2] + v_n[3] * v[3][2];
      const Vec3fa_t curve3 = v_n[0] * v[0][3] + v_n[1] * v[1][3] + v_n[2] * v[2][3] + v_n[3] * v[3][3];

      const Vec4f u_n = CubicBSplineCurve::eval(uu);

      return (u_n[0] * curve0 + u_n[1] * curve1 + u_n[2] * curve2 + u_n[3] * curve3) * 1.0f/36.0f;
    }

#if defined(__MIC__)

    __forceinline mic_f eval4(const mic_f uu, const mic_f vv) const
    {
      // FIXME: merge u,v and extract after computation

      const mic4f v_n = CubicBSplineCurve::eval(vv); //FIXME: precompute in table

      const mic_f curve0 = v_n[0] * broadcast4to16f(&v[0][0]) + v_n[1] * broadcast4to16f(&v[1][0]) + v_n[2] * broadcast4to16f(&v[2][0]) + v_n[3] * broadcast4to16f(&v[3][0]);
      const mic_f curve1 = v_n[0] * broadcast4to16f(&v[0][1]) + v_n[1] * broadcast4to16f(&v[1][1]) + v_n[2] * broadcast4to16f(&v[2][1]) + v_n[3] * broadcast4to16f(&v[3][1]);
      const mic_f curve2 = v_n[0] * broadcast4to16f(&v[0][2]) + v_n[1] * broadcast4to16f(&v[1][2]) + v_n[2] * broadcast4to16f(&v[2][2]) + v_n[3] * broadcast4to16f(&v[3][2]);
      const mic_f curve3 = v_n[0] * broadcast4to16f(&v[0][3]) + v_n[1] * broadcast4to16f(&v[1][3]) + v_n[2] * broadcast4to16f(&v[2][3]) + v_n[3] * broadcast4to16f(&v[3][3]);

      const mic4f u_n = CubicBSplineCurve::eval(uu); //FIXME: precompute in table

      return (u_n[0] * curve0 + u_n[1] * curve1 + u_n[2] * curve2 + u_n[3] * curve3) * mic_f(1.0f/36.0f);
    }
    
#endif

  };



  __forceinline std::ostream &operator<<(std::ostream &o, const RegularCatmullClarkPatch &p)
    {
      for (size_t y=0;y<4;y++)
	for (size_t x=0;x<4;x++)
	  o << "[" << y << "][" << x << "] " << p.v[y][x] << std::endl;
      return o;
    } 


  class __aligned(64) GregoryPatch : public RegularCatmullClarkPatchT<Vec3fa> 
  {
  public:

    Vec3fa f[2][2]; // need 16 + 4 = 20 control points

    GregoryPatch() {
      memset(this,0,sizeof(GregoryPatch));
    }

    GregoryPatch(const Vec3fa matrix[4][4],
		 const Vec3fa f_m[2][2]) 
      {
	for (size_t y=0;y<4;y++)
	  for (size_t x=0;x<4;x++)
	    v[y][x] = (Vec3fa_t)matrix[y][x];

	for (size_t y=0;y<2;y++)
	  for (size_t x=0;x<2;x++)
	    f[y][x] = (Vec3fa_t)f_m[y][x];
      }

    Vec3fa& p0() { return v[0][0]; }
    Vec3fa& p1() { return v[0][3]; }
    Vec3fa& p2() { return v[3][3]; }
    Vec3fa& p3() { return v[3][0]; }

    Vec3fa& e0_p() { return v[0][1]; }
    Vec3fa& e0_m() { return v[1][0]; }
    Vec3fa& e1_p() { return v[1][3]; }
    Vec3fa& e1_m() { return v[0][2]; }
    Vec3fa& e2_p() { return v[3][2]; }
    Vec3fa& e2_m() { return v[2][3]; }
    Vec3fa& e3_p() { return v[2][0]; }
    Vec3fa& e3_m() { return v[3][1]; }

    Vec3fa& f0_p() { return v[1][1]; }
    Vec3fa& f1_p() { return v[1][2]; }
    Vec3fa& f2_p() { return v[2][2]; }
    Vec3fa& f3_p() { return v[2][1]; }
    Vec3fa& f0_m() { return f[0][0]; }
    Vec3fa& f1_m() { return f[0][1]; }
    Vec3fa& f2_m() { return f[1][1]; }
    Vec3fa& f3_m() { return f[1][0]; }

    const Vec3fa& p0() const { return v[0][0]; }
    const Vec3fa& p1() const { return v[0][3]; }
    const Vec3fa& p2() const { return v[3][3]; }
    const Vec3fa& p3() const { return v[3][0]; }

    const Vec3fa& e0_p() const { return v[0][1]; }
    const Vec3fa& e0_m() const { return v[1][0]; }
    const Vec3fa& e1_p() const { return v[1][3]; }
    const Vec3fa& e1_m() const { return v[0][2]; }
    const Vec3fa& e2_p() const { return v[3][2]; }
    const Vec3fa& e2_m() const { return v[2][3]; }
    const Vec3fa& e3_p() const { return v[2][0]; }
    const Vec3fa& e3_m() const { return v[3][1]; }

    const Vec3fa& f0_p() const { return v[1][1]; }
    const Vec3fa& f1_p() const { return v[1][2]; }
    const Vec3fa& f2_p() const { return v[2][2]; }
    const Vec3fa& f3_p() const { return v[2][1]; }
    const Vec3fa& f0_m() const { return f[0][0]; }
    const Vec3fa& f1_m() const { return f[0][1]; }
    const Vec3fa& f2_m() const { return f[1][1]; }
    const Vec3fa& f3_m() const { return f[1][0]; }


    Vec3fa initCornerVertex(const IrregularCatmullClarkPatch &irreg_patch,
			    const size_t index)
    {
      return irreg_patch.ring[index].getLimitVertex();
    }


    Vec3fa initPositiveEdgeVertex(const IrregularCatmullClarkPatch &irreg_patch,
				  const size_t index,
				  const Vec3fa &p_vtx)
    {
      const Vec3fa tangent = irreg_patch.ring[index].getLimitTangent();
      return 1.0f/3.0f * tangent + p_vtx;
    }

    Vec3fa initNegativeEdgeVertex(const IrregularCatmullClarkPatch &irreg_patch,
				  const size_t index,
				  const Vec3fa &p_vtx)
    {
      const Vec3fa tangent = irreg_patch.ring[index].getSecondLimitTangent();
      return 1.0f/3.0f * tangent + p_vtx;
    }


    void initFaceVertex(const IrregularCatmullClarkPatch &irreg_patch,
			const size_t index,
			const Vec3fa &p_vtx,
			const Vec3fa &e0_p_vtx,
			const Vec3fa &e1_m_vtx,
			const unsigned int valence_p1,
			const Vec3fa &e0_m_vtx,
			const Vec3fa &e3_p_vtx,
			const unsigned int valence_p3,
			Vec3fa &f_p_vtx,
			Vec3fa &f_m_vtx)
    {
      const unsigned int valence         = irreg_patch.ring[index].valence;
      const unsigned int num_vtx         = irreg_patch.ring[index].num_vtx;
      //const unsigned int hard_edge_index = irreg_patch.ring[index].hard_edge_index;

      const Vec3fa &vtx     = irreg_patch.ring[index].vtx;
      const Vec3fa e_i      = irreg_patch.ring[index].getEdgeCenter( 0 );
      const Vec3fa c_i_m_1  = irreg_patch.ring[index].getQuadCenter( 0 );
      const Vec3fa e_i_m_1  = irreg_patch.ring[index].getEdgeCenter( 1 );

      Vec3fa c_i, e_i_p_1;
      //if (unlikely(hard_edge_index == 0))
      if (unlikely(!irreg_patch.ring[index].has_last_face()))
	{
	  /* mirror quad center and edge mid-point */
	  c_i     = c_i_m_1 + 2 * (e_i - c_i_m_1);
	  e_i_p_1 = e_i_m_1 + 2 * (vtx - e_i_m_1);
	}
      else
	{
	  c_i     = irreg_patch.ring[index].getQuadCenter( valence-1 );
	  e_i_p_1 = irreg_patch.ring[index].getEdgeCenter( valence-1 );
	}

      Vec3fa c_i_m_2, e_i_m_2;
      //if (unlikely(hard_edge_index+2 == num_vtx-2) || valence == 2)
      if (unlikely(!irreg_patch.ring[index].has_second_face() || valence == 2))
	{
	  /* mirror quad center and edge mid-point */
	  c_i_m_2  = c_i_m_1 + 2 * (e_i_m_1 - c_i_m_1);
	  e_i_m_2  = e_i + 2 * (vtx - e_i);	  
	}
      else
	{
	  c_i_m_2  = irreg_patch.ring[index].getQuadCenter( 1 );
	  e_i_m_2  = irreg_patch.ring[index].getEdgeCenter( 2 ); //FIXME: is this correct?
	}


      const float d = 3.0f;
      const float c     = cosf(2.0*M_PI/(float)valence);
      const float c_e_p = cosf(2.0*M_PI/(float)valence_p1);
      const float c_e_m = cosf(2.0*M_PI/(float)valence_p3);

      const Vec3fa r_e_p = 1.0f/3.0f * (e_i_m_1 - e_i_p_1) + 2.0f/3.0f * (c_i_m_1 - c_i);

      f_p_vtx =  1.0f / d * (c_e_p * p_vtx + (d - 2.0f*c - c_e_p) * e0_p_vtx + 2.0f*c* e1_m_vtx + r_e_p);

      const Vec3fa r_e_m = 1.0f/3.0f * (e_i - e_i_m_2) + 2.0f/3.0f * (c_i_m_1 - c_i_m_2);


      f_m_vtx = 1.0f / d * (c_e_m * p_vtx + (d - 2.0f*c - c_e_m) * e0_m_vtx + 2.0f*c* e3_p_vtx + r_e_m);

#if 0
      DBG_PRINT( irreg_patch.ring[index].vtx );
      DBG_PRINT( p_vtx );
      
      DBG_PRINT( e0_p_vtx );
      DBG_PRINT( e1_m_vtx );

      DBG_PRINT( e0_m_vtx );
      DBG_PRINT( e3_p_vtx );

      DBG_PRINT( c_i );
      DBG_PRINT( c_i_m_1 );
      DBG_PRINT( c_i_m_2 );

      DBG_PRINT( e_i_p_1 );
      DBG_PRINT( e_i );
      DBG_PRINT( e_i_m_1 );
      DBG_PRINT( e_i_m_2 );

      DBG_PRINT( r_e_p );           
      DBG_PRINT( r_e_m );           

      DBG_PRINT(f_p_vtx);
      DBG_PRINT(f_m_vtx);
      //exit(0);
#endif

    }

    __forceinline void init(const IrregularCatmullClarkPatch &irreg_patch)
    {
      p0() = initCornerVertex(irreg_patch,0);
      p1() = initCornerVertex(irreg_patch,1);
      p2() = initCornerVertex(irreg_patch,2);
      p3() = initCornerVertex(irreg_patch,3);

      e0_p() = initPositiveEdgeVertex(irreg_patch,0, p0());
      e1_p() = initPositiveEdgeVertex(irreg_patch,1, p1());
      e2_p() = initPositiveEdgeVertex(irreg_patch,2, p2());
      e3_p() = initPositiveEdgeVertex(irreg_patch,3, p3());

      e0_m() = initNegativeEdgeVertex(irreg_patch,0, p0());
      e1_m() = initNegativeEdgeVertex(irreg_patch,1, p1());
      e2_m() = initNegativeEdgeVertex(irreg_patch,2, p2());
      e3_m() = initNegativeEdgeVertex(irreg_patch,3, p3());

      const unsigned int valence_p0 = irreg_patch.ring[0].valence;
      const unsigned int valence_p1 = irreg_patch.ring[1].valence;
      const unsigned int valence_p2 = irreg_patch.ring[2].valence;
      const unsigned int valence_p3 = irreg_patch.ring[3].valence;

#if 0
      DBG_PRINT( p0() );
      DBG_PRINT( p1() );
      DBG_PRINT( p2() );
      DBG_PRINT( p3() );

      DBG_PRINT( e0_p() );
      DBG_PRINT( e1_p() );
      DBG_PRINT( e2_p() );
      DBG_PRINT( e3_p() );

      DBG_PRINT( e0_m() );
      DBG_PRINT( e1_m() );
      DBG_PRINT( e2_m() );
      DBG_PRINT( e3_m() );
      //exit(0);
#endif




      initFaceVertex(irreg_patch,0,p0(),e0_p(),e1_m(),valence_p1,e0_m(),e3_p(),valence_p3,f0_p(),f0_m() );

      initFaceVertex(irreg_patch,1,p1(),e1_p(),e2_m(),valence_p2,e1_m(),e0_p(),valence_p0,f1_p(),f1_m() );

      initFaceVertex(irreg_patch,2,p2(),e2_p(),e3_m(),valence_p3,e2_m(),e1_p(),valence_p1,f2_p(),f2_m() );

      initFaceVertex(irreg_patch,3,p3(),e3_p(),e0_m(),valence_p0,e3_m(),e2_p(),valence_p3,f3_p(),f3_m() );

    }

   __forceinline BBox3fa bounds() const
    {
      const Vec3fa *const cv = &v[0][0];
      BBox3fa bounds ( cv[0] );
      for (size_t i = 1; i<16 ; i++)
	bounds.extend( cv[i] );
      bounds.extend(f[0][0]);
      bounds.extend(f[1][0]);
      bounds.extend(f[1][1]);
      bounds.extend(f[1][1]);
      return bounds;
    }
 
   __forceinline void exportConrolPoints( Vec3fa matrix[4][4], Vec3fa f_m[2][2] ) const
   {
     for (size_t y=0;y<4;y++)
       for (size_t x=0;x<4;x++)
	 matrix[y][x] = (Vec3fa_t)v[y][x];

     for (size_t y=0;y<2;y++)
       for (size_t x=0;x<2;x++)
	 f_m[y][x] = (Vec3fa_t)f[y][x];
   }

   __forceinline void exportBicubicBezierPatch( Vec3fa matrix[4][4], const float uu, const float vv ) const
   {
     for (size_t y=0;y<4;y++)
       for (size_t x=0;x<4;x++)
	 matrix[y][x] = (Vec3fa_t)v[y][x];

     if (uu == 0.0f || uu == 1.0f || vv == 0.0f || vv == 1.0f) return;

     // FIXME: merge u,v and extract after computation

     const Vec3fa_t F0 = (      uu  * f0_p() +       vv  * f0_m()) * 1.0f/(uu+vv);
     const Vec3fa_t F1 = ((1.0f-uu) * f1_m() +       vv  * f1_p()) * 1.0f/(1.0f-uu+vv);
     const Vec3fa_t F2 = ((1.0f-uu) * f2_p() + (1.0f-vv) * f2_m()) * 1.0f/(2.0f-uu-vv);
     const Vec3fa_t F3 = (      uu  * f3_m() + (1.0f-vv) * f3_p()) * 1.0f/(1.0f+uu-vv);

     matrix[1][1] = F0;
     matrix[1][2] = F1;
     matrix[2][2] = F2;
     matrix[2][1] = F3;     

   }

   
#if defined(__MIC__)
   static __forceinline mic_f eval4(const Vec3fa matrix[4][4],
				    const Vec3fa f[2][2],
				    const mic_f uu,
				    const mic_f vv) 
   {
     const mic_m m_border = (uu == 0.0f) | (uu == 1.0f) | (vv == 0.0f) | (vv == 1.0f);

     const mic_f f0_p = (Vec3fa_t)matrix[1][1];
     const mic_f f0_m = (Vec3fa_t)f[0][0];

     const mic_f f1_p = (Vec3fa_t)matrix[1][2];
     const mic_f f1_m = (Vec3fa_t)f[0][1];

     const mic_f f2_p = (Vec3fa_t)matrix[2][2];
     const mic_f f2_m = (Vec3fa_t)f[1][1];

     const mic_f f3_p = (Vec3fa_t)matrix[2][1];
     const mic_f f3_m = (Vec3fa_t)f[1][0];

     const mic_f F0 = select(m_border,f0_p, (      uu  * f0_p +       vv  * f0_m) * 1.0f/(uu+vv)      );
     const mic_f F1 = select(m_border,f1_p, ((1.0f-uu) * f1_m +       vv  * f1_p) * 1.0f/(1.0f-uu+vv) );
     const mic_f F2 = select(m_border,f2_p, ((1.0f-uu) * f2_p + (1.0f-vv) * f2_m) * 1.0f/(2.0f-uu-vv) );
     const mic_f F3 = select(m_border,f3_p, (      uu  * f3_m + (1.0f-vv) * f3_p) * 1.0f/(1.0f+uu-vv) );

     const mic_f one_minus_uu = mic_f(1.0f) - uu;
     const mic_f one_minus_vv = mic_f(1.0f) - vv;      

     // FIXME: merge u,v and extract after computation
     const mic_f B0_u = one_minus_uu * one_minus_uu * one_minus_uu;
     const mic_f B0_v = one_minus_vv * one_minus_vv * one_minus_vv;
     const mic_f B1_u = 3.0f * one_minus_uu * one_minus_uu * uu;
     const mic_f B1_v = 3.0f * one_minus_vv * one_minus_vv * vv;
     const mic_f B2_u = 3.0f * one_minus_uu * uu * uu;
     const mic_f B2_v = 3.0f * one_minus_vv * vv * vv;
     const mic_f B3_u = uu * uu * uu;
     const mic_f B3_v = vv * vv * vv;

     const mic_f res = 
	(B0_u * (Vec3fa_t)matrix[0][0] + B1_u * (Vec3fa_t)matrix[0][1] + B2_u * (Vec3fa_t)matrix[0][2] + B3_u * (Vec3fa_t)matrix[0][3]) * B0_v + 
	(B0_u * (Vec3fa_t)matrix[1][0] + B1_u * F0 + B2_u * F1 + B3_u * (Vec3fa_t)matrix[1][3]) * B1_v + 
	(B0_u * (Vec3fa_t)matrix[2][0] + B1_u * F3 + B2_u * F2 + B3_u * (Vec3fa_t)matrix[2][3]) * B2_v + 
	(B0_u * (Vec3fa_t)matrix[3][0] + B1_u * (Vec3fa_t)matrix[3][1] + B2_u * (Vec3fa_t)matrix[3][2] + B3_u * (Vec3fa_t)matrix[3][3]) * B3_v; 
     return res;
   }
#endif

    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      __aligned(64) Vec3fa matrix[4][4];
      exportBicubicBezierPatch(matrix,uu,vv);

      const float one_minus_uu = 1.0f - uu;
      const float one_minus_vv = 1.0f - vv;      

     // FIXME: merge u,v and extract after computation

      const float B0_u = one_minus_uu * one_minus_uu * one_minus_uu;
      const float B0_v = one_minus_vv * one_minus_vv * one_minus_vv;
      const float B1_u = 3.0f * one_minus_uu * one_minus_uu * uu;
      const float B1_v = 3.0f * one_minus_vv * one_minus_vv * vv;
      const float B2_u = 3.0f * one_minus_uu * uu * uu;
      const float B2_v = 3.0f * one_minus_vv * vv * vv;
      const float B3_u = uu * uu * uu;
      const float B3_v = vv * vv * vv;

      const Vec3fa_t res = 
	(B0_u * matrix[0][0] + B1_u * matrix[0][1] + B2_u * matrix[0][2] + B3_u * matrix[0][3]) * B0_v + 
	(B0_u * matrix[1][0] + B1_u * matrix[1][1] + B2_u * matrix[1][2] + B3_u * matrix[1][3]) * B1_v + 
	(B0_u * matrix[2][0] + B1_u * matrix[2][1] + B2_u * matrix[2][2] + B3_u * matrix[2][3]) * B2_v + 
	(B0_u * matrix[3][0] + B1_u * matrix[3][1] + B2_u * matrix[3][2] + B3_u * matrix[3][3]) * B3_v; 
      
      return res;

    }

  };


 __forceinline std::ostream &operator<<(std::ostream &o, const GregoryPatch &p)
    {
      for (size_t y=0;y<4;y++)
	for (size_t x=0;x<4;x++)
	  o << "v[" << y << "][" << x << "] " << p.v[y][x] << std::endl;

     for (size_t y=0;y<2;y++)
	for (size_t x=0;x<2;x++)
	  o << "f[" << y << "][" << x << "] " << p.f[y][x] << std::endl;
      return o;
    } 


};

