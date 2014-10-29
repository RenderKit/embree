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
    Vec3fa vtx;
    Vec3fa ring[2*MAX_VALENCE]; // two vertices per face
    float crease_weight[MAX_VALENCE];
    unsigned int valence;
    unsigned int num_vtx;
    int hard_edge_index;
    float level;                 // per vertex subdivision level

#if !defined(__MIC__)
    typedef Vec3fa Vertex;
#else
    typedef Vec3fa_t Vertex;      
#endif

    CatmullClark1Ring() {}

    __forceinline const Vec3fa& begin() const {
      assert(num_vtx>0);
      return ring[0];
    }

    __forceinline const Vec3fa& first() const {
      assert(num_vtx>2);
      return ring[2];
    }

    __forceinline bool has_first() const {
      return hard_edge_index != 0;
    }
    
    __forceinline const Vec3fa& last() const {
      assert(num_vtx>=4);
      return ring[num_vtx-4];
    }

    __forceinline bool has_last() const {
      assert(num_vtx>=2);
      return (hard_edge_index == -1) || ((num_vtx-4) >= hard_edge_index+2);
    }

    __forceinline const Vec3fa& end() const {
      assert(num_vtx>=2);
      return ring[num_vtx-2];
    }

    __forceinline BBox3fa bounds() const
    {
      BBox3fa bounds ( vtx );
      for (size_t i = 0; i<num_vtx ; i++)
	bounds.extend( ring[i] );
      return bounds;
    }

    
    __forceinline void init(const SubdivMesh::HalfEdge* const h, const Vec3fa* const vertices)
    {
      hard_edge_index = -1;
      vtx = (Vertex)vertices[ h->getStartVertexIndex() ];

      SubdivMesh::HalfEdge* p = (SubdivMesh::HalfEdge*) h;

      size_t i=0;
      do {
        assert(i < 2*MAX_VALENCE);
        crease_weight[i/2] = p->crease_weight;
	ring[i++] = (Vertex) vertices[ p->next()->getStartVertexIndex() ];
        
	if (unlikely(!p->hasOpposite())) { 
          init_clockwise(h,vertices,i);
          return;
        }
	p = p->opposite();

        assert(i < 2*MAX_VALENCE);
	ring[i++] = (Vertex) vertices[ p->prev()->getStartVertexIndex() ];
	p = p->next();
        
      } while (p != h);

      num_vtx = i;
      valence = i >> 1;
    }

    __forceinline void init_clockwise(const SubdivMesh::HalfEdge* const h, const Vec3fa* const vertices, size_t i)
    {
      /*! mark first hard edge and store dummy vertex for face between the two hard edges*/
      hard_edge_index = i-1;
      crease_weight[i/2] = inf; 
      ring[i++] = Vec3fa(nan);
      
      /*! first cycle clock-wise until we found the second edge */	  
      SubdivMesh::HalfEdge* p = (SubdivMesh::HalfEdge*) h;
      p = p->prev();	  
      while(p->hasOpposite()) {
        p = p->opposite();
        p = p->prev();	      
      }
      
      /*! store second hard edge and diagonal vertex */
      crease_weight[i/2] = inf;
      ring[i++] = (Vertex)vertices[ p->getStartVertexIndex() ];
      ring[i++] = (Vertex)vertices[ p->prev()->getStartVertexIndex() ];
      p = p->next();
	  
      /*! continue counter-clockwise */	  
      while (p != h) 
      {
        assert( i < 2*MAX_VALENCE );
        ring[i++] = (Vertex)vertices[ p->next()->getStartVertexIndex() ];
        p = p->opposite();
        
        assert( i < 2*MAX_VALENCE );
        ring[i++] = (Vertex)vertices[ p->prev()->getStartVertexIndex() ];
        p = p->next();	    
      }

      num_vtx = i;
      valence = i >> 1;
    }

    __forceinline void update(CatmullClark1Ring& dest) const
    {
      dest.valence         = valence;
      dest.num_vtx         = num_vtx;
      dest.hard_edge_index = hard_edge_index;

      /* calculate face points */
      Vertex S = zero;
      for (size_t i=0; i<valence-1; i++) {
        S += dest.ring[2*i+1] = ((vtx + ring[2*i]) + (ring[2*i+1] + ring[2*i+2])) * 0.25f;
      }
      S += dest.ring[num_vtx-1] = ((vtx + ring[num_vtx-2]) + (ring[num_vtx-1] + ring[0])) * 0.25f;
      
      /* calculate new edge points */
      size_t num_creases = 0;
      size_t crease_id[MAX_VALENCE];
      Vertex C = 0.0f;
      for (size_t i=1; i<valence; i++)
      {
        const Vertex v = vtx + ring[2*i];
        const Vertex f = dest.ring[2*i-1] + dest.ring[2*i+1];
        S += ring[2*i];
        dest.crease_weight[i] = crease_weight[i];
        
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
        const Vertex v = vtx + ring[2*i];
        const Vertex f = dest.ring[num_vtx-1] + dest.ring[2*i+1];
        S += ring[2*i];
        dest.crease_weight[i] = crease_weight[i];

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
      const Vertex v_smooth = (Vertex)(S*inv_valence + (float(valence)-2.0f)*vtx)*inv_valence;
      dest.vtx = v_smooth;
      if (likely(num_creases <= 1)) 
        return;
      
      /* compute new vertex using crease rule */
      if (likely(num_creases == 2)) {
        const Vertex v_sharp = (Vertex)(C + 6.0f * vtx) * (1.0f / 8.0f);
        const float crease_weight0 = crease_weight[crease_id[0]];
        const float crease_weight1 = crease_weight[crease_id[1]];
        dest.vtx = v_sharp;
        dest.crease_weight[crease_id[0]] = max(0.25f*(3.0f*crease_weight0 + crease_weight1)-1.0f,0.0f);
        dest.crease_weight[crease_id[1]] = max(0.25f*(3.0f*crease_weight1 + crease_weight0)-1.0f,0.0f);
        const float t0 = 0.5f*(crease_weight0+crease_weight1), t1 = 1.0f-t0;
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
      
      for (size_t i=0; i<valence; i++) {
        if (crease_weight[i] > 0.0f) return false;
      }
      return true;
    }

    __forceinline Vec3fa getLimitVertex() const
    {
      Vertex F( 0.0f );
      Vertex E( 0.0f );

      for (size_t i=0; i<valence; i++) {
        F += ring[2*i+1];
        E += ring[2*i];
      }

      const float n = (float)valence;
      return (Vertex)(n*n*vtx+4*E+F) / ((n+5.0f)*n);      
    }

    __forceinline Vec3fa getLimitTangent() const // FIXME: what is this supposed to calclate? there should be 2 tangents
    {
      Vertex alpha( 0.0f );
      Vertex beta( 0.0f );

      const float n = (float)valence;
      const float c = 1.0f/n * 1.0f / sqrtf(4.0f + cos(M_PI/n)*cos(M_PI/n));  
      for (size_t i=0; i<valence; i++)
	{
	  alpha += (1.0f/n + cosf(M_PI/n) * c) * cosf(2.0f*M_PI*(float)i/n) * ring[2*i];
          beta += c * cosf((2.0f*M_PI*(float)i+M_PI)/n) * ring[2*i+1];
	}

      return alpha +  beta;      
    }


    friend __forceinline std::ostream &operator<<(std::ostream &o, const CatmullClark1Ring &c)
      {
	o << "vtx " << c.vtx << " valence " << c.valence << " num_vtx " << c.num_vtx << " hard_edge_index " << c.hard_edge_index << " ring: " << std::endl;
	for (size_t i=0;i<c.num_vtx;i++)
	  o << i << " -> " << c.ring[i] << std::endl;
	return o;
      } 
  };

  class CatmullClark1Edge
  {
  public:
    void init(size_t N, CatmullClark1Ring& r0, CatmullClark1Ring& r1)
    {
      v.init(N,3);
      for (size_t i=0; i<N; i++) v(i,0) = v(i,1) = v(i,2) = Vec3fa(nan);
      this->K = 1;
      init(r0,r1);
    }

    __forceinline void subdivide(CatmullClark1Ring& r0, CatmullClark1Ring& r1) 
    {
      assert(2*K+1 <= v.width());
      size_t K0 = K;
      size_t K1 = 2*K;
      
      Vec3fa v20 = zero;
      Vec3fa v21 = zero;
      Vec3fa v22 = zero;
      
      Vec3fa v10 = v((K+1)-1,0);
      Vec3fa v11 = v((K+1)-1,1);
      Vec3fa v12 = v((K+1)-1,2);
      
      for (ssize_t x=K-1; x>=0; x--) 
	{
	  /* load next column */
	  Vec3fa v00 = v(x,0);
	  Vec3fa v01 = v(x,1);
	  Vec3fa v02 = v(x,2);
        
	  /* calculate face points and edge centers */
	  const Vec3fa c00 = 0.25f*(v00+v10+v01+v11);
	  const Vec3fa c10 = 0.50f*(v10+v11);
	  const Vec3fa c20 = 0.25f*(v10+v20+v11+v21);
	  const Vec3fa c01 = 0.50f*(v01+v11);
	  const Vec3fa c21 = 0.50f*(v11+v21);
	  const Vec3fa c02 = 0.25f*(v01+v11+v02+v12);
	  const Vec3fa c12 = 0.50f*(v11+v12);
	  const Vec3fa c22 = 0.25f*(v11+v21+v12+v22);
        
	  /* store face points and edge point at 2*x+1 */
	  v(2*x+1,0) = c00;
	  v(2*x+1,1) = 0.5f*(c01+0.5f*(c00+c02));
	  v(2*x+1,2) = c02;
        
	  /* store face points and edge point at 2*x+2 */
	  const Vec3fa F = 0.25f*(c00+c20+c02+c22);
	  const Vec3fa R = 0.25f*(c10+c01+c21+c12);
	  const Vec3fa P = v11;
	  v(2*x+2,0) = 0.5f*(c10+0.5f*(c00+c20));
	  v(2*x+2,1) = 0.25*F + 0.5*R + 0.25*P;
	  v(2*x+2,2) = 0.5f*(c12+0.5f*(c02+c22));
        
	  /* propagate points to next iteration */
	  v20 = v10; v10 = v00;
	  v21 = v11; v11 = v01;
	  v22 = v12; v12 = v02;
	}        

      K = 2*K;
      init(r0,r1);
    }
    
    __forceinline void init(CatmullClark1Ring& ring0, CatmullClark1Ring& ring1) 
    {
      v(0,0) = ring0.first();
      v(0,1) = ring0.vtx;
      v(0,2) = ring0.end();
      if (!ring0.has_first()) v(0,0) = 2*v(0,1) - v(0,2);

      v(K,0) = ring1.last();
      v(K,1) = ring1.vtx;
      v(K,2) = ring1.begin();
      if (!ring1.has_last()) v(K,0) = 2*v(K,1) - v(K,2);
    }
    
    friend __forceinline std::ostream &operator<<(std::ostream& out, const CatmullClark1Edge& edge)
    {
      out << std::endl;
      for (size_t y=0; y<3; y++) {
        for (size_t x=0; x<edge.K+1; x++) {
          //out << patch.v(x,y) << " ";
          out << std::setw(10) << edge.v(x,y).x << " ";
        }
        out << std::endl;
      }
      return out;
    } 
    
  public:
    Array2D<Vec3fa> v;
    size_t K;
  };
  
  class __aligned(64) IrregularCatmullClarkPatch
  {
  public:
    CatmullClark1Ring ring[4];
    float level[4];

#if !defined(__MIC__)
    typedef Vec3fa Vertex;
#else
    typedef Vec3fa_t Vertex;      
#endif

    __forceinline IrregularCatmullClarkPatch () {}

    __forceinline IrregularCatmullClarkPatch (const SubdivMesh::HalfEdge* first_half_edge, const Vec3fa* vertices) 
    {
      for (size_t i=0; i<4; i++) {
        ring[i].init(first_half_edge+i,vertices);
        level[i] = first_half_edge[i].level;
      }
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
      dest1.vtx = dest0.vtx = (Vertex)p0.ring[0];

      dest1.ring[6] = dest0.ring[0] = (Vertex)p0.ring[p0.num_vtx-1];
      dest1.ring[7] = dest0.ring[1] = (Vertex)p1.ring[0];
      dest1.ring[0] = dest0.ring[2] = (Vertex)p1.vtx;
      dest1.ring[1] = dest0.ring[3] = (Vertex)p1.ring[p1.num_vtx-4];
      dest1.ring[2] = dest0.ring[4] = (Vertex)p0.ring[1];
      dest1.ring[3] = dest0.ring[5] = (Vertex)p0.ring[2];
      dest1.ring[4] = dest0.ring[6] = (Vertex)p0.vtx;
      dest1.ring[5] = dest0.ring[7] = (Vertex)p0.ring[p0.num_vtx-2];

      dest1.crease_weight[3] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[1] = p1.crease_weight[p1.valence-1];
      dest1.crease_weight[1] = dest0.crease_weight[2] = 0.0f;
      dest1.crease_weight[2] = dest0.crease_weight[3] = p0.crease_weight[0];
    }


    static __forceinline void init_border(const CatmullClark1Ring &p0,
                                          const CatmullClark1Ring &p1,
                                          CatmullClark1Ring &dest0,
                                          CatmullClark1Ring &dest1) 
    {
#if !defined(__MIC__)
    typedef Vec3fa Vertex;
#else
    typedef Vec3fa_t Vertex;      
#endif

      dest1.valence = dest0.valence = 3;
      dest1.num_vtx = dest0.num_vtx = 6;
      dest0.hard_edge_index = 2;
      dest1.hard_edge_index = 0;
      dest1.vtx  = dest0.vtx = (Vertex)p0.ring[0];

      dest1.ring[ 4] = dest0.ring[ 0] = (Vertex)p0.ring[p0.num_vtx-1];
      dest1.ring[ 5] = dest0.ring[ 1] = (Vertex)p1.ring[0];
      dest1.ring[ 0] = dest0.ring[ 2] = (Vertex)p1.vtx;
      dest1.ring[ 1] = dest1.ring[ 3] = (Vertex)p0.ring[p0.hard_edge_index+1]; // dummy
      dest1.ring[ 2] = dest0.ring[ 4] = (Vertex)p0.vtx;
      dest1.ring[ 3] = dest0.ring[ 5] = (Vertex)p0.ring[p0.num_vtx-2];

      dest1.crease_weight[2] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[1] = p1.crease_weight[p1.valence-1];
      dest1.crease_weight[1] = dest0.crease_weight[2] = p0.crease_weight[0];
    }

    static __forceinline void init_regular(const Vertex &center, const Vertex center_ring[8], const size_t offset, CatmullClark1Ring &dest)
    {
      dest.valence = 4;
      dest.num_vtx = 8;
      dest.hard_edge_index = -1;
      dest.vtx     = (Vertex)center;
      for (size_t i=0; i<8; i++) 
	dest.ring[i] = (Vertex)center_ring[(offset+i)%8];
      for (size_t i=0; i<8; i++) 
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
      
      if (likely(ring[0].hard_edge_index != 0))
        init_regular(patch[0].ring[0],patch[1].ring[1],patch[0].ring[1],patch[1].ring[0]);
      else
        init_border(patch[0].ring[0],patch[1].ring[1],patch[0].ring[1],patch[1].ring[0]);

      if (likely(ring[1].hard_edge_index != 0))
        init_regular(patch[1].ring[1],patch[2].ring[2],patch[1].ring[2],patch[2].ring[1]);
      else
        init_border(patch[1].ring[1],patch[2].ring[2],patch[1].ring[2],patch[2].ring[1]);

      if (likely(ring[2].hard_edge_index != 0))
        init_regular(patch[2].ring[2],patch[3].ring[3],patch[2].ring[3],patch[3].ring[2]);
      else
        init_border(patch[2].ring[2],patch[3].ring[3],patch[2].ring[3],patch[3].ring[2]);

      if (likely(ring[3].hard_edge_index != 0))
        init_regular(patch[3].ring[3],patch[0].ring[0],patch[3].ring[0],patch[0].ring[3]);
      else
        init_border(patch[3].ring[3],patch[0].ring[0],patch[3].ring[0],patch[0].ring[3]);

      Vertex center = (ring[0].vtx + ring[1].vtx + ring[2].vtx + ring[3].vtx) * 0.25f;
      Vertex center_ring[8];

      // counter-clockwise
      center_ring[0] = (Vertex)patch[3].ring[3].ring[0];
      center_ring[1] = (Vertex)patch[3].ring[3].vtx;
      center_ring[2] = (Vertex)patch[2].ring[2].ring[0];
      center_ring[3] = (Vertex)patch[2].ring[2].vtx;
      center_ring[4] = (Vertex)patch[1].ring[1].ring[0];
      center_ring[5] = (Vertex)patch[1].ring[1].vtx;
      center_ring[6] = (Vertex)patch[0].ring[0].ring[0];
      center_ring[7] = (Vertex)patch[0].ring[0].vtx;

      init_regular(center,center_ring,0,patch[0].ring[2]);
      init_regular(center,center_ring,2,patch[3].ring[1]);
      init_regular(center,center_ring,4,patch[2].ring[0]);
      init_regular(center,center_ring,6,patch[1].ring[3]);
    }

    __forceinline void init( FinalQuad& quad ) const
    {
      quad.vtx[0] = (Vertex)ring[0].vtx;
      quad.vtx[1] = (Vertex)ring[1].vtx;
      quad.vtx[2] = (Vertex)ring[2].vtx;
      quad.vtx[3] = (Vertex)ring[3].vtx;
    };

    friend __forceinline std::ostream &operator<<(std::ostream &o, const IrregularCatmullClarkPatch &p)
    {
      o << "rings: " << std::endl;
      for (size_t i=0;i<4;i++)
	o << i << " -> " << p.ring[i] << std::endl;
      return o;
    } 
  };

  struct SubdivideIrregularCatmullClarkPatch
  {
    SubdivideIrregularCatmullClarkPatch (const IrregularCatmullClarkPatch& patch, const unsigned int levels)
    : K(1)
    {
      size_t N = 1<<levels;
      size_t M = N+1;
      v.init(M,M,Vec3fa(nan));
      ring00 = patch.ring[0];
      ring10 = patch.ring[1];
      ring11 = patch.ring[2];
      ring01 = patch.ring[3];
      edgeT.init(M,ring00,ring10);
      edgeR.init(M,ring10,ring11);
      edgeB.init(M,ring11,ring01);
      edgeL.init(M,ring01,ring00);
      init();

      for (size_t l=0; l<levels; l++)
        subdivide();
    }

      void subdivide_points()
      {
        size_t K0 = K;
        size_t K1 = 2*K;

        assert(2*K+1 <= v.width());
        assert(2*K+1 <= v.height());

        for (ssize_t y=K; y>=0; y--) {
          for (ssize_t x=K; x>=0; x--) {
            v(2*x+0,2*y+0) = v(x,y);
          }
        }
        for (ssize_t y=0; y<K; y++) {
          for (ssize_t x=0; x<K; x++) {
            v(2*x+1,2*y+0) = Vec3fa(nan);
            v(2*x+0,2*y+1) = Vec3fa(nan);
            v(2*x+1,2*y+1) = Vec3fa(nan);
          }
        }

        for (ssize_t x=K-1; x>=0; x--) 
        {
          const Vec3fa c00 = v(2*x+0,0);
          const Vec3fa c10 = v(2*x+2,0);
          const Vec3fa c01 = v(2*x+0,2);
          const Vec3fa c11 = v(2*x+2,2);
          v(2*x+1,1) = 0.25f*(c00+c01+c10+c11);
          v(2*x+2,1) = 0.50f*(c10+c11);
        }
        
        for (size_t y=1; y<K; y++)
        {
          //Vec3fa v20 = zero;
          Vec3fa c20 = zero;
          Vec3fa v21 = zero;
          Vec3fa v22 = zero;

          //Vec3fa v10 = v(2*K,0);
          Vec3fa v11 = v(2*K,2*y);
          Vec3fa v12 = v(2*K,2*y+2);

          for (ssize_t x=K-1; x>=0; x--) 
          {
            /* load next column */
            //Vec3fa v00 = v(2*x,2*y-2);
            Vec3fa v01 = v(2*x,2*y+0);
            Vec3fa v02 = v(2*x,2*y+2);
            
            /* calculate face points and edge centers */
            const Vec3fa c00 = v(2*x+1,2*y-1);
            const Vec3fa c10 = v(2*x+2,2*y-1);
            //const Vec3fa c20 = v(2*x+3,2*y-1);
            const Vec3fa c01 = 0.50f*(v01+v11);
            const Vec3fa c21 = 0.50f*(v11+v21);
            const Vec3fa c02 = 0.25f*(v01+v11+v02+v12);
            const Vec3fa c12 = 0.50f*(v11+v12);
            const Vec3fa c22 = 0.25f*(v11+v21+v12+v22);

            /* store face points and edge point at 2*x+1 */
            //v(2*x+1,2*y-1) = c00;
            v(2*x+1,2*y+0) = 0.5f*(c01+0.5f*(c00+c02));
            v(2*x+1,2*y+1) = c02;

            /* store face points and edge point at 2*x+2 */
            const Vec3fa F = 0.25f*(c00+c20+c02+c22);
            const Vec3fa R = 0.25f*(c10+c01+c21+c12);
            const Vec3fa P = v11;
            v(2*x+2,2*y-1) = 0.5f*(c10+0.5f*(c00+c20));
            v(2*x+2,2*y+0) = 0.25*F + 0.5*R + 0.25*P;
            v(2*x+2,2*y+1) = c12;
            
            /* propagate points to next iteration */
            //v20 = v10; v10 = v00;
            v21 = v11; v11 = v01;
            v22 = v12; v12 = v02;
            c20 = c00;
          }        
        }

        for (ssize_t x=1; x<K; x++) 
          v(2*x,2*K-1) = 0.5f*(v(2*x,2*K-1)+0.5f*(v(2*x-1,2*K-1)+v(2*x+1,2*K-1)));

        K=2*K;
        init();
      }

      void init()
      {
        for (size_t i=0; i<(K+1); i++)
        {
          v(i,0) = edgeT.v(i,1);
          v(K,i) = edgeR.v(i,1);
          v(i,K) = edgeB.v(K-i,1);
          v(0,i) = edgeL.v(K-i,1);
        }
      }

      void subdivide()
      {
        CatmullClark1Ring ring00a; ring00.update(ring00a); ring00 = ring00a;
        CatmullClark1Ring ring01a; ring01.update(ring01a); ring01 = ring01a;
        CatmullClark1Ring ring10a; ring10.update(ring10a); ring10 = ring10a;
        CatmullClark1Ring ring11a; ring11.update(ring11a); ring11 = ring11a;
        edgeT.subdivide(ring00,ring10);
        edgeR.subdivide(ring10,ring11);
        edgeB.subdivide(ring11,ring01);
        edgeL.subdivide(ring01,ring00);
        subdivide_points();
      }

  public:
    size_t K;
    CatmullClark1Ring ring00,ring01,ring10,ring11;
    CatmullClark1Edge edgeT, edgeB, edgeL, edgeR; 
    Array2D<Vec3fa> v;
  };

  class CubicBSplineCurve
  {
  public:

#if !defined(__MIC__)
    typedef Vec3fa Vertex;
#else
    typedef Vec3fa_t Vertex;      
#endif

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

#if !defined(__MIC__)
    typedef Vec3fa Vertex;
#else
    typedef Vec3fa_t Vertex;      
#endif

    static __forceinline Vertex eval(const float u, const Vertex &p0, const Vertex &p1, const Vertex &p2, const Vertex &p3)
    {
      // FIXME: lookup
      const float t  = u;
      const float s  = 1.0f - u;
      const float n0 = s*s*s;
      const float n1 = 3.0f*t*s*t;
      const float n2 = 3.0f*s*t*s;
      const float n3 = t*t*t;
      const Vertex n = p0 * n0 + p1 * n1 + p2 * n2 + p3 * n3;
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

#if !defined(__MIC__)
      typedef Vec3fa Vertex;
#else
      typedef Vec3fa_t Vertex;
#endif

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

    __forceinline void init(const SubdivMesh::HalfEdge *const first_half_edge,
			    const Vec3fa *const vertices)
    {
      // quad(0,0)
      const SubdivMesh::HalfEdge *v11 = first_half_edge;
      const SubdivMesh::HalfEdge *v01 = v11->nextAdjacentEdge()->opposite();
      const SubdivMesh::HalfEdge *v00 = v01->prev();
      const SubdivMesh::HalfEdge *v10 = v00->prev();

      v[1][1] = (Vertex)vertices[v11->getStartVertexIndex()];
      v[1][0] = (Vertex)vertices[v10->getStartVertexIndex()];
      v[0][0] = (Vertex)vertices[v00->getStartVertexIndex()];
      v[0][1] = (Vertex)vertices[v01->getStartVertexIndex()];

      // quad(0,2)
      const SubdivMesh::HalfEdge *v12 = v11->next();
      const SubdivMesh::HalfEdge *v13 = v12->nextAdjacentEdge()->opposite();
      const SubdivMesh::HalfEdge *v03 = v13->prev();
      const SubdivMesh::HalfEdge *v02 = v03->prev();
      

      v[1][2] = (Vertex)vertices[v12->getStartVertexIndex()];
      v[1][3] = (Vertex)vertices[v13->getStartVertexIndex()];
      v[0][3] = (Vertex)vertices[v03->getStartVertexIndex()];
      v[0][2] = (Vertex)vertices[v02->getStartVertexIndex()];

      // quad(2,2)
      const SubdivMesh::HalfEdge *v22 = v12->next();
      const SubdivMesh::HalfEdge *v32 = v22->nextAdjacentEdge()->opposite();
      const SubdivMesh::HalfEdge *v33 = v32->prev();
      const SubdivMesh::HalfEdge *v23 = v33->prev();

      v[2][2] = (Vertex)vertices[v22->getStartVertexIndex()];
      v[3][2] = (Vertex)vertices[v32->getStartVertexIndex()];
      v[3][3] = (Vertex)vertices[v33->getStartVertexIndex()];
      v[2][3] = (Vertex)vertices[v23->getStartVertexIndex()];

      // quad(2,0)
      const SubdivMesh::HalfEdge *v21 = v22->next();
      const SubdivMesh::HalfEdge *v20 = v21->nextAdjacentEdge()->opposite();
      const SubdivMesh::HalfEdge *v30 = v20->prev();
      const SubdivMesh::HalfEdge *v31 = v30->prev();

      v[2][0] = (Vertex)vertices[v20->getStartVertexIndex()];
      v[3][0] = (Vertex)vertices[v30->getStartVertexIndex()];
      v[3][1] = (Vertex)vertices[v31->getStartVertexIndex()];
      v[2][1] = (Vertex)vertices[v21->getStartVertexIndex()];
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

      const Vertex curve0 = v_n[0] * v[0][0] + v_n[1] * v[1][0] + v_n[2] * v[2][0] + v_n[3] * v[3][0];
      const Vertex curve1 = v_n[0] * v[0][1] + v_n[1] * v[1][1] + v_n[2] * v[2][1] + v_n[3] * v[3][1];
      const Vertex curve2 = v_n[0] * v[0][2] + v_n[1] * v[1][2] + v_n[2] * v[2][2] + v_n[3] * v[3][2];
      const Vertex curve3 = v_n[0] * v[0][3] + v_n[1] * v[1][3] + v_n[2] * v[2][3] + v_n[3] * v[3][3];

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

#if !defined(__MIC__)
    typedef Vec3fa Vertex;
#else
    typedef Vec3fa_t Vertex;      
#endif

    Vec3fa f[2][2]; // need 16 + 4 = 20 control points

    GregoryPatch() {
      memset(this,0,sizeof(GregoryPatch));
    }

    GregoryPatch(const Vec3fa matrix[4][4],
		 const Vec3fa f_m[2][2]) 
      {
	for (size_t y=0;y<4;y++)
	  for (size_t x=0;x<4;x++)
	    v[y][x] = (Vertex)matrix[y][x];

	for (size_t y=0;y<2;y++)
	  for (size_t x=0;x<2;x++)
	    f[y][x] = (Vertex)f_m[y][x];
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

    Vec3fa initCornerVertex(const SubdivMesh::HalfEdge *const h,
			    const Vec3fa *const vertices)
    {
      CatmullClark1Ring ring;
      ring.init(h,vertices);
      return ring.getLimitVertex();
    }


#if 1
    Vec3fa initEdgeVertex(const SubdivMesh::HalfEdge *const h,
                          const Vec3fa *const vertices,
                          const Vec3fa &p_vtx)
    {
      CatmullClark1Ring ring;
      ring.init(h,vertices);
      Vec3fa tangent = ring.getLimitTangent();
      return 1.0f/3.0f * tangent + p_vtx;
    }
#else
    // the version from the paper seems to produce incorrect tangents

    Vec3fa initEdgeVertex(const SubdivMesh::HalfEdge *const h,
                          const Vec3fa *const vertices,
                          const Vec3fa &p_vtx)
    {
      Vertex q( 0.0f );
      const unsigned int valence = h->getEdgeValence();
      const float n = (float)valence;
      const float sigma = 1.0f / sqrtf((4.0f + cosf(M_PI/n) * cosf(M_PI/n)));

      const float b = (1.0f-sigma*cosf(M_PI/n));
      SubdivMesh::HalfEdge *p = (SubdivMesh::HalfEdge*)h;
      unsigned int i=0;
      do 
        {
          const Vertex m_i = p->getEdgeMidPointVertex(vertices);
          const Vertex c_i = p->getFaceMidPointVertex(vertices);
          const Vertex q_i = b * cosf((2.0f*M_PI*(float)i)/n) * m_i + 2.0f*sigma*cosf((2.0f*M_PI*(float)i+M_PI)/n) * c_i; 
          q += q_i;
          assert( p->hasOpposite() );
          p = p->opposite();
           /*! continue with next adjacent edge */
	  p = p->next();
          i++;
        } while( p != h);
      assert( i == n );
      q *= 2.0f/n;
      const float lambda = 1.0f/16.0f*(5.0f+cosf((2.0f*M_PI)/n)+cosf(M_PI/n)*sqrtf(18.0f+2.0f*cosf((2.0f*M_PI)/n)));
      return p_vtx + 2.0f/3.0f*lambda*q;
    }
#endif

    Vec3fa initFaceVertex(const SubdivMesh::HalfEdge *const h,
                          const Vec3fa *const vertices,
                          const Vec3fa &p_vtx,
			  const Vec3fa &e_p_vtx,
			  const Vec3fa &e_m_vtx,
			  const unsigned int valence0,
			  const unsigned int valence1,
			  const float sign = 1.0f)
    {
      const Vec3fa center_i       = h->getFaceMidPointVertex(vertices);
      const Vec3fa center_i_m_1   = h->opposite()->getFaceMidPointVertex(vertices);
      const Vec3fa midpoint_i_p_1 = h->prev()->getEdgeMidPointVertex(vertices);
      const Vec3fa midpoint_i_m_1 = h->opposite()->next()->getEdgeMidPointVertex(vertices);
      const Vec3fa r0 = 1.0f/3.0f * sign*(midpoint_i_p_1 - midpoint_i_m_1) + 2.0f/3.0f * sign*(center_i - center_i_m_1);

      const float d = 3.0f;
      const float c0 = cosf(2.0*M_PI/(float)valence0);
      const float c1 = cosf(2.0*M_PI/(float)valence1);

      return 1.0f / d * (c1 * p_vtx + (d - 2.0f*c0 - c1) * e_p_vtx + 2.0f*c0* e_m_vtx + r0);     
    }

    __forceinline void init(const SubdivMesh::HalfEdge *const first_half_edge,
			    const Vec3fa *const vertices)
    {
      const SubdivMesh::HalfEdge *const h_p0 = first_half_edge;
      const SubdivMesh::HalfEdge *const h_p1 = h_p0->next();
      const SubdivMesh::HalfEdge *const h_p2 = h_p1->next();
      const SubdivMesh::HalfEdge *const h_p3 = h_p2->next();

      const unsigned int valence_p0 = h_p0->getEdgeValence();
      const unsigned int valence_p1 = h_p1->getEdgeValence();
      const unsigned int valence_p2 = h_p2->getEdgeValence();
      const unsigned int valence_p3 = h_p3->getEdgeValence();

      const bool border_p0 = h_p0->hasIrregularEdge();
      const bool border_p1 = h_p0->hasIrregularEdge();
      const bool border_p2 = h_p0->hasIrregularEdge();
      const bool border_p3 = h_p0->hasIrregularEdge();

      /* DBG_PRINT( irreg_p0 ); */
      /* DBG_PRINT( irreg_p1 ); */
      /* DBG_PRINT( irreg_p2 ); */
      /* DBG_PRINT( irreg_p3 ); */

      if (border_p0 || 
	  border_p1 || 
	  border_p2 || 
	  border_p3)
	FATAL("initialization of gregory patches for border patches not yet implemented");

      p0() = initCornerVertex(h_p0,vertices);
      p1() = initCornerVertex(h_p1,vertices);
      p2() = initCornerVertex(h_p2,vertices);
      p3() = initCornerVertex(h_p3,vertices);

      const SubdivMesh::HalfEdge *const h_e0_p = h_p0;
      const SubdivMesh::HalfEdge *const h_e1_p = h_p1;
      const SubdivMesh::HalfEdge *const h_e2_p = h_p2;
      const SubdivMesh::HalfEdge *const h_e3_p = h_p3;

      e0_p() = initEdgeVertex(h_e0_p, vertices, p0());
      e1_p() = initEdgeVertex(h_e1_p, vertices, p1());
      e2_p() = initEdgeVertex(h_e2_p, vertices, p2());
      e3_p() = initEdgeVertex(h_e3_p, vertices, p3());

     

      
      if (h_p3->hasOpposite())
	{
	  const SubdivMesh::HalfEdge *const h_e0_m = h_p3->opposite();
	  e0_m() = initEdgeVertex(h_e0_m, vertices, p0());
	  f0_p() = initFaceVertex(h_e0_p,vertices,p0(),e0_p(),e0_m(),valence_p0,valence_p1);
	  f0_m() = initFaceVertex(h_e0_m,vertices,p0(),e0_m(),e0_p(),valence_p0,valence_p3,-1.0f); 
	}

      if (h_p0->hasOpposite())
	{
	  const SubdivMesh::HalfEdge *const h_e1_m = h_p0->opposite();
	  e1_m() = initEdgeVertex(h_e1_m, vertices, p1());
	  f1_p() = initFaceVertex(h_e1_p,vertices,p1(),e1_p(),e1_m(),valence_p1,valence_p2);
	  f1_m() = initFaceVertex(h_e1_m,vertices,p1(),e1_m(),e1_p(),valence_p1,valence_p0,-1.0f);
	}

      if (h_p1->hasOpposite())
	{
	  const SubdivMesh::HalfEdge *const h_e2_m = h_p1->opposite();
	  e2_m() = initEdgeVertex(h_e2_m, vertices, p2());
	  f2_p() = initFaceVertex(h_e2_p,vertices,p2(),e2_p(),e2_m(),valence_p2,valence_p3);
	  f2_m() = initFaceVertex(h_e2_m,vertices,p2(),e2_m(),e2_p(),valence_p2,valence_p1,-1.0f);
	}

      if (h_p2->hasOpposite())
	{
	  const SubdivMesh::HalfEdge *const h_e3_m = h_p2->opposite();
	  e3_m() = initEdgeVertex(h_e3_m, vertices, p3());
	  f3_p() = initFaceVertex(h_e3_p,vertices,p3(),e3_p(),e3_m(),valence_p3,valence_p2);
	  f3_m() = initFaceVertex(h_e3_m,vertices,p3(),e3_m(),e3_p(),valence_p3,valence_p0,-1.0f);
	}
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
 
      DBG_PRINT( valence_p0 );
      DBG_PRINT( valence_p1 );
      DBG_PRINT( valence_p2 );
      DBG_PRINT( valence_p3 );

      DBG_PRINT( f0_p() );
      DBG_PRINT( f1_p() );
      DBG_PRINT( f2_p() );
      DBG_PRINT( f3_p() );

      DBG_PRINT( f0_m() );
      DBG_PRINT( f1_m() );
      DBG_PRINT( f2_m() );
      DBG_PRINT( f3_m() );
#endif
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
	 matrix[y][x] = (Vertex)v[y][x];

     for (size_t y=0;y<2;y++)
       for (size_t x=0;x<2;x++)
	 f_m[y][x] = (Vertex)f[y][x];
   }

   __forceinline void exportBicubicBezierPatch( Vec3fa matrix[4][4], const float uu, const float vv ) const
   {
     for (size_t y=0;y<4;y++)
       for (size_t x=0;x<4;x++)
	 matrix[y][x] = (Vertex)v[y][x];

     if (uu == 0.0f || uu == 1.0f || vv == 0.0f || vv == 1.0f) return;

     // FIXME: merge u,v and extract after computation

     const Vertex F0 = (      uu  * f0_p() +       vv  * f0_m()) * 1.0f/(uu+vv);
     const Vertex F1 = ((1.0f-uu) * f1_m() +       vv  * f1_p()) * 1.0f/(1.0f-uu+vv);
     const Vertex F2 = ((1.0f-uu) * f2_p() + (1.0f-vv) * f2_m()) * 1.0f/(2.0f-uu-vv);
     const Vertex F3 = (      uu  * f3_m() + (1.0f-vv) * f3_p()) * 1.0f/(1.0f+uu-vv);

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

     const mic_f f0_p = (Vertex)matrix[1][1];
     const mic_f f0_m = (Vertex)f[0][0];

     const mic_f f1_p = (Vertex)matrix[1][2];
     const mic_f f1_m = (Vertex)f[0][1];

     const mic_f f2_p = (Vertex)matrix[2][2];
     const mic_f f2_m = (Vertex)f[1][1];

     const mic_f f3_p = (Vertex)matrix[2][1];
     const mic_f f3_m = (Vertex)f[1][0];

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
	(B0_u * (Vertex)matrix[0][0] + B1_u * (Vertex)matrix[0][1] + B2_u * (Vertex)matrix[0][2] + B3_u * (Vertex)matrix[0][3]) * B0_v + 
	(B0_u * (Vertex)matrix[1][0] + B1_u * F0 + B2_u * F1 + B3_u * (Vertex)matrix[1][3]) * B1_v + 
	(B0_u * (Vertex)matrix[2][0] + B1_u * F3 + B2_u * F2 + B3_u * (Vertex)matrix[2][3]) * B2_v + 
	(B0_u * (Vertex)matrix[3][0] + B1_u * (Vertex)matrix[3][1] + B2_u * (Vertex)matrix[3][2] + B3_u * (Vertex)matrix[3][3]) * B3_v; 
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

      const Vertex res = 
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

