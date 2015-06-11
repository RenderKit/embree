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

#include "../geometry.h"
#include "../scene_subdiv_mesh.h"

namespace embree
{
  struct __aligned(64) FinalQuad {
    Vec3fa vtx[4];
  };

  template<typename Vertex, typename Vertex_t = Vertex>
    struct __aligned(64) CatmullClark1RingT
  {
    static const size_t MAX_FACE_VALENCE = SubdivMesh::MAX_RING_FACE_VALENCE;
    static const size_t MAX_EDGE_VALENCE = SubdivMesh::MAX_RING_EDGE_VALENCE;
    static const size_t MAX_DEPTH_SUBDIVISION = 10;

    array_t<Vertex,MAX_EDGE_VALENCE> ring ; // FIXME: also store size in these arrays for more accurate checks
    array_t<float,MAX_FACE_VALENCE> crease_weight;
    
    int border_index;
    Vertex vtx;
    unsigned int face_valence;
    unsigned int edge_valence;
    float vertex_crease_weight;
    float vertex_level; // maximal level of all adjacent edges
    float edge_level; // level of first edge
    unsigned int eval_start_index;
    unsigned int eval_unique_identifier;
    bool noForcedSubdivision; // varying edge crease weight stitching fix

  public:
    CatmullClark1RingT () : eval_start_index(0), eval_unique_identifier(0) {}

    __forceinline bool hasBorder() const {
      return border_index != -1;
    }
    
    __forceinline const Vertex& front(size_t i) const {
      assert(edge_valence>i);
      return ring[i];
    }
    
    __forceinline const Vertex& back(size_t i) const {
      assert(i>0 && edge_valence>=i);
      return ring[edge_valence-i];
    }
    
    __forceinline bool has_last_face() const {
      return border_index != edge_valence-2;
    }

    __forceinline bool has_opposite_front(size_t i) const {
      return border_index != 2*i;
    }

    __forceinline bool has_opposite_back(size_t i) const {
      return border_index != (edge_valence-2-2*i);
    }
    
    __forceinline bool has_second_face() const {
      return (border_index == -1) || (border_index >= 4);
    }
    
    __forceinline Vertex regular_border_vertex_4() const 
    {
      assert(border_index != -1);
      assert(face_valence == 2 || face_valence == 3);
      if (face_valence == 3 && border_index == 4) return ring[4];
      else return 2.0f*vtx-ring[0];
    }

    __forceinline Vertex regular_border_vertex_5() const 
    {
      assert(border_index != -1);
      assert(face_valence == 2 || face_valence == 3);
      if (face_valence == 2) return 2.0f*vtx-ring[1];
      else if (border_index == 2) return 2.0f*ring[4]-ring[5];
      else { assert(border_index == 4); return 2.0f*ring[4]-ring[3]; }
    }

    __forceinline Vertex regular_border_vertex_6() const 
    {
      assert(border_index != -1);
      assert(face_valence == 2 || face_valence == 3);
      if (face_valence == 3 && border_index == 2) return ring[4];
      else return 2.0f*vtx-ring[2];
    }

    __forceinline BBox3fa bounds() const
    {
      BBox3fa bounds ( vtx );
      for (size_t i = 0; i<edge_valence ; i++)
	bounds.extend( ring[i] );
      return bounds;
    }

    template<typename LoadVertex>
      __forceinline void init(const SubdivMesh::HalfEdge* const h, const LoadVertex& load) 
    {
      noForcedSubdivision = true;
      border_index = -1;
      vtx = load(h);
      vertex_crease_weight = h->vertex_crease_weight;
      
      SubdivMesh::HalfEdge* p = (SubdivMesh::HalfEdge*) h;
      
      size_t i=0;
      unsigned min_vertex_index = (unsigned)-1;
      unsigned min_vertex_index_face = (unsigned)-1;
      edge_level = p->edge_level;
      vertex_level = 0.0f;

      do
      {
        vertex_level = max(vertex_level,p->edge_level);
        crease_weight[i/2] = p->hasOpposite() ? p->edge_crease_weight : float(inf);

        /* store first two vertices of face */
        p = p->next();
        ring[i++] = load(p);
        
        /* find minimal start vertex */
        unsigned vertex_index = p->getStartVertexIndex();
        if (vertex_index < min_vertex_index) { min_vertex_index = vertex_index; min_vertex_index_face = i>>1; }

        p = p->next();
        ring[i++] = load(p);
        p = p->next();
        crease_weight[i/2] = p->edge_crease_weight;
       
        /* continue with next face */
        if (likely(p->hasOpposite())) 
          p = p->opposite();
        
        /* if there is no opposite go the long way to the other side of the border */
        else
        {
          /* find minimal start vertex */
          unsigned vertex_index = p->getStartVertexIndex();
          if (vertex_index < min_vertex_index) { min_vertex_index = vertex_index; min_vertex_index_face = i>>1; }

          /*! mark first border edge and store dummy vertex for face between the two border edges */
          border_index = i;
          crease_weight[i/2] = inf; 
          ring[i++] = load(p);
          ring[i++] = vtx; // dummy vertex
          	  
          /*! goto other side of border */
          p = (SubdivMesh::HalfEdge*) h;
          while (p->hasOpposite()) 
            p = p->opposite()->next();
        }

      } while (p != h); 

      edge_valence = i;
      face_valence = i >> 1;
      eval_unique_identifier = min_vertex_index;
      eval_start_index = min_vertex_index_face;

      assert( hasValidPositions() );

    }
      
    __forceinline void subdivide(CatmullClark1RingT& dest) const
    {
      dest.noForcedSubdivision    = true;
      dest.edge_level             = 0.5f*edge_level;
      dest.vertex_level           = 0.5f*vertex_level;
      dest.face_valence           = face_valence;
      dest.edge_valence           = edge_valence;
      dest.border_index           = border_index;
      dest.vertex_crease_weight   = max(0.0f,vertex_crease_weight-1.0f);
      dest.eval_start_index       = eval_start_index;
      dest.eval_unique_identifier = eval_unique_identifier;

      assert(eval_start_index < edge_valence);

      /* calculate face points */
      Vertex_t S = Vertex_t(0.0f);
      for (size_t i=0; i<face_valence; i++) {
        ////////////////////////////////////////////////
        size_t face_index = i + eval_start_index;
        if (face_index >= face_valence) face_index -= face_valence;
        ////////////////////////////////////////////////

        size_t index0     = 2*face_index;
        size_t index1     = 2*face_index+1;
        size_t index2     = 2*face_index+2;
        if (index0 >= edge_valence) index0 -= edge_valence;
        if (index1 >= edge_valence) index1 -= edge_valence;
        if (index2 >= edge_valence) index2 -= edge_valence;
        assert(index0 < edge_valence);
        assert(index1 < edge_valence);
        assert(index2 < edge_valence);
        S += dest.ring[index1] = ((vtx + ring[index0]) + (ring[index1] + ring[index2])) * 0.25f;
      }
      
      /* calculate new edge points */
      size_t num_creases = 0;
      array_t<size_t,MAX_FACE_VALENCE> crease_id;
      Vertex_t C = Vertex_t(0.0f);
      for (size_t i=0; i<face_valence; i++)
      {
        ////////////////////////////////////////////////
        size_t face_index = i + eval_start_index;
        if (face_index >= face_valence) face_index -= face_valence;
        ////////////////////////////////////////////////
      
        //size_t face_index = i;
        size_t index      = 2*face_index;
        size_t prev_index = face_index == 0 ? edge_valence-1 : 2*face_index-1;
        size_t next_index = 2*face_index+1;

        const Vertex_t v = vtx + ring[index];
        const Vertex_t f = dest.ring[prev_index] + dest.ring[next_index];
        S += ring[index];
        dest.crease_weight[face_index] = max(crease_weight[face_index]-1.0f,0.0f);
        //dest.crease_weight[i] = crease_weight[face_index] < 1.0f ? 0.0f : 0.5f*crease_weight[face_index];
	dest.noForcedSubdivision &= crease_weight[face_index] == 0.0f || crease_weight[face_index] > 1.0f;
        
        /* fast path for regular edge points */
        if (likely(crease_weight[face_index] <= 0.0f)) {
          dest.ring[index] = (v+f) * 0.25f;
        }
        
        /* slower path for hard edge rule */
        else {
          C += ring[index]; crease_id[num_creases++] = face_index;
          dest.ring[index] = v*0.5f;
	  
          /* even slower path for blended edge rule */
          if (unlikely(crease_weight[face_index] < 1.0f)) {
            const float w0 = crease_weight[face_index], w1 = 1.0f-w0;
            dest.ring[index] = w1*((v+f)*0.25f) + w0*(v*0.5f);
          }
        }
      }
      
      /* compute new vertex using smooth rule */
      const float inv_face_valence = 1.0f / (float)face_valence;
      const Vertex_t v_smooth = (Vertex_t)(S*inv_face_valence + (float(face_valence)-2.0f)*vtx)*inv_face_valence;
      dest.vtx = v_smooth;
      
      /* compute new vertex using vertex_crease_weight rule */
      if (unlikely(vertex_crease_weight > 0.0f)) 
      {
        if (vertex_crease_weight >= 1.0f) {
          dest.vtx = vtx;
        } else {
          const float t0 = vertex_crease_weight, t1 = 1.0f-t0;
          dest.vtx = t0*vtx + t1*v_smooth;
        }
        return;
      }
      
      if (likely(num_creases <= 1))
        return;
      
      /* compute new vertex using crease rule */
      if (likely(num_creases == 2)) {
        const Vertex_t v_sharp = (Vertex_t)(C + 6.0f * vtx) * (1.0f / 8.0f);
        const float crease_weight0 = crease_weight[crease_id[0]];
        const float crease_weight1 = crease_weight[crease_id[1]];
        dest.vtx = v_sharp;
        dest.crease_weight[crease_id[0]] = max(0.25f*(3.0f*crease_weight0 + crease_weight1)-1.0f,0.0f);
        dest.crease_weight[crease_id[1]] = max(0.25f*(3.0f*crease_weight1 + crease_weight0)-1.0f,0.0f);
	dest.noForcedSubdivision = (dest.crease_weight[crease_id[0]] != 0.0f) || (dest.crease_weight[crease_id[1]] != 0.0f);
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
    
    __forceinline bool isRegular() const 
    {
      if (border_index == -1) {
	if (face_valence == 4) return true;
      } else {
	if (face_valence < 4) return true;
      }
      return false;
    }

     /* returns true if the vertex can be part of a dicable B-Spline patch or is a final Quad */
    __forceinline bool isRegularOrFinal(const size_t depth) const 
    {
      if (depth < MAX_DEPTH_SUBDIVISION)
      {
	if (border_index == -1) 
	{
	  if (face_valence != 4)
	    return false;
	  if (vertex_crease_weight > 0.0f) 
	    return false;
	} 
	else {
	  if (face_valence == 2 && vertex_crease_weight > 1E5); // FIXME: use inf
	  else if (face_valence == 3 && vertex_crease_weight == 0.0f);
	  else return false;
	}

	for (size_t i=1; i<face_valence; i++)
	  if (crease_weight[i] > 0.0f && (2*i != border_index) && (2*(i-1) != border_index)) 
	    return false;

	if (crease_weight[0] > 0.0f && (2*(face_valence-1) != border_index)) 
	  return false;
	
	if (!noForcedSubdivision)
	  return false;
      }
      return true;
    }

    /* returns true if the vertex can be part of a dicable B-Spline patch or is a final Quad */
    __forceinline bool isRegularOrFinal2(const size_t depth) const 
    {
      //if (depth < 2)
      if (vertex_level > 1.0f) 
      {
	if (border_index == -1) 
	{
	  if (face_valence != 4)
	    return false;
	  if (vertex_crease_weight > 0.0f) 
	    return false;
	} 
	else {
	  if (face_valence == 2 && vertex_crease_weight > 1E5); // FIXME: use inf
	  else if (face_valence == 3 && vertex_crease_weight == 0.0f); // FIXME: document
	  else return false;
	}

	for (size_t i=1; i<face_valence; i++)
	  if (crease_weight[i] > 0.0f && (2*i != border_index) && (2*(i-1) != border_index)) 
	    return false;

	if (crease_weight[0] > 0.0f && (2*(face_valence-1) != border_index)) 
	  return false;
	
	if (!noForcedSubdivision)
	  return false;
      }
      return true;
    }

    /* returns true if the vertex can be part of a dicable gregory patch (using gregory patches) */
    __forceinline bool isGregory() const 
    {
      if (vertex_crease_weight == (float)pos_inf) 
        return true; // FIXME: wrong !!!!!

      if (vertex_crease_weight > 0.0f) 
        return false;
      
      for (size_t i=1; i<face_valence; i++) 
        if (crease_weight[i] > 0.0f && (2*i != border_index) && (2*(i-1) != border_index)) 
          return false;
      
      if (crease_weight[0] > 0.0f && (2*(face_valence-1) != border_index)) 
        return false;
      
      if (!noForcedSubdivision)
        return false;

      return true;
    }

    /* returns true if the vertex can be part of a dicable gregory patch (using gregory patches) */
    __forceinline bool isGregoryOrFinal(const size_t depth) const 
    {
      if (depth < MAX_DEPTH_SUBDIVISION && vertex_level > 1.0f) return isGregory();
      return true;
    }
    
    __forceinline Vertex ksum(Vertex_t &sum, Vertex_t &c, const Vertex_t &i) const
    {
      Vertex_t y = i - c;
      Vertex_t t = sum + y;
      c = (t - sum) - y;
      return t;
    }

    /* computes the limit vertex */
    __forceinline Vertex getLimitVertex() const
    {
      /* FIXME: is this correct ? */ 
      if (unlikely(std::isinf(vertex_crease_weight)))
        return vtx;

      /* border vertex rule */
      if (unlikely(border_index != -1))
      {
	//if (unlikely(std::isinf(vertex_crease_weight)))
        //return vtx;
	
	const unsigned int second_border_index = border_index+2 >= edge_valence ? 0 : border_index+2;
	return (4.0f * vtx + (ring[border_index] + ring[second_border_index])) * 1.0f/6.0f;
      }
      
      Vertex_t F( 0.0f );
      Vertex_t E( 0.0f );
      
      //Vertex_t c_F ( 0.0f );
      //Vertex_t c_E ( 0.0f );

      //F = ksum(F,c_F,ring[2*i+1]);
      //E = ksum(E,c_E,ring[2*i]);

      assert(eval_start_index < face_valence);

      for (size_t i=0; i<face_valence; i++) {
        ////////////////////////////////////////////////
        size_t index = i+eval_start_index;
        if (index >= face_valence) index -= face_valence;
        ////////////////////////////////////////////////

        F += ring[2*index+1];
        E += ring[2*index];
      }

      const float n = (float)face_valence;
      return (Vertex_t)(n*n*vtx+4.0f*E+F) / ((n+5.0f)*n);      
    }
    
    /* gets limit tangent in the direction of egde vtx -> ring[0] */
    __forceinline Vertex getLimitTangent() const 
    {
      if (unlikely(std::isinf(vertex_crease_weight)))
        return ring[0] - vtx;
      
      /* border vertex rule */
      if (unlikely(border_index != -1))
      {
	//if (unlikely(std::isinf(vertex_crease_weight)))
        //return ring[0] - vtx;
	
	if (border_index != edge_valence-2 && face_valence != 2) {
	  return ring[0] - vtx; 
	}
	else
	{
	  const unsigned int second_border_index = border_index+2 >= edge_valence ? 0 : border_index+2;
	  return (ring[second_border_index] - ring[border_index]) * 0.5f;
	}
      }
      
      Vertex_t alpha( 0.0f );
      Vertex_t beta ( 0.0f );
      
      const float n = (float)face_valence;
      //const float delta = 1.0f / sqrtf(4.0f + cos(M_PI/n)*cos(M_PI/n));
      //const float c0 = 2.0f/n * delta;
      //const float c1 = 1.0f/n * (1.0f - delta*cosf(M_PI/n));
      const float c0 = 1.0f/n * 1.0f / sqrtf(4.0f + cosf(M_PI/n)*cosf(M_PI/n));  
      const float c1 = (1.0f/n + cosf(M_PI/n) * c0); // FIXME: plus or minus


      //Vertex_t c_alpha( 0.0f );
      //Vertex_t c_beta ( 0.0f );

      assert(eval_start_index < face_valence);

      for (size_t i=0; i<face_valence; i++)
      {
        ////////////////////////////////////////////////
        size_t index = i+eval_start_index;
        if (index >= face_valence) index -= face_valence;
        ////////////////////////////////////////////////

	const float a = c1 * cosf(2.0f*M_PI*index/n);
	const float b = c0 * cosf((2.0f*M_PI*index+M_PI)/n); // FIXME: factor of 2 missing?
        //alpha = ksum(alpha,c_alpha,a*ring[2*i]);
        //beta  = ksum(beta ,c_beta ,b*ring[2*i+1]);
	alpha +=  a * ring[2*index];
	beta  +=  b * ring[2*index+1];
      }

      return alpha + beta;
    }
    
    /* gets limit tangent in the direction of egde vtx -> ring[edge_valence-2] */
    __forceinline Vertex getSecondLimitTangent() const 
    {
      if (unlikely(std::isinf(vertex_crease_weight)))
        return ring[2] - vtx;
 
      /* border vertex rule */
      if (unlikely(border_index != -1))
      {
        //if (unlikely(std::isinf(vertex_crease_weight)))
        //return ring[2] - vtx;
        
        //if (border_index == 0 && face_valence != 2) {
        if (border_index == edge_valence-2 && face_valence != 2) {
          return ring[2] - vtx;
        }
        else {
          const unsigned int second_border_index = border_index+2 >= edge_valence ? 0 : border_index+2;
          return (ring[border_index] - ring[second_border_index]) * 0.5f;
        }
      }
      
      Vertex_t alpha( 0.0f );
      Vertex_t beta ( 0.0f );
      const float n = (float)face_valence;
      //const float delta = 1.0f / sqrtf(4.0f + cos(M_PI/n)*cos(M_PI/n));
      //const float c0 = 2.0f/n * delta;
      //const float c1 = 1.0f/n * (1.0f - delta*cosf(M_PI/n));
      const float c0 = 1.0f/n * 1.0f / sqrtf(4.0f + cosf(M_PI/n)*cosf(M_PI/n));  
      const float c1 = (1.0f/n + cosf(M_PI/n) * c0);

      //Vertex_t c_alpha( 0.0f );
      //Vertex_t c_beta ( 0.0f );

      assert(eval_start_index < face_valence);

      for (size_t i=0; i<face_valence; i++)
      {
        ////////////////////////////////////////////////
        size_t index = i+eval_start_index;
        if (index >= face_valence) index -= face_valence;
        ////////////////////////////////////////////////

        size_t prev_index = index == 0 ? face_valence-1 : index-1; // need to be bit-wise exact in cosf eval

	const float a = c1 * cosf(2.0f*M_PI*(float(prev_index))/n);
	const float b = c0 * cosf((2.0f*M_PI*(float(prev_index))+M_PI)/n);
        //alpha = ksum(alpha,c_alpha,a*ring[2*i]);
        //beta  = ksum(beta ,c_beta ,b*ring[2*i+1]);
	alpha += a * ring[2*index];
	beta  += b * ring[2*index+1];
      }

      return alpha + beta;      
    }

    /* gets surface normal */
    const Vertex getNormal() const  {
      return cross(getSecondLimitTangent(),getLimitTangent());
    }
    
    /* returns center of the n-th quad in the 1-ring */
    __forceinline Vertex getQuadCenter(const size_t index) const
    {
      const Vertex_t &p0 = vtx;
      const Vertex_t &p1 = ring[2*index+0];
      const Vertex_t &p2 = ring[2*index+1];
      const Vertex_t &p3 = index == face_valence-1 ? ring[0] : ring[2*index+2];
      const Vertex p = (p0+p1+p2+p3) * 0.25f;
      return p;
    }
    
    /* returns center of the n-th edge in the 1-ring */
    __forceinline Vertex getEdgeCenter(const size_t index) const {
      return (vtx + ring[index*2]) * 0.5f;
    }

    bool hasValidPositions() const
    {
      for (size_t i=0; i<edge_valence; i++) {
        if (!isvalid(ring[i]))
          return false;
      }	
      return true;
    }

    friend __forceinline std::ostream &operator<<(std::ostream &o, const CatmullClark1RingT &c)
    {
      o << "vtx " << c.vtx << " size = " << c.edge_valence << ", " << 
	"hard_edge = " << c.border_index << ", face_valence " << c.face_valence << 
	", edge_level = " << c.edge_level << ", vertex_level = " << c.vertex_level << ", eval_start_index: " << c.eval_start_index << ", ring: " << std::endl;
      
      for (size_t i=0; i<c.edge_valence; i++) {
        o << i << " -> " << c.ring[i];
        if (i % 2 == 0) o << " crease = " << c.crease_weight[i/2];
        o << std::endl;
      }
      return o;
    } 
  };
  
  template<typename Vertex, typename Vertex_t = Vertex>
    struct __aligned(64) GeneralCatmullClark1RingT
  {
    typedef CatmullClark1RingT<Vertex,Vertex_t> CatmullClark1Ring;
    static const size_t MAX_FACE_VALENCE = SubdivMesh::MAX_RING_FACE_VALENCE;
    static const size_t MAX_EDGE_VALENCE = SubdivMesh::MAX_RING_EDGE_VALENCE;
    
    struct Face 
    {
      __forceinline Face() {}
      __forceinline Face (int size, float crease_weight)
        : size(size), crease_weight(crease_weight) {}

      // FIXME: add member that returns total number of vertices

      int size;              // number of vertices-2 of nth face in ring
      float crease_weight;
    };

    Vertex vtx;
    array_t<Vertex,MAX_EDGE_VALENCE> ring; 
    array_t<Face,MAX_FACE_VALENCE> faces;
    unsigned int face_valence;
    unsigned int edge_valence;
    int border_face;
    float vertex_crease_weight;
    float vertex_level;                      //!< maximal level of adjacent edges
    float edge_level; // level of first edge
    bool only_quads;  // true if all faces are quads
    unsigned int eval_start_face_index;
    unsigned int eval_start_vertex_index;
    unsigned int eval_unique_identifier;


    GeneralCatmullClark1RingT() 
      : eval_start_face_index(0), eval_start_vertex_index(0), eval_unique_identifier(0) {}
    
    __forceinline bool has_last_face() const {
      return border_face != face_valence-1;
    }
    
    __forceinline bool has_second_face() const {
      return (border_face == -1) || (border_face >= 2);
    }

    bool hasValidPositions() const
    {
      for (size_t i=0; i<edge_valence; i++) {
        if (!isvalid(ring[i]))
          return false;
      }	
      return true;
    }

    template<typename LoadVertex>
      __forceinline void init(const SubdivMesh::HalfEdge* const h, const LoadVertex& load)
    {
      only_quads = true;
      border_face = -1;
      vtx = load(h);
      vertex_crease_weight = h->vertex_crease_weight;
      SubdivMesh::HalfEdge* p = (SubdivMesh::HalfEdge*) h;
      
      size_t e=0, f=0;
      unsigned min_vertex_index = (unsigned)-1;
      unsigned min_vertex_index_face = (unsigned)-1;
      unsigned min_vertex_index_vertex = (unsigned)-1;
      edge_level = p->edge_level;
      vertex_level = 0.0f;
      do 
      {
        SubdivMesh::HalfEdge* p_prev = p->prev();
        SubdivMesh::HalfEdge* p_next = p->next();
        const float crease_weight = p->hasOpposite() ? p->edge_crease_weight : float(inf);
        vertex_level = max(vertex_level,p->edge_level);

        /* find minimal start vertex */
        unsigned vertex_index = p_next->getStartVertexIndex();
        if (vertex_index < min_vertex_index) { min_vertex_index = vertex_index; min_vertex_index_face = f; min_vertex_index_vertex = e; }

	/* store first N-2 vertices of face */
	size_t vn = 0;
        for (p = p_next; p!=p_prev; p=p->next()) {
          ring[e++] = load(p);
          vn++;
	}
	faces[f++] = Face(vn,crease_weight);
	only_quads &= (vn == 2);
	
        /* continue with next face */
        if (likely(p->hasOpposite())) 
          p = p->opposite();
        
        /* if there is no opposite go the long way to the other side of the border */
        else
        {
          /* find minimal start vertex */
          unsigned vertex_index = p->getStartVertexIndex();
          if (vertex_index < min_vertex_index) { min_vertex_index = vertex_index; min_vertex_index_face = f; min_vertex_index_vertex = e; }

          /*! mark first border edge and store dummy vertex for face between the two border edges */
          border_face = f;
	  faces[f++] = Face(2,inf); 
          ring[e++] = load(p);
          ring[e++] = vtx; // dummy vertex
	  
          /*! goto other side of border */
          p = (SubdivMesh::HalfEdge*) h;
          while (p->hasOpposite()) 
            p = p->opposite()->next();
        }
	
      } while (p != h); 
      
      edge_valence = e;
      face_valence = f;
      eval_unique_identifier = min_vertex_index;
      eval_start_face_index = min_vertex_index_face;
      eval_start_vertex_index = min_vertex_index_vertex;

      ////////////////////////////
      //vertex_crease_weight = inf;
      ////////////////////////////

      assert( hasValidPositions() );
    }
    
    __forceinline void subdivide(CatmullClark1Ring& dest) const
    {
      dest.noForcedSubdivision = true;
      dest.edge_level = 0.5f*edge_level;
      dest.vertex_level = 0.5f*vertex_level;
      dest.face_valence = face_valence;
      dest.edge_valence = 2*face_valence;
      dest.border_index = border_face == -1 ? -1 : 2*border_face; // FIXME:
      dest.vertex_crease_weight    = max(0.0f,vertex_crease_weight-1.0f);
      dest.eval_start_index        = eval_start_face_index;
      dest.eval_unique_identifier  = eval_unique_identifier;
      assert(dest.face_valence <= CatmullClark1Ring::MAX_FACE_VALENCE);

      /* calculate face points */
      Vertex_t S = Vertex_t(0.0f);
      for (size_t face=0, v=eval_start_vertex_index; face<face_valence; face++) {
        ////////////////////////////////////////////////
        size_t f = (face + eval_start_face_index)%face_valence;
        ////////////////////////////////////////////////

        Vertex_t F = vtx;
        for (size_t k=v; k<=v+faces[f].size; k++) F += ring[k%edge_valence]; // FIXME: optimize
        S += dest.ring[2*f+1] = F/float(faces[f].size+2);
        v+=faces[f].size;
        ////////////////////////////////////////////////        
        v%=edge_valence;
        ////////////////////////////////////////////////
      }
      
      /* calculate new edge points */
      size_t num_creases = 0;
      array_t<size_t,MAX_FACE_VALENCE> crease_id;
      Vertex_t C = Vertex_t(0.0f);
      for (size_t face=0, j=eval_start_vertex_index; face<face_valence; face++)
      {
        ////////////////////////////////////////////////
        size_t i = (face + eval_start_face_index)%face_valence;
        ////////////////////////////////////////////////
        
        const Vertex_t v = vtx + ring[j];
        Vertex_t f = dest.ring[2*i+1];
        if (i == 0) f += dest.ring[dest.edge_valence-1]; 
        else        f += dest.ring[2*i-1];
        S += ring[j];
        dest.crease_weight[i] = max(faces[i].crease_weight-1.0f,0.0f);
        
        /* fast path for regular edge points */
        if (likely(faces[i].crease_weight <= 0.0f)) {
          dest.ring[2*i] = (v+f) * 0.25f;
        }
        
        /* slower path for hard edge rule */
        else {
          C += ring[j]; crease_id[num_creases++] = i;
          dest.ring[2*i] = v*0.5f;
	  
          /* even slower path for blended edge rule */
          if (unlikely(faces[i].crease_weight < 1.0f)) {
            const float w0 = faces[i].crease_weight, w1 = 1.0f-w0;
            dest.ring[2*i] = w1*((v+f)*0.25f) + w0*(v*0.5f);
          }
        }
        j+=faces[i].size;
        ////////////////////////////////////////////////                
        j%=edge_valence;
        ////////////////////////////////////////////////                
      }
      
      /* compute new vertex using smooth rule */
      const float inv_face_valence = 1.0f / (float)face_valence;
      const Vertex_t v_smooth = (Vertex_t)(S*inv_face_valence + (float(face_valence)-2.0f)*vtx)*inv_face_valence;
      dest.vtx = v_smooth;
      
      /* compute new vertex using vertex_crease_weight rule */
      if (unlikely(vertex_crease_weight > 0.0f)) 
      {
        if (vertex_crease_weight >= 1.0f) {
          dest.vtx = vtx;
        } else {
          const float t0 = vertex_crease_weight, t1 = 1.0f-t0;
          dest.vtx = t0*vtx + t1*v_smooth;
        }
        return;
      }
      
      if (likely(num_creases <= 1))
        return;
      
      /* compute new vertex using crease rule */
      if (likely(num_creases == 2)) {
        const Vertex_t v_sharp = (Vertex_t)(C + 6.0f * vtx) * (1.0f / 8.0f);
        const float crease_weight0 = faces[crease_id[0]].crease_weight;
        const float crease_weight1 = faces[crease_id[1]].crease_weight;
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

    void convert(CatmullClark1Ring& dst) const
    {
      assert(only_quads);
      dst.edge_level = edge_level;
      dst.vertex_level = vertex_level;
      dst.vtx = vtx;
      dst.face_valence = face_valence;
      dst.edge_valence = 2*face_valence;
      dst.border_index = border_face == -1 ? -1 : 2*border_face;
      for (size_t i=0; i<face_valence; i++) 
	dst.crease_weight[i] = faces[i].crease_weight;
      dst.vertex_crease_weight = vertex_crease_weight;
      dst.noForcedSubdivision = true;
      for (size_t i=0; i<edge_valence; i++) dst.ring[i] = ring[i];

      //dst.updateEvalStartIndex();
      dst.eval_start_index = eval_start_face_index;
      dst.eval_unique_identifier = eval_unique_identifier;

      assert( dst.hasValidPositions() );
    }

    
    void computeGregoryPatchEdgePoints(Vertex &p0,
                                       Vertex &e0_plus,
                                       Vertex &e0_minus,
                                       Vertex &r0_plus,
                                       Vertex &r0_minus) const
    {
      Vertex cm_ring[2*MAX_FACE_VALENCE];
      Vertex first_border_vertex, second_border_vertex;

      const size_t border_index = border_face == -1 ? -1 : 2*border_face; 

      /* calculate face centroids and edge midpoints */
      for (size_t f=0, v=0; f<face_valence; f++) {
        Vertex_t F = vtx;
        for (size_t k=v; k<=v+faces[f].size; k++)
          F += ring[k%edge_valence];
        cm_ring[2*f+1] = F/float(faces[f].size+2);
	cm_ring[2*f] = 0.5f*(vtx+ring[v]);
	if (unlikely(2*f == border_index))
	  {
	    first_border_vertex  = ring[v];
	    second_border_vertex = ring[(v+faces[f].size) % edge_valence];
	  }
        v+=faces[f].size;
      }

      const float N = face_valence;

      /* limit Vertex p0 */
      if (unlikely(std::isinf(vertex_crease_weight)))
	p0 = vtx;
      else if (unlikely(border_index != -1))
	p0 = (4.0f * vtx + first_border_vertex + second_border_vertex) * 1.0f/6.0f;
      else
	{
	  p0 = Vertex( zero );
	  for (size_t face=0; face<face_valence; face++)
	    {
	      size_t f = (face + eval_start_face_index)%face_valence;
	      p0 += cm_ring[2*f] + cm_ring[2*f+1];
	    }
	  p0 *= 4.0f / (N * (N + 5.0f));
	  p0 += (N - 3.0f) / (N + 5.0f) * vtx;
	}

      const float sigma = 1.0f / sqrtf(4.0f + cosf(M_PI/N) * cosf(M_PI/N));  
      const float alpha = 1.0f/16.0f * (5.0f + cosf(2.0f*M_PI/N) + cosf(M_PI/N) * sqrtf(18.0f+2.0f*cosf(2.0f*M_PI/N)));

      /* tangent q0 */
      Vertex q0( zero );
      if (unlikely(std::isinf(vertex_crease_weight)))
        q0 = ring[0] - vtx;
      else if (unlikely(border_index != -1))
	{
	  if (border_index == edge_valence-2 || face_valence == 2)
          { 
	    q0 = ring[0] - vtx;
          }
	  else
          {
	    q0 = (second_border_vertex - first_border_vertex) * 0.5f;
          }
	}
      else
	{
	  for (size_t face=0; face<face_valence; face++)
	    {
	      const size_t index = (face + eval_start_face_index)%face_valence;
	      const Vertex m_i = cm_ring[2*index+0];
	      const Vertex c_i = cm_ring[2*index+1];        
	      q0 += (1.0f - sigma * cosf(M_PI/N)) * cosf((2.0f*M_PI*index)/N) * m_i +
	            (2.0f * sigma * cosf((2.0f*M_PI*index+M_PI)/N)) * c_i;
	    }
	  q0 *= 2.0f / N; 
	}

      /* tangent q1 */
      Vertex q1( zero );
      if (unlikely(std::isinf(vertex_crease_weight)))
        q1 = ring[faces[0].size] - vtx;
      else if (unlikely(border_index != -1))
	{
	  if (border_index == 2 || face_valence == 2) 
          {
	    q1 = ring[faces[0].size] - vtx;
          }
	  else
          {
	    q1 = (first_border_vertex - second_border_vertex) * 0.5f;
          }
	}
      else
	{
	  for (ssize_t face=0; face<face_valence; face++)
	    {
	      const ssize_t f = (face + eval_start_face_index)%face_valence;

	      const ssize_t index1 = (f-1) % face_valence;
	      const ssize_t index0 = f;
	      const Vertex m_i = cm_ring[2*index0+0];
	      const Vertex c_i = cm_ring[2*index0+1];        
	      q1 += \
	      (1.0f - sigma * cosf(M_PI/N)) * cosf((2.0f*M_PI*index1)/N) * m_i + 
	      (2.0f * sigma * cosf((2.0f*M_PI*index1+M_PI)/N)) * c_i;	      
	    }
	  q1 *= 2.0f / N;
	}
      
      /* e0_plus */
      //e0_plus  = p0 + 2.0f/3.0f * alpha * q0;
      e0_plus  = p0 + 1.0f/3.0f * q0;


      /* e0_minus */
      //e0_minus = p0 + 2.0f/3.0f * alpha * q1;
      e0_minus = p0 + 1.0f/3.0f * q1;

      /* r0_plus, r0_minus */
      const Vertex e_i      = cm_ring[0];
      const Vertex c_i_m_1  = cm_ring[1];
      const Vertex e_i_m_1  = cm_ring[2];
      
      Vertex c_i, e_i_p_1;
      const bool hasHardEdge =			\
      std::isinf(vertex_crease_weight) &&
      std::isinf(faces[0].crease_weight);
                
      if (unlikely(border_index == face_valence-2) || hasHardEdge)
        {
          /* mirror quad center and edge mid-point */
          c_i     = c_i_m_1 + 2 * (e_i - c_i_m_1);
          e_i_p_1 = e_i_m_1 + 2 * (vtx - e_i_m_1);
        }
      else
        {
          c_i     = cm_ring[2*(face_valence-1)+1];
          e_i_p_1 = cm_ring[2*(face_valence-1)+0];
        }
      
      Vertex c_i_m_2, e_i_m_2;
      if (unlikely(border_index == 2 || face_valence == 2 || hasHardEdge))
      {
        /* mirror quad center and edge mid-point */
        c_i_m_2  = c_i_m_1 + 2 * (e_i_m_1 - c_i_m_1);
        e_i_m_2  = e_i + 2 * (vtx - e_i);	  
      }
      else
      {
        c_i_m_2  = cm_ring[2*1+1];
        e_i_m_2  = cm_ring[2*2+0];
      }      
      

      r0_plus  = 1.0f/3.0f * (e_i_m_1 - e_i_p_1) + 2.0f/3.0f * (c_i_m_1 - c_i);      
      r0_minus = 1.0f/3.0f * (e_i     - e_i_m_2) + 2.0f/3.0f * (c_i_m_1 - c_i_m_2);
    }
    
    friend __forceinline std::ostream &operator<<(std::ostream &o, const GeneralCatmullClark1RingT &c)
    {
      o << "vtx " << c.vtx << " size = " << c.edge_valence << ", border_face = " << c.border_face << ", " << " face_valence = " << c.face_valence << 
	", edge_level = " << c.edge_level << ", vertex_level = " << c.vertex_level << ", ring: " << std::endl;
      for (size_t v=0, f=0; f<c.face_valence; v+=c.faces[f++].size) {
        for (size_t i=v; i<v+c.faces[f].size; i++) {
          o << i << " -> " << c.ring[i];
          if (i == v) o << " crease = " << c.faces[f].crease_weight;
          o << std::endl;
        }
      }
      return o;
    } 

    friend __forceinline bool equalRingEval(CatmullClark1Ring& source, CatmullClark1Ring& dest) 
    {
      if (source.face_valence != dest.face_valence) return false;
      if (source.edge_valence != dest.edge_valence) return false;
      
      size_t start_index_source = 2 * source.eval_start_index;
      size_t start_index_dest   = 2 * dest.eval_start_index;
      
      for (size_t i=0;i<source.edge_valence;i++)
        {
          size_t index_source = (start_index_source + i) % source.edge_valence;
          size_t index_dest   = (start_index_dest   + i) % source.edge_valence;          
          if ( source.ring[index_source] != dest.ring[index_dest] )
            {
              PRINT(source.eval_start_index);
              PRINT(dest.eval_start_index);              
              PRINT(index_source);
              PRINT(index_dest);
              PRINT(source.ring[index_source]);
              PRINT(dest.ring[index_dest]);
              return false;              
            }
        }
      return true;
    }

  };  
}
