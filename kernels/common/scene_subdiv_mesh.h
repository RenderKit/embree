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
#include "common/buffer.h"


namespace embree
{
  class SubdivMesh : public Geometry
  {
  public:
    SubdivMesh(Scene* parent, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, 
               size_t numCreases, size_t numCorners, size_t numHoles, size_t numTimeSteps);
    ~SubdivMesh();

    void enabling();
    void disabling();
    void setMask (unsigned mask);
    void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride);
    void* map(RTCBufferType type);
    void unmap(RTCBufferType type);
    void setUserData (void* ptr, bool ispc);
    void immutable ();
    bool verify ();
    void setDisplacementFunction (RTCDisplacementFunc func, const RTCBounds& bounds);

    unsigned int mask;                //!< for masking out geometry
    unsigned int numTimeSteps;        //!< number of time steps (1 or 2)  

    size_t numFaces;                  //!< number of faces
    size_t numEdges;                  //!< number of edges
    size_t numVertices;               //!< number of vertices

    RTCDisplacementFunc displFunc;    //!< displacement function
    BBox3fa             displBounds;  //!< bounds for displacement

    size_t size() const { return numFaces; };

    
    class HalfEdge
    {
      friend class SubdivMesh;
    public:

      HalfEdge () 
        : vtx_index(-1), next_half_edge_ofs(0), prev_half_edge_ofs(0), opposite_half_edge_ofs(0), edge_crease_weight(0), 
          vertex_crease_weight(0), edge_level(0), align(0) {}

      __forceinline bool hasOpposite() const { return opposite_half_edge_ofs != 0; }

      __forceinline       HalfEdge* next()       { assert( next_half_edge_ofs != 0 ); return &this[next_half_edge_ofs]; }
      __forceinline const HalfEdge* next() const { assert( next_half_edge_ofs != 0 ); return &this[next_half_edge_ofs]; }

      __forceinline       HalfEdge* prev()       { assert( prev_half_edge_ofs != 0 ); return &this[prev_half_edge_ofs]; }
      __forceinline const HalfEdge* prev() const { assert( prev_half_edge_ofs != 0 ); return &this[prev_half_edge_ofs]; }

      __forceinline       HalfEdge* opposite()       { assert( opposite_half_edge_ofs != 0 ); return &this[opposite_half_edge_ofs]; }
      __forceinline const HalfEdge* opposite() const { assert( opposite_half_edge_ofs != 0 ); return &this[opposite_half_edge_ofs]; }

      __forceinline       HalfEdge* rotate()       { return opposite()->next(); }
      __forceinline const HalfEdge* rotate() const { return opposite()->next(); }

      __forceinline unsigned int getStartVertexIndex() const { return vtx_index; }
      __forceinline unsigned int getEndVertexIndex  () const { return next()->vtx_index; }

      __forceinline bool hasIrregularEdge() const 
      {
	HalfEdge *p = (HalfEdge*)this;
	do {
	  if (unlikely(!p->hasOpposite()))
	    return true;
	  p = p->opposite();
	  p = p->next();
	} while( p != this);

        return false;
      }

      __forceinline unsigned int getEdgeValence() const 
      {
	unsigned int i=0;
	HalfEdge *p = (HalfEdge*)this;
	bool foundEdge = false;

	do {
	  i++;
	  if (unlikely(!p->hasOpposite()))
	    {
	      foundEdge = true;
	      break;
	    }

	  p = p->opposite();
	  p = p->next();
	} while( p != this);

	if (unlikely(foundEdge))
	  {
	    p = (HalfEdge*)this;
	    p = p->prev();
	    i++;
	    while(p->hasOpposite())
	      {
		p = p->opposite();
		p = p->prev();	      
		i++;
	      }
	  }

	return i;
      };

      __forceinline bool isFaceRegular() const 
      {
	HalfEdge *p = (HalfEdge*)this;
	if (p->getEdgeValence() != 4) return false;
        if (p->hasIrregularEdge()) return false;
	p = p->next();
	if (p->getEdgeValence() != 4) return false;
        if (p->hasIrregularEdge()) return false;
	p = p->next();
	if (p->getEdgeValence() != 4) return false;
        if (p->hasIrregularEdge()) return false;
	p = p->next();
	if (p->getEdgeValence() != 4) return false;
        if (p->hasIrregularEdge()) return false;
	return true;
      }

      __forceinline bool faceHasEdges() const 
      {
	HalfEdge *p = (HalfEdge*)this;
	if (p->hasOpposite() == false) return true;
	p = p->next();
	if (p->hasOpposite() == false) return true;
	p = p->next();
	if (p->hasOpposite() == false) return true;
	p = p->next();
	if (p->hasOpposite() == false) return true;
	return false;
      }

      friend __forceinline std::ostream &operator<<(std::ostream &o, const SubdivMesh::HalfEdge &h)
      {
        return o << "{ " << 
          "edge = " << h.vtx_index << " -> " << h.next()->vtx_index << ", " << 
          "edge_crease = " << h.edge_crease_weight << ", " << 
          "vertex_crease = " << h.vertex_crease_weight << ", " << 
          "edge_level = " << h.edge_level << 
          "}";
      } 

    private:
      unsigned int vtx_index;         //!< index of edge start vertex
      int next_half_edge_ofs;         //!< relative offset to next half edge of face
      int prev_half_edge_ofs;         //!< relative offset to previous half edge of face
      int opposite_half_edge_ofs;     //!< relative offset to opposite half edge

    public:
      float edge_crease_weight;       //!< crease weight attached to edge
      float vertex_crease_weight;     //!< crease weight attached to start vertex
      float edge_level;               //!< subdivision factor for edge
      float align;                    //!< aligns the structure to 32 bytes
    };

  public: // FIXME: make private

    BufferT<int> faceVertices;
    BufferT<int> holes;
    std::vector<bool> full_holes;
    std::vector<HalfEdge*> faceStartEdge;

    /*! Indices of the vertices composing each face, provided by the application */
    BufferT<unsigned int> vertexIndices;

    /*! Vertex buffer, provided by the application */
    BufferT<Vec3fa> vertices[2];      //!< vertex array

    /*! Crease buffer, provided by the application */
    BufferT<Vec2i> edge_creases;
    BufferT<float> edge_crease_weights;

    /*! Vertex_Crease buffer, provided by the application */
    BufferT<int> vertex_creases;
    BufferT<float> vertex_crease_weights;

    /*! Subdivision level per edge, provided by the application */
    BufferT<float> levels;

    /*! Half edge structure. */
    HalfEdge *halfEdges;

  public:

    /*! Coordinates of the vertex at the given index in the mesh. */
    __forceinline const Vec3fa &getVertexPosition(unsigned int index, const unsigned int t = 0) const { 
      return vertices[t][index]; 
    }

    __forceinline const Vec3fa *getVertexPositionPtr(const unsigned int t = 0) const { return &vertices[t][0]; }

    __forceinline const HalfEdge &getHalfEdgeForQuad(unsigned int q, const unsigned int i=0) const { 
      return *(faceStartEdge[q]+i);
    }

    __forceinline const Vec3fa &getVertexPositionForHalfEdge(const HalfEdge &e) const { 
      return getVertexPosition( e.vtx_index );
    }

    __forceinline const Vec3fa &getVertexPositionForQuad(unsigned int q, const unsigned int i=0) const { 
      return getVertexPositionForHalfEdge( getHalfEdgeForQuad(q,i) );
    }


    void initializeHalfEdgeStructures ();

    /*! calculates the bounds of the face associated with the half-edge */
    __forceinline BBox3fa getFaceBounds(const HalfEdge* e) const 
    {
      BBox3fa b = getVertexPosition(e->getStartVertexIndex());
      for (const HalfEdge* p = e->next(); p!=e; p=p->next()) 
        b.extend(getVertexPosition(p->getStartVertexIndex()));
      return b;
    }

    /*! calculates the bounds of the 1-ring associated with the vertex of the half-edge */
    __forceinline BBox3fa get1RingBounds(const HalfEdge* h) const 
    {
      BBox3fa bounds = empty;
      const HalfEdge* p = h;
      do 
      {
        /* calculate bounds of current face */
        bounds.extend(getFaceBounds(p));
        p = p->prev();
                      
        /* continue with next face */
        if (likely(p->hasOpposite())) 
          p = p->opposite();
        
        /* if there is no opposite go the long way to the other side of the border */
        else {
          p = h;
          while (p->hasOpposite()) 
            p = p->opposite()->next();
        }

      } while (p != h); 
      
      return bounds;
    }

    /*! calculates the bounds of the i'th subdivision patch */
    __forceinline BBox3fa bounds(size_t i) const 
    {
      const HalfEdge* h = faceStartEdge[i];
      BBox3fa bounds = get1RingBounds(h);
      for (const HalfEdge* p=h->next(); p!=h; p=p->next())
        bounds.extend(get1RingBounds(p));
      return bounds;
    }

    /*! check if the i'th primitive is valid */
    __forceinline bool valid(size_t i, BBox3fa* bbox = NULL) const {
      if (bbox) *bbox = bounds(i);
      return !full_holes[i];
    }

  };
};
