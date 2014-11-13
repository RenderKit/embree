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

      /*! tests if the start vertex of the edge is regular */
      __forceinline bool isRegularVertex() const 
      {
	const HalfEdge* p = this;

        if (!p->hasOpposite()) return false;
        if ((p = p->rotate()) == this) return false;

        if (!p->hasOpposite()) return false;
        if ((p = p->rotate()) == this) return false;

        if (!p->hasOpposite()) return false;
        if ((p = p->rotate()) == this) return false;

        if (!p->hasOpposite()) return false;
        if ((p = p->rotate()) != this) return false;

        return true;
      }

      /*! tests if the face is a regular face */
      __forceinline bool isRegularFace() const 
      {
	const HalfEdge* p = this;

        if (!p->isRegularVertex()) return false;
        if ((p = p->next()) == this) return false;

        if (!p->isRegularVertex()) return false;
        if ((p = p->next()) == this) return false;

        if (!p->isRegularVertex()) return false;
        if ((p = p->next()) == this) return false;
        
        if (!p->isRegularVertex()) return false;
        if ((p = p->next()) != this) return false;

	return true;
      }

      /*! calculates conservative bounds of a catmull clark subdivision face */
      __forceinline BBox3fa bounds(const BufferT<Vec3fa>& vertices) const
      {
        BBox3fa bounds = this->get1RingBounds(vertices);
        for (const HalfEdge* p=this->next(); p!=this; p=p->next())
          bounds.extend(p->get1RingBounds(vertices));
        return bounds;
      }

      /*! stream output */
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

      /*! calculates the bounds of the face associated with the half-edge */
      __forceinline BBox3fa getFaceBounds(const BufferT<Vec3fa>& vertices) const 
      {
        BBox3fa b = vertices[getStartVertexIndex()];
        for (const HalfEdge* p = next(); p!=this; p=p->next()) 
          b.extend(vertices[p->getStartVertexIndex()]);
        return b;
      }

      /*! calculates the bounds of the 1-ring associated with the vertex of the half-edge */
      __forceinline BBox3fa get1RingBounds(const BufferT<Vec3fa>& vertices) const 
      {
        BBox3fa bounds = empty;
        const HalfEdge* p = this;
        do 
        {
          /* calculate bounds of current face */
          bounds.extend(p->getFaceBounds(vertices));
          p = p->prev();
          
          /* continue with next face */
          if (likely(p->hasOpposite())) 
            p = p->opposite();
          
          /* if there is no opposite go the long way to the other side of the border */
          else {
            p = this;
            while (p->hasOpposite()) 
              p = p->opposite()->next();
          }
          
        } while (p != this); 
        
        return bounds;
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

    size_t size() const { return numFaces; };

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

    /*! calculates the bounds of the i'th subdivision patch */
    __forceinline BBox3fa bounds(size_t i) const {
      return faceStartEdge[i]->bounds(vertices[0]);
    }

    /*! check if the i'th primitive is valid */
    __forceinline bool valid(size_t i, BBox3fa* bbox = NULL) const {
      if (bbox) *bbox = bounds(i);
      return !full_holes[i];
    }

  public: // FIXME: make private

    unsigned int mask;                //!< for masking out geometry
    unsigned int numTimeSteps;        //!< number of time steps (1 or 2)  

    size_t numFaces;                  //!< number of faces
    size_t numEdges;                  //!< number of edges
    size_t numVertices;               //!< number of vertices

    RTCDisplacementFunc displFunc;    //!< displacement function
    BBox3fa             displBounds;  //!< bounds for displacement

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
    HalfEdge* halfEdges;
  };
};
