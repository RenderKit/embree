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
  class SubdivFace;
  class SubdivHalfEdge;
  class SubdivVertex;

  class SubdivMesh : public Geometry
  {
  public:
    SubdivMesh(Scene* parent, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, size_t numTimeSteps);

    void enabling();
    void disabling();
    void setMask (unsigned mask);
    void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride);
    void* map(RTCBufferType type);
    void unmap(RTCBufferType type);
    void setUserData (void* ptr, bool ispc);
    void immutable ();
    bool verify ();

    unsigned int mask;                //!< for masking out geometry
    unsigned int numTimeSteps;        //!< number of time steps (1 or 2)  

    size_t numFaces;                  //!< number of faces
    size_t numEdges;                  //!< number of edges
    size_t numVertices;               //!< number of vertices

    size_t size() const { return numFaces; };

    /*! calculates the bounds of the i'th subdivision patch */
    __forceinline BBox3fa bounds(size_t i) const 
    {
      PRINT("SubdivMesh::bounds not implemented");
      return empty;
    }
    
    class HalfEdge
    {
    public:
      unsigned int vtx_index;
      unsigned int start_halfedge_id : 30;
      unsigned int local_halfedge_id :  2;
      unsigned int opposite;

      bool hasOpposite() const {
        return opposite != (unsigned int)-1;
      };

      const HalfEdge &getOppositeHalfEdge(const HalfEdge *const halfEdges) const { 
        assert( opposite != (unsigned int)-1 );
        return halfEdges[opposite]; 
      };

      unsigned int getStartVertexIndex(const HalfEdge *const halfEdges) const { 
        return vtx_index; 
      };
      
      const HalfEdge &next(const HalfEdge *const halfEdges) const {
        return halfEdges[ start_halfedge_id + ((local_halfedge_id+1)%4) ];
      };

      const HalfEdge &prev(const HalfEdge *const halfEdges) const {
        return halfEdges[ start_halfedge_id + ((local_halfedge_id+3)%4) ];
      };

      unsigned int getEndVertexIndex(const HalfEdge *const halfEdges) const {
        return next(halfEdges).vtx_index;
      };
      
    };



  private:
    BufferT<Vec3fa> vertices[2];      //!< vertex array

    
    /*! Indices of the vertices composing each face, provided by the application */
    BufferT<unsigned int> vertexIndices;

    /*! Offsets into the vertexIndices array indexed by face, provided by the application */
    BufferT<unsigned int> vertexOffsets;


    HalfEdge *halfEdges;

  public:

    /*! Coordinates of the vertex at the given index in the mesh. */
    __forceinline const Vec3fa &getPosition(unsigned int index, const unsigned int t = 0) const { return vertices[t][index]; }

    HalfEdge *initializeHalfEdgeStructures (unsigned int &numHalfEdges);


  };

#if 0

#define HALF_EDGE_VERTEX_BITS       4
#define HALF_EDGE_VERTEX_MASK       0xF
#define HALF_EDGE_VERTEX_SLOT(a)  ((int32_t)(a &  HALF_EDGE_VERTEX_MASK))
#define HALF_EDGE_FACE_INDEX(a)   ((int32_t)(a >> HALF_EDGE_VERTEX_BITS))
#define HALF_EDGE_INDEX(a, b)     ((int64_t) a << HALF_EDGE_VERTEX_BITS | b)

    /*! Number of edges currently contained in the mesh. */
    __forceinline size_t edgeCount() { return(vertexIndices.size() / 2); }

    /*! Number of faces currently contained in the mesh. */
    __forceinline size_t faceCount() { return(vertexOffsets.size() - 1); }

    /*! Coordinates of the vertex at the given index in the mesh. */
    __forceinline const Vec3fa &getCoordinates(unsigned int index) const { return vertices[0][index]; }

    /*! Half edge at the given index in the mesh. */
    __forceinline SubdivHalfEdge getEdge(unsigned int offset, unsigned int index);

    /*! Face at the given index in the mesh. */
    __forceinline SubdivFace getFace(unsigned int index);

    /*! An offset into the vertex index array. */
    __forceinline unsigned int getFaceVertexOffset(unsigned int index) { return vertexOffsets[index]; }

    /*! Pointer to the vertex index array. */
    __forceinline unsigned int *getIndexBuffer() { return &vertexIndices[0]; }

    /*! Pointer to the offset index array. */
    __forceinline unsigned int *getOffsetBuffer() { return &vertexOffsets[0]; }

    /*! Opposing half edge at the given index in the mesh. */
    __forceinline SubdivHalfEdge getOppositeEdge(unsigned int index);

    /*! Vertex at the given index in the mesh. */
    __forceinline SubdivVertex getVertex(unsigned int index);

    /*! Pointer to the vertex coordinates array. */
    __forceinline Vec3fa *getVertexBuffer() { return &vertices[0][0]; }

    /*! Index of a vertex in the mesh corresponding to an instance in a face. */
    __forceinline unsigned int getVertexIndex(unsigned int index) { return vertexIndices[index]; }

    /*! Edge associated with the vertex at the given index in the mesh. */
    __forceinline SubdivHalfEdge getVertexEdge(unsigned int index);

    /*! Number of vertices currently contained in the mesh. */
    __forceinline size_t vertexCount() { return vertices[0].size(); }

  class SubdivVertex {
  public:

    /*! Constructor. */
    __forceinline SubdivVertex(unsigned int id) : id(id) {}

    /*! Number of edges adjacent to the vertex. */
    __forceinline unsigned int edgeCount(SubdivMesh *mesh);
    /*! Number of faces adjacent to the vertex. */
    __forceinline unsigned int faceCount(SubdivMesh *mesh) { return edgeCount(mesh); }

    /*! Coordinates of the vertex. */
    __forceinline const Vec3fa &getCoordinates(SubdivMesh *mesh) const { return mesh->getCoordinates(id); }

    /*! An edge adjacent to the vertex. */
    __forceinline SubdivHalfEdge getEdge(SubdivMesh *mesh) const;

    /*! Index of the vertex in the mesh. */
    __forceinline unsigned int getIndex() const { return id; }

  private:

    /*! Index of the vertex in the mesh. */
    unsigned int id;

  };


  class SubdivFace
  {
  public:

    /*! Constructor. */
    __forceinline SubdivFace(unsigned int id) : id(id) {}

    /*! Number of edges adjacent to the face. */
    __forceinline unsigned int edgeCount(SubdivMesh *mesh) const { return vertexCount(mesh); }

    /*! Half edge at the given index in the face. */
    __forceinline SubdivHalfEdge getEdge(SubdivMesh *mesh, unsigned int index) const;

    /*! Index of the face in the mesh. */
    __forceinline unsigned int getIndex() const { return(id); }

    /*! Opposing half edge at the given index in the face. */
    __forceinline SubdivHalfEdge getOppositeEdge(SubdivMesh *mesh, unsigned int index) const;

    /*! Vertex at the given index in the face. */
    __forceinline SubdivVertex getVertex(SubdivMesh *mesh, unsigned int index) const;

    /*! Number of vertices adjacent to the face. */
    __forceinline unsigned int vertexCount(SubdivMesh *mesh) const { return mesh->getFaceVertexOffset(id + 1) - mesh->getFaceVertexOffset(id); }

  private:

    /*! Offset into the vertex offset array for this face. */
    unsigned int id;

  };

  class SubdivHalfEdge {
  public:

    /*! Constructor. */
    __forceinline SubdivHalfEdge(size_t id) : id(id) {}

    /*! Double word representation of the half edge. */
    __forceinline operator size_t() const { return id; }

    /*! Face adjacent to the half edge. */
    __forceinline SubdivFace getFace(SubdivMesh *mesh) const { return mesh->getFace(HALF_EDGE_FACE_INDEX(id)); }

    /*! Index of the edge in the mesh. */
    __forceinline size_t getIndex(SubdivMesh *mesh) const { return mesh->getFaceVertexOffset(HALF_EDGE_FACE_INDEX(id)) + HALF_EDGE_VERTEX_SLOT(id); }

    /*! Next edge in the adjacent face. */
    __forceinline SubdivHalfEdge getNextEdge(SubdivMesh *mesh) const { return getFace(mesh).getEdge(mesh,HALF_EDGE_VERTEX_SLOT(id) + 1); }

    /*! Next edge around the adjacent vertex. */
    __forceinline SubdivHalfEdge getNextVertexEdge(SubdivMesh *mesh) const { return getOppositeEdge(mesh).getNextEdge(mesh); }

    /*! Half edge opposite this half edge. */
    __forceinline SubdivHalfEdge getOppositeEdge(SubdivMesh *mesh) const { return mesh->getOppositeEdge(getIndex(mesh)); }

    /*! Vertex adjacent to the half edge. */
    __forceinline SubdivVertex getVertex(SubdivMesh *mesh) const { return mesh->getVertex(mesh->getVertexIndex(getIndex(mesh))); }

  private:

    /*! Offset into the vertex offset array for the adjacent face, and index of the adjacent vertex in that face. */
    size_t id;
  };


  __forceinline SubdivHalfEdge SubdivMesh::getEdge(unsigned int offset, unsigned int index)
    { 
      return SubdivHalfEdge(HALF_EDGE_INDEX(offset, index)); 
    }

  __forceinline SubdivFace SubdivMesh::getFace(unsigned int index) 
    { 
      return SubdivFace(index); 
    }

  __forceinline SubdivHalfEdge SubdivMesh::getOppositeEdge(unsigned int index) 
    { 
      return SubdivHalfEdge(oppositeEdges[index]); 
    }
  
  __forceinline SubdivVertex SubdivMesh::getVertex(unsigned int index) 
    { 
      return SubdivVertex(index); 
    }

  __forceinline SubdivHalfEdge SubdivMesh::getVertexEdge(unsigned int index) 
    { 
      return SubdivHalfEdge(vertexEdges[index]); 
    }


  __forceinline unsigned int SubdivVertex::edgeCount(SubdivMesh *mesh) 
  { 
    unsigned int i = 1;  
    SubdivHalfEdge origin = getEdge(mesh), edge = origin.getNextVertexEdge(mesh);  
    while (edge != origin) edge = edge.getNextVertexEdge(mesh), i++;  return i; 
  }

  __forceinline SubdivHalfEdge SubdivVertex::getEdge(SubdivMesh *mesh) const 
    { 
      return mesh->getVertexEdge(id); 
    }

  __forceinline SubdivHalfEdge SubdivFace::getEdge(SubdivMesh *mesh, unsigned int index) const 
    { 
      return mesh->getEdge(id, index % edgeCount(mesh)); 
    }

  __forceinline SubdivHalfEdge SubdivFace::getOppositeEdge(SubdivMesh *mesh, unsigned int index) const 
    { 
      return mesh->getOppositeEdge(mesh->getFaceVertexOffset(id) + index % edgeCount(mesh)); 
    }
  
  __forceinline SubdivVertex SubdivFace::getVertex(SubdivMesh *mesh, unsigned int index) const 
    { 
      return mesh->getVertex(mesh->getVertexIndex(mesh->getFaceVertexOffset(id) + index % vertexCount(mesh))); 
    }
#endif

};
