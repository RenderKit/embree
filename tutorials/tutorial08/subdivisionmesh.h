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

#include <vector>
#include "math/vec3.h"

using namespace embree;

#define HALF_EDGE_VERTEX_BITS       4
#define HALF_EDGE_VERTEX_MASK       0xF
#define HALF_EDGE_VERTEX_SLOT(a)  ((int32_t)(a &  HALF_EDGE_VERTEX_MASK))
#define HALF_EDGE_FACE_INDEX(a)   ((int32_t)(a >> HALF_EDGE_VERTEX_BITS))
#define HALF_EDGE_INDEX(a, b)     ((int64_t) a << HALF_EDGE_VERTEX_BITS | b)

// namespace embree {

    class SubdivisionMesh {
    public:

        class HalfEdge;
        class Vertex;

        class Face {
        public:

            /*! Constructor. */
            inline Face(int32_t id, SubdivisionMesh *mesh) : id(id), mesh(mesh) {}

            /*! Number of edges adjacent to the face. */
            inline int32_t edgeCount() const { return(vertexCount()); }

            /*! Half edge at the given index in the face. */
            inline HalfEdge getEdge(int32_t index) const { return(mesh->getEdge(id, index % edgeCount())); }

            /*! Opposing half edge at the given index in the face. */
            inline HalfEdge getOppositeEdge(int32_t index) const { return(mesh->getOppositeEdge(mesh->getIndex(id) + index % edgeCount())); }

            /*! Index of the face in the mesh. */
            inline int32_t getIndex() const { return(id); }

            /*! Vertex at the given index in the face. */
            inline Vertex getVertex(int32_t index) const { return(mesh->getVertex(mesh->getVertexIndex(mesh->getIndex(id) + index % vertexCount()))); }

            /*! Associate the given vertex with the face. */
            inline void setVertex(int32_t index, Vertex &vertex) { mesh->setVertex(mesh->getIndex(id) + index, vertex); }

            /*! Number of vertices adjacent to the face. */
            inline int32_t vertexCount() const { return(mesh->getIndex(id + 1) - mesh->getIndex(id)); }

        private:

            /*! Offset into the vertex offset array for this face. */
            int32_t id;

            /*! Parent mesh. */
            SubdivisionMesh *mesh;

        };

        class HalfEdge {
        public:

            /*! Constructor. */
            inline HalfEdge(int64_t id, SubdivisionMesh *mesh) : id(id), mesh(mesh) {}

            /*! Double word representation of the half edge. */
            inline operator int64_t() const { return(id); }

            /*! Face adjacent to the half edge. */
            inline Face getFace() const { return(mesh->getFace(HALF_EDGE_FACE_INDEX(id))); }

            /*! Index of the edge in the mesh. */
            inline int64_t getIndex() const { return(mesh->getIndex(HALF_EDGE_FACE_INDEX(id)) + HALF_EDGE_VERTEX_SLOT(id)); }

            /*! Next edge in the adjacent face. */
            inline HalfEdge getNextEdge() const { return(getFace().getEdge(HALF_EDGE_VERTEX_SLOT(id) + 1)); }

            /*! Next edge around the adjacent vertex. */
            inline HalfEdge getNextVertexEdge() const { return(getOppositeEdge().getNextEdge()); }

            /*! Half edge opposite this half edge. */
            inline HalfEdge getOppositeEdge() const { return(mesh->getOppositeEdge(mesh->getIndex(HALF_EDGE_FACE_INDEX(id)) + HALF_EDGE_VERTEX_SLOT(id))); }

            /*! Vertex adjacent to the half edge. */
            inline Vertex getVertex() const { return(mesh->getVertex(mesh->getVertexIndex(mesh->getIndex(HALF_EDGE_FACE_INDEX(id)) + HALF_EDGE_VERTEX_SLOT(id)))); }

        private:

            /*! Offset into the vertex offset array for the adjacent face, and index of the adjacent vertex in that face. */
            int64_t id;

            /*! Parent mesh. */
            SubdivisionMesh *mesh;

        };

        class Vertex {
        public:

            /*! Constructor. */
            inline Vertex(int32_t id, SubdivisionMesh *mesh) : id(id), mesh(mesh) {}

            /*! Number of edges adjacent to the vertex. */
            inline int32_t edgeCount() { int32_t i = 1;  HalfEdge origin = getEdge(), edge = origin.getNextVertexEdge();  while (edge != origin) edge = edge.getNextVertexEdge(), i++;  return(i); }

            /*! Number of faces adjacent to the vertex. */
            inline int32_t faceCount() { return(edgeCount()); }

            /*! Coordinates of the vertex. */
            inline Vec3f getCoordinates() const { return(mesh->getCoordinates(id)); }

            /*! An edge adjacent to the vertex. */
            inline HalfEdge getEdge() const { return(mesh->getVertexEdge(id)); }

            /*! Index of the vertex in the mesh. */
            inline int32_t getIndex() const { return(id); }

            /*! Set the coordinates of the vertex. */
            inline void setCoordinates(const Vec3f &coordinates) { mesh->setCoordinates(id, coordinates); }

            /*! Associate an edge with the vertex. */
            inline void setEdge(HalfEdge &edge) { mesh->setVertexEdge(id, edge); }

        private:

            /*! Index of the vertex in the mesh. */
            int32_t id;

            /*! Parent mesh. */
            SubdivisionMesh *mesh;

        };

    public:

        /*! Constructor. */
        SubdivisionMesh(const float *vertices, const int32_t *indices, const int32_t *offsets, size_t vertexcount, size_t edgecount, size_t facecount);

        /*! Constructor. */
        SubdivisionMesh(const std::vector<Vec3f> &vertices, const std::vector<int32_t> &indices, const std::vector<int32_t> &offsets);

        /*! Constructor. */
        SubdivisionMesh() : vertexOffsets(std::vector<int32_t>(1, 0)) {}

        /*! Destroy the current mesh data. */
        inline void clear() { vertexCoordinates.clear();  vertexIndices.clear();  vertexOffsets.resize(1, 0); }

        /*! Compute edge adjacency information over the mesh. */
        void commit();

        /*! Number of half edges currently contained in the mesh. */
        inline size_t edgeCount() { return(vertexIndices.size()); }

        /*! Number of faces currently contained in the mesh. */
        inline size_t faceCount() { return(vertexOffsets.size() - 1); }

        /*! Coordinates of the vertex at the given index in the mesh. */
        inline Vec3f getCoordinates(int32_t index) { return(vertexCoordinates.at(index)); }

        /*! Half edge at the given index in the mesh. */
        inline HalfEdge getEdge(int32_t offset, int32_t index) { return(HalfEdge(HALF_EDGE_INDEX(offset, index), this)); }

        /*! Face at the given index in the mesh. */
        inline Face getFace(int32_t index) { return(Face(index, this)); }

        /*! An offset into the vertex index array. */
        inline int32_t getIndex(int32_t index) { return(vertexOffsets.at(index)); }

        /*! Opposing half edge at the given index in the mesh. */
        inline HalfEdge getOppositeEdge(int32_t index) { return(HalfEdge(oppositeEdges.at(index), this)); }

        /*! Vertex at the given index in the mesh. */
        inline Vertex getVertex(int32_t index) { return((index < 0) ? newVertex() : Vertex(index, this)); }

        /*! Index of a vertex in the mesh corresponding to an instance in a face. */
        inline int32_t getVertexIndex(int32_t index) { return(vertexIndices.at(index)); }

        /*! Edge associated with the vertex at the given index in the mesh. */
        inline HalfEdge getVertexEdge(int32_t index) { return(HalfEdge(vertexEdges.at(index), this)); }

        /*! Add an empty face with the given vertex count to the mesh. */
        inline Face newFace(int32_t valence) { vertexIndices.insert(vertexIndices.end(), valence, -1);  vertexOffsets.push_back(getIndex(faceCount()) + valence);  return(Face(faceCount() - 1, this)); }

        /*! Add an empty vertex to the mesh. */
        inline Vertex newVertex() { vertexCoordinates.push_back(Vec3f(0.0f, 0.0f, 0.0f));  return(Vertex(vertexCount() - 1, this)); }

        /*! Update the coordinates of the vertex at the given index in the mesh. */
        inline void setCoordinates(int32_t index, const Vec3f &coordinates) { vertexCoordinates.at(index) = coordinates; }

        /*! Store the two halfs of an edge in the mesh. */
        inline void setOppositeEdges(const HalfEdge &first, const HalfEdge &second) { oppositeEdges.at(first.getIndex()) = second;  oppositeEdges.at(second.getIndex()) = first; }

        /*! Store an index to a vertex in the mesh. */
        inline void setVertex(int32_t index, Vertex &vertex) { vertexIndices.at(index) = vertex.getIndex(); }

        /*! Associate an edge with a vertex in the mesh. */
        inline void setVertexEdge(int32_t index, HalfEdge &edge) { vertexEdges.at(index) = edge; }

        /*! Number of vertices currently contained in the mesh. */
        inline size_t vertexCount() { return(vertexCoordinates.size()); }

    private:

        /*! Opposing half edges adjacent to each face. */
        std::vector<int64_t> oppositeEdges;

        /*! Vertex coordinates. */
        std::vector<Vec3f> vertexCoordinates;

        /*! A single half edge adjacent to each vertex. */
        std::vector<int64_t> vertexEdges;

        /*! Indices of the vertices composing each face. */
        std::vector<int32_t> vertexIndices;

        /*! Offsets into the vertexIndices array indexed by face. */
        std::vector<int32_t> vertexOffsets;

        /*! Pair up opposing half edges in the mesh. */
        void matchOpposingEdges();

    };

// }

