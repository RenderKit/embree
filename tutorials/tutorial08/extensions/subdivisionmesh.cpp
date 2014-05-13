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

#include <algorithm>
#include "subdivisionmesh.h"

class EdgeList {
public:

    class Edge {
    public:

        /*! Constructor. */
        Edge(const SubdivisionMesh::HalfEdge &edge, const SubdivisionMesh::Vertex &vertex) : v0(edge.getVertex().getIndex()), v1(vertex.getIndex()), edge(edge) { if (v1 < v0) std::swap(v0, v1); }

        /*! Indices of vertices defining the edge. */
        int32_t v0, v1;

        /*! One half of the edge. */
        int64_t edge;

    };

    /*! Constructor. */
    inline EdgeList(SubdivisionMesh *mesh) : mesh(mesh) {}

    /*! Append the edges of a face to the list. */
    inline void appendEdges(const SubdivisionMesh::Face &face) { for (size_t i=0 ; i < face.edgeCount() ; i++) data.push_back(Edge(face.getEdge(i), face.getVertex(i + 1))); }

    /*! Compare two edges. */
    static bool compareEdges(const Edge &first, const Edge &second) { return((first.v0 == second.v0) ? (first.v1 < second.v1) : (first.v0 < second.v0)); }

    /*! Half edge at the given index in the list. */
    inline SubdivisionMesh::HalfEdge getEdge(size_t index) { return(SubdivisionMesh::HalfEdge(data.at(index).edge, mesh)); }

    /*! Change the list capacity. */
    inline void reserve(size_t count) { data.reserve(count); }

    /*! Sort the edge list. */
    inline void sort() { std::sort(data.begin(), data.end(), compareEdges); }

private:

    /*! Edge data. */
    std::vector<Edge> data;

    /*! Corresponding mesh. */
    SubdivisionMesh *mesh;

};

SubdivisionMesh::SubdivisionMesh(const size_t coarseFaceCount, const size_t coarseEdgeCount, const size_t coarseVertexCount) {

    vertexIndices.resize(coarseEdgeCount * 2, -1);  vertexCoordinates.resize(coarseVertexCount, Vec3f(0.0f, 0.0f, 0.0f));
    vertexOffsets.resize(coarseFaceCount + 1,  0);  holeMarkers.resize(coarseFaceCount, 0);
    creaseWeights.resize(coarseEdgeCount * 2,  0);

}

void SubdivisionMesh::commit() {

    EdgeList list(this);  list.reserve(edgeCount() * 2);
    for (size_t i=0 ; i < faceCount() ; i++) list.appendEdges(getFace(i));  list.sort();

    oppositeEdges = std::vector<int64_t>(edgeCount() * 2, -1);
    for (size_t i=0 ; i < edgeCount() * 2 ; i += 2) setOppositeEdges(list.getEdge(i), list.getEdge(i + 1));

    vertexEdges = std::vector<int64_t>(vertexCount(), -1);
    for (size_t i=0 ; i < edgeCount() * 2 ; i++) { HalfEdge edge = getOppositeEdge(i);  setVertexEdge(edge.getVertex().getIndex(), edge); }

}

