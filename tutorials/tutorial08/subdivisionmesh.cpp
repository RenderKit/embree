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

// namespace embree {

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

    SubdivisionMesh::SubdivisionMesh(const float *vertices, const int32_t *indices, const int32_t *offsets, size_t vertexcount, size_t edgecount, size_t facecount) {

        vertexCoordinates.reserve(vertexcount);  for (size_t i=0, j=0 ; i < vertexcount ; i++) vertexCoordinates.push_back(Vec3f(vertices[j++], vertices[j++], vertices[j++]));
        vertexIndices.reserve(edgecount);        for (size_t i=0 ; i < edgecount ; i++) vertexIndices.push_back(indices[i]);
        vertexOffsets.reserve(facecount + 1);    for (size_t i=0 ; i < facecount + 1 ; i++) vertexOffsets.push_back(offsets[i]);

    }

    SubdivisionMesh::SubdivisionMesh(const std::vector<Vec3f> &vertices, const std::vector<int32_t> &indices, const std::vector<int32_t> &offsets) {

        vertexCoordinates = vertices;
        vertexIndices     = indices;
        vertexOffsets     = offsets;

    }

    void SubdivisionMesh::commit() {

        EdgeList list(this);  list.reserve(edgeCount());
        for (size_t i=0 ; i < faceCount() ; i++) list.appendEdges(getFace(i));  list.sort();

        oppositeEdges = std::vector<int64_t>(edgeCount(), -1);
        for (size_t i=0 ; i < edgeCount() ; i += 2) setOppositeEdges(list.getEdge(i), list.getEdge(i + 1));

/*
printf("Vertex Coordinates\n");
for (size_t i=0 ; i < vertexCoordinates.size() ; i++) printf("%f %f %f\n", vertexCoordinates[i].x, vertexCoordinates[i].y, vertexCoordinates[i].z);
printf("Vertex Offsets\n");
for (size_t i=0 ; i < vertexOffsets.size() ; i++) printf("%d\n", vertexOffsets[i]);
printf("Vertex Indices\n");
for (size_t i=0 ; i < vertexIndices.size() ; i++) printf("%d\n", vertexIndices[i]);
printf("Opposite Edges\n");
for (size_t i=0 ; i < oppositeEdges.size() ; i++) (oppositeEdges.at(i) < 0) ? printf("-1\n") : printf("%d %d\n", HALF_EDGE_FACE_INDEX(oppositeEdges.at(i)), HALF_EDGE_VERTEX_SLOT(oppositeEdges.at(i)));
printf("Edge List\n");
for (size_t i=0 ; i < list.data.size() ; i++) printf("%d %d %d %d\n", list.data[i].v0, list.data[i].v1, HALF_EDGE_FACE_INDEX(list.data[i].edge), HALF_EDGE_VERTEX_SLOT(list.data[i].edge));
printf("DONE\n");
*/

        vertexEdges = std::vector<int64_t>(vertexCount(), -1);
        for (size_t i=0 ; i < edgeCount() ; i++) { HalfEdge edge = getOppositeEdge(i);  setVertexEdge(edge.getVertex().getIndex(), edge); }

    }

// }

