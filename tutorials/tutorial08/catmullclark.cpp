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

#include "catmullclark.h"

void catmullClarkEdgeVertex(SubdivisionMesh &mesh, const SubdivisionMesh::HalfEdge &edge) {

    SubdivisionMesh::Vertex vertex = mesh.getFace(edge.getOppositeEdge().getIndex()).getVertex(1);
    vertex.setCoordinates(vertex.getCoordinates() + (edge.getVertex().getCoordinates() + mesh.getFace(edge.getIndex()).getVertex(2).getCoordinates()) * 0.25f);
    mesh.getFace(edge.getIndex()).setVertex(1, vertex);
    mesh.getFace(edge.getNextEdge().getIndex()).setVertex(3, vertex);

}

void catmullClarkFaceVertex(SubdivisionMesh &mesh, const SubdivisionMesh::Face &face) {

    SubdivisionMesh::Vertex vertex = mesh.newVertex();
    for (size_t i=0 ; i < face.vertexCount() ; i++) vertex.setCoordinates(vertex.getCoordinates() + face.getVertex(i).getCoordinates() / face.vertexCount());
    for (size_t i=0 ; i < face.vertexCount() ; i++) mesh.getFace(face.getEdge(i).getIndex()).setVertex(2, vertex);

}

void catmullClarkMoveVertex(SubdivisionMesh &mesh, SubdivisionMesh::HalfEdge edge, int32_t valence) {

    SubdivisionMesh::Vertex vertex = mesh.newVertex();

    for (size_t i=0 ; i < valence ; i++, edge = edge.getNextVertexEdge())
        vertex.setCoordinates(vertex.getCoordinates() + mesh.getFace(edge.getIndex()).getVertex(2).getCoordinates() / valence);

    for (size_t i=0 ; i < valence ; i++, edge = edge.getNextVertexEdge())
        vertex.setCoordinates(vertex.getCoordinates() + mesh.getFace(edge.getIndex()).getVertex(1).getCoordinates() * 2.0f / valence);

    vertex.setCoordinates((vertex.getCoordinates() + edge.getVertex().getCoordinates() * (valence - 3.0f)) / valence);
    for (size_t i=0 ; i < valence ; i++, edge = edge.getNextVertexEdge()) mesh.getFace(edge.getIndex()).setVertex(0, vertex);

}

void catmullClarkSubdivideMesh(SubdivisionMesh &coarseMesh, SubdivisionMesh &fineMesh) {

    fineMesh.clear();
    for (size_t i=0 ; i < coarseMesh.edgeCount()   ; i++) fineMesh.newFace(4);
    for (size_t i=0 ; i < coarseMesh.faceCount()   ; i++) catmullClarkFaceVertex(fineMesh, coarseMesh.getFace(i));
    for (size_t i=0 ; i < coarseMesh.edgeCount()   ; i++) catmullClarkEdgeVertex(fineMesh, coarseMesh.getOppositeEdge(i));
    for (size_t i=0 ; i < coarseMesh.vertexCount() ; i++) catmullClarkMoveVertex(fineMesh, coarseMesh.getVertex(i).getEdge(), coarseMesh.getVertex(i).edgeCount());

}

