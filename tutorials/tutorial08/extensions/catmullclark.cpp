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

/*! Global vertex counter. */
int32_t vertexCounter = 0;

/*! Assigned sum of two float vectors. */
Vec3f &operator+=(Vec3f &lhs, const Vec3f &rhs) { lhs.x += rhs.x;  lhs.y += rhs.y;  lhs.z += rhs.z;  return(lhs); }

void catmullClarkEdgeVertex(SubdivisionMesh &mesh, const SubdivisionMesh::HalfEdge &coarseEdge) {

    /*! Index of the new edge vertex in the subdivided mesh. */
    int32_t vertexIndex = mesh.getFace(coarseEdge.getOppositeEdge().getIndex()).getVertex(1).getIndex();  if (vertexIndex < 0) vertexIndex = vertexCounter++;

    /*! Coordinates of the new edge vertex. */
    Vec3f &vertexCoordinates = mesh.getVertexBuffer()[vertexIndex];

    /*! The new vertex is the average of the old edge vertices and the new face vertices adjacent to the old edge. */
    vertexCoordinates += (coarseEdge.getVertex().getCoordinates() + mesh.getFace(coarseEdge.getIndex()).getVertex(2).getCoordinates()) * 0.25f;

    /*! Associate the new vertex with 1 of the 4 adjacent faces in the subdivided mesh. */
    mesh.getIndexBuffer()[mesh.getIndex(coarseEdge.getIndex()) + 1] = vertexIndex;

    /*! Associate the new vertex with 1 of the 4 adjacent faces in the subdivided mesh. */
    mesh.getIndexBuffer()[mesh.getIndex(coarseEdge.getNextEdge().getIndex()) + 3] = vertexIndex;

}

void catmullClarkFaceVertex(SubdivisionMesh &mesh, const SubdivisionMesh::Face &coarseFace) {

    /*! Index of the new face vertex in the subdivided mesh. */
    int32_t vertexIndex = vertexCounter++;

    /*! Coordinates of the new face vertex. */
    Vec3f &vertexCoordinates = mesh.getVertexBuffer()[vertexIndex];

    /*! The new vertex is the average of the vertices adjacent to the coarse face. */
    for (size_t i=0 ; i < coarseFace.vertexCount() ; i++) vertexCoordinates += coarseFace.getVertex(i).getCoordinates() / (float)coarseFace.vertexCount();

    /*! Associate the new vertex with the adjacent faces in the subdivided mesh. */
    for (size_t i=0 ; i < coarseFace.vertexCount() ; i++) mesh.getIndexBuffer()[mesh.getIndex(coarseFace.getEdge(i).getIndex()) + 2] = vertexIndex;

}

void catmullClarkMoveVertex(SubdivisionMesh &mesh, SubdivisionMesh::HalfEdge coarseEdge, int32_t valence) {

    /*! Index of the old vertex in the subdivided mesh. */
    int32_t vertexIndex = vertexCounter++;

    /*! Coordinates of the new vertex. */
    Vec3f &vertexCoordinates = mesh.getVertexBuffer()[vertexIndex];

    /*! The new vertex is the average of the new face vertices adjacent to the old vertex. */
    for (size_t i=0 ; i < valence ; i++, coarseEdge = coarseEdge.getNextVertexEdge()) vertexCoordinates += mesh.getFace(coarseEdge.getIndex()).getVertex(2).getCoordinates() / (float)valence;

    /*! Plus the weighted average of the new edge vertices adjacent to the old vertex. */
    for (size_t i=0 ; i < valence ; i++, coarseEdge = coarseEdge.getNextVertexEdge()) vertexCoordinates += mesh.getFace(coarseEdge.getIndex()).getVertex(1).getCoordinates() * 2.0f / (float)valence;

    /*! Plus the coordinates of the old vertex. */
    vertexCoordinates = (vertexCoordinates + coarseEdge.getVertex().getCoordinates() * (valence - 3.0f)) / (float)valence;

    /*! Associate the new vertex with the adjacent faces in the subdivided mesh. */
    for (size_t i=0 ; i < valence ; i++, coarseEdge = coarseEdge.getNextVertexEdge()) mesh.getIndexBuffer()[mesh.getIndex(coarseEdge.getIndex()) + 0] = vertexIndex;

}

SubdivisionMesh subdivideMesh(SubdivisionMesh &coarseMesh) {

    /*! Compute edge adjacency information over the coarse mesh. */
    coarseMesh.commit();  vertexCounter = 0;

    /*! Allocate memory for the subdivided mesh. */
    SubdivisionMesh mesh(coarseMesh.edgeCount() * 2, coarseMesh.edgeCount() * 4, coarseMesh.vertexCount() + coarseMesh.edgeCount() + coarseMesh.faceCount());

    /*! Each new face in the subdivided mesh will have 4 vertices. */
    for (size_t i=0, j=0 ; i < mesh.faceCount() + 1 ; i++, j += 4) mesh.getOffsetBuffer()[i] = j;

    /*! A new vertex is generated for each face in the coarse mesh. */
    for (size_t i=0 ; i < coarseMesh.faceCount() ; i++) catmullClarkFaceVertex(mesh, coarseMesh.getFace(i));

    /*! A new vertex is generated for each edge in the coarse mesh. */
    for (size_t i=0 ; i < coarseMesh.edgeCount() * 2 ; i++) catmullClarkEdgeVertex(mesh, coarseMesh.getOppositeEdge(i));

    /*! A new vertex is generated for each vertex in the coarse mesh. */
    for (size_t i=0 ; i < coarseMesh.vertexCount() ; i++) catmullClarkMoveVertex(mesh, coarseMesh.getVertex(i).getEdge(), coarseMesh.getVertex(i).edgeCount());

    /*! The subdivided mesh. */
    return(mesh);

}

