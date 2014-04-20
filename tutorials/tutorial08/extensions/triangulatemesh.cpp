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

#include <map>
#include "triangulatemesh.h"

size_t triangulatedMeshVertices(size_t numFaces, size_t numEdges, size_t numVertices, int subdivisionLevel) {

    if (subdivisionLevel == 0) return(numVertices);  numVertices += numEdges + numFaces;
    for (size_t i=1 ; i < subdivisionLevel ; i++) numVertices += numEdges * ::pow(4.0, i - 1) * 6;
    return(numVertices);

}

void triangulateFace(const SubdivisionMesh::Face &face, Triangle *triangles) {

    for (size_t i=0, j=1, k=2 ; j < face.vertexCount() - 1 ; i++, j++, k++) {

        triangles[i].v0 = face.getVertex(0).getIndex();
        triangles[i].v1 = face.getVertex(j).getIndex();
        triangles[i].v2 = face.getVertex(k).getIndex();

    }

}

void triangulateMesh(RTCScene sceneID, unsigned int meshID, SubdivisionMesh &mesh) {

    /*! Map the triangle mesh vertex buffer from Embree space into user space. */
    Vertex *vertices = (Vertex *) rtcMapBuffer(sceneID, meshID, RTC_VERTEX_BUFFER);

    /*! Copy vertex data from the subdivision mesh into the triangle mesh buffer. */
    for (size_t i=0 ; i < mesh.vertexCount() ; i++) { Vec3f p = mesh.getCoordinates(i);  vertices[i].x = p.x;  vertices[i].y = p.y;  vertices[i].z = p.z; }

    /*! Unmap the triangle mesh buffer. */
    rtcUnmapBuffer(sceneID, meshID, RTC_VERTEX_BUFFER);

    /*! Map the triangle mesh index buffer from Embree space into user space. */
    Triangle *triangles = (Triangle *) rtcMapBuffer(sceneID, meshID, RTC_INDEX_BUFFER);

    /* Copy vertex indices into the triangle mesh buffer. */
    for (size_t i=0, j=0 ; i < mesh.faceCount() ; j += mesh.getFace(i).vertexCount() - 2, i++) triangulateFace(mesh.getFace(i), &triangles[j]);

    /*! Unmap the triangle mesh buffer. */
    rtcUnmapBuffer(sceneID, meshID, RTC_INDEX_BUFFER);

}

