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
#include "api.h"
#include "catmullclark.h"
#include "subdivisionmesh.h"
#include "triangulatemesh.h"

/*! Embree identifier for the triangle mesh corresponding to the subdivision mesh. */
unsigned int meshID;

/*! Requested level of subdivision set in tutorial08.cpp. */
int subdivisionLevel = 1;

/*! Subdivision mesh. */
SubdivisionMesh subdivisionMesh;

void rtcxCommit(RTCScene sceneID) {

    /*! Temporary mesh used during subdivision. */
    SubdivisionMesh catmullClarkMesh = subdivisionMesh;

    /*! Subdivide the mesh using Catmull-Clark. */
    for (size_t i=0 ; i < subdivisionLevel ; i++) catmullClarkMesh = subdivideMesh(catmullClarkMesh);

    /*! Triangulate the subdivided mesh. */
    //triangulateMesh(sceneID, meshID, catmullClarkMesh);

    /*! Commit the Embree scene geometry. */
    //rtcCommit(sceneID);

}

void *rtcxMapBuffer(RTCScene sceneID, unsigned geomID, RTCXBufferType type) {

    /*! Pointer to the edge crease weight buffer of the subdivision mesh. */
    if (type == RTCX_DATA_BUFFER0) return(subdivisionMesh.getCreaseBuffer());

    /*! Pointer to the hole buffer of the subdivision mesh. */
    if (type == RTCX_DATA_BUFFER1) return(subdivisionMesh.getHoleBuffer());

    /*! Pointer to the index buffer of the subdivision mesh. */
    if (type == RTCX_INDEX_BUFFER) return(subdivisionMesh.getIndexBuffer());

    /*! Pointer to the offset buffer of the subdivision mesh. */
    if (type == RTCX_OFFSET_BUFFER) return(subdivisionMesh.getOffsetBuffer());

    /*! Pointer to the vertex buffer of the subdivision mesh. */
    if (type == RTCX_VERTEX_BUFFER) return(subdivisionMesh.getVertexBuffer());  return(NULL);

}

unsigned rtcxNewSubdivisionMesh(RTCScene sceneID, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices) {

    /*! Number of faces in the mesh following subdivision and triangulation. */
    size_t numTriangles = (subdivisionLevel > 0) ? numEdges * ::pow(4.0, subdivisionLevel) : numFaces * 2;

    /*! Number of vertices in the mesh following subdivision. */
    //size_t numTriangleVertices = triangulatedMeshVertices(numFaces, numEdges, numVertices, subdivisionLevel);

    /*! Allocate storage for the triangle mesh corresponding to the subdivision mesh. */
    //meshID = rtcNewTriangleMesh(sceneID, flags, numTriangles, numTriangleVertices);

    /*! Allocate storage for the subdivision mesh. */
    subdivisionMesh = SubdivisionMesh(numFaces, numEdges, numVertices);  return(meshID);

}

