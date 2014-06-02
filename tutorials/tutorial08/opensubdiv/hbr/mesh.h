// ========================================================================= //
//  The OpenSubdiv API:                                                      //
//                                                                           //
//     Copyright 2013 Pixar                                                  //
//                                                                           //
//  This partial implementation of the OpenSubdiv API:                       //
//                                                                           //
//     Copyright 2014 Intel Corporation                                      //
//                                                                           //
//  Licensed under the Apache License, Version 2.0 (the "Apache License")    //
//  with the following modification; you may not use this file except in     //
//  compliance with the Apache License and the following modification to it: //
//  Section 6. Trademarks. is deleted and replaced with:                     //
//                                                                           //
//  6. Trademarks. This License does not grant permission to use the trade   //
//     names, trademarks, service marks, or product names of the Licensor    //
//     and its affiliates, except as required to comply with Section 4(c) of //
//     the License and to reproduce the content of the NOTICE file.          //
//                                                                           //
//  You may obtain a copy of the Apache License at                           //
//                                                                           //
//      http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                           //
//  Unless required by applicable law or agreed to in writing, software      //
//  distributed under the Apache License with the above modification is      //
//  distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY //
//  KIND, either express or implied. See the Apache License for the specific //
//  language governing permissions and limitations under the Apache License. //
// ========================================================================= //

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include "extensions/api.h"
#include "opensubdiv/hbr/catmark.h"
#include "opensubdiv/hbr/subdivision.h"

namespace OpenSubdiv {

template <typename T> class HbrFace;
template <typename T> class HbrVertex;

template <typename T> class HbrMesh {
public:

    /*! The subdivision algorithm is specified by the HbrSubdivision object subtype (currently ignored). */
    HbrMesh(HbrSubdivision<T> *subdivisionMethod) : offsets(std::vector<int32_t>(1, 0)), subdivisionMethod(subdivisionMethod) { }

    /*! Destructor. */
   ~HbrMesh();

    /*! The mesh specification is complete, create the corresponding mesh object in the Embree scene state. */
    void Finish(RTCScene sceneID);

    /*! Retrieve the face with the given ID. */
    HbrFace<T> *GetFace(int id) const { return((id >= faces.size()) ? NULL : faces[id]); }

    /*! Subdivision occurs within Embree thus all faces created outside Embree are coarse. */
    int GetNumCoarseFaces() const { return(GetNumFaces()); }

    /*! Return the number of faces in the mesh. */
    int GetNumFaces() const { return(faces.size()); }

    /*! Return the number of vertices in the mesh. */
    int GetNumVertices() const { int vertexCount=0;  for (size_t i=0 ; i < vertices.size() ; i++) if (vertices[i]) vertexCount++;  return(vertexCount); }

    /*! Return the subdivision object. */
    HbrSubdivision<T> *GetSubdivision() const { return(subdivisionMethod); }

    /*! Retrieve the vertex with the given ID. */
    HbrVertex<T> *GetVertex(int id) const { return((id >= vertices.size()) ? NULL : vertices[id]); }

    /*! Add a face defined as a list of vertex IDs to the mesh. */
    HbrFace<T> *NewFace(int vertexCount, const int *vertexIndices, int);

    /*! Add a face defined as a list of vertices to the mesh. */
    HbrFace<T> *NewFace(int vertexCount, HbrVertex<T> **vertexList, HbrFace<T> *, int);

    /*! Add a vertex with the given ID to the mesh. */
    HbrVertex<T> *NewVertex(int id, const T &vertexData);

    /*! Add a vertex to the mesh, with an ID assigned by the mesh. */
    HbrVertex<T> *NewVertex(const T &vertexData) { return(NewVertex(vertices.size(), vertexData)); }

    /*! Add an empty vertex to the mesh, with an ID assigned by the mesh. */
    HbrVertex<T> *NewVertex() { return(NewVertex(vertices.size(), T())); }

    /*! Print several mesh statistics. */
    void PrintStats(std::ostream &out);

private:

    /*! HbrFace accesses private member variables. */
    friend class HbrFace<T>;

    /*! HbrVertex accesses private member variables. */
    friend class HbrVertex<T>;

    /*! Crease weights for each half edge. */
    std::vector<float> creases;

    /*! Vertex data. */
    std::vector<T> data;

    /*! OpenSubdiv faces point into the "indices" array. */
    std::vector<HbrFace<T> *> faces;

    /*! Markers to indicate which faces are holes. */
    std::vector<int8_t> holes;

    /*! Vertex indices composing each face. */
    std::vector<int32_t> indices;

    /*! Offsets into the indices array indexed by face. */
    std::vector<int32_t> offsets;

    /*! The subdivision algorithm is specified by the HbrSubdivision object subtype (currently ignored). */
    HbrSubdivision<T> *subdivisionMethod;

    /*! OpenSubdiv vertices point into the "data" array. */
    std::vector<HbrVertex<T> *> vertices;

    /*! Write data into an Embree subdivision mesh buffer. */
    void WriteEmbreeMeshBuffer(RTCScene sceneID, unsigned meshID, RTCXBufferType bufferID, void *sourceData, size_t sourceSize);

};

} // namespace OpenSubdiv

#include "opensubdiv/hbr/face.h"
#include "opensubdiv/hbr/vertex.h"

namespace OpenSubdiv {

template <typename T> HbrMesh<T>::~HbrMesh() {

    /*! Destroy the faces. */
    for (size_t i=0 ; i < faces.size() ; i++) delete faces[i];

    /*! Destroy the vertices. */
    for (size_t i=0 ; i < vertices.size() ; i++) delete vertices[i];

}

template <>
void HbrMesh<Vec3f>::Finish(RTCScene sceneID) {

    /*! Construct an Embree subdivision mesh object (mesh is assumed to have no boundary edges). */
    unsigned int meshID = rtcxNewSubdivisionMesh(sceneID, RTC_GEOMETRY_STATIC, faces.size(), indices.size() / 2, vertices.size());

    /*! Copy vertex coordinates into the Embree mesh. */
    WriteEmbreeMeshBuffer(sceneID, meshID, RTCX_VERTEX_BUFFER, &data[0], data.size() * sizeof(Vec3f));

    /*! Copy vertex indices into the Embree mesh. */
    WriteEmbreeMeshBuffer(sceneID, meshID, RTCX_INDEX_BUFFER, &indices[0], indices.size() * sizeof(int32_t));

    /*! Copy face offsets into the Embree mesh. */
    WriteEmbreeMeshBuffer(sceneID, meshID, RTCX_OFFSET_BUFFER, &offsets[0], offsets.size() * sizeof(int32_t));

    /*! Copy edge crease weights into the Embree mesh. */
    WriteEmbreeMeshBuffer(sceneID, meshID, RTCX_DATA_BUFFER0, &creases[0], creases.size() * sizeof(float));

    /*! Copy face holes into the Embree mesh. */
    WriteEmbreeMeshBuffer(sceneID, meshID, RTCX_DATA_BUFFER1, &holes[0], holes.size() * sizeof(int8_t));

}

template <typename T> HbrFace<T> *HbrMesh<T>::NewFace(int vertexCount, const int *vertexIndices, int) {

    /*! Verify the face vertices exist. */
    for (int i=0 ; i < vertexCount ; i++) if (GetVertex(vertexIndices[i]) == NULL) return(NULL);

    /*! Resize the face array. */
    int id = faces.size();  faces.resize(id + 1, NULL);

    /*! Create a fresh face. */
    HbrFace<T> *face = new HbrFace<T>();

    /*! Initialize the face. */
    face->Initialize(id, this, vertexCount, vertexIndices);  faces[id] = face;  return(face);

}

template <typename T> HbrFace<T> *HbrMesh<T>::NewFace(int vertexCount, HbrVertex<T> **vertexList, HbrFace<T> *, int) {

    /*! Gather the indices of the vertices composing the face. */
    int32_t vertexIndices[vertexCount];  for (int i=0 ; i < vertexCount ; i++) vertexIndices[i] = vertexList[i]->GetID();

    /*! Resize the face array. */
    int id = faces.size();  faces.resize(id + 1, NULL);

    /*! Create a fresh face. */
    HbrFace<T> *face = new HbrFace<T>();

    /*! Initialize the face. */
    face->Initialize(id, this, vertexCount, vertexIndices);  faces[id] = face;  return(face);

}

template <typename T> HbrVertex<T> *HbrMesh<T>::NewVertex(int id, const T &vertexData) {

    /*! Optionally resize the vertex and vertex data arrays. */
    if (vertices.size() <= id) vertices.resize(id + 1, NULL), data.resize(id + 1);

    /*! Create a fresh vertex. */
    HbrVertex<T> *vertex = vertices[id];  delete vertex;  vertex = new HbrVertex<T>();

    /*! Initialize the vertex. */
    vertex->Initialize(id, this, vertexData);  vertices[id] = vertex;  return(vertex);

}

template <typename T> void HbrMesh<T>::PrintStats(std::ostream &out) {

    /*! Number of actual vertices in the mesh. */
    int vertexCount = GetNumVertices();  out << "\n Mesh contains " << vertexCount << " vertices.\n";

    /*! Number of actual faces in the mesh. */
    int faceCount = GetNumFaces();  out << " Mesh contains " << faceCount << " faces.\n";

    /*! Average number of sides per face. */
    int faceSides = indices.size();  out << " Average face sidedness " << (float) faceSides / faceCount << ".\n\n";

}

template <typename T> void HbrMesh<T>::WriteEmbreeMeshBuffer(RTCScene sceneID, unsigned meshID, RTCXBufferType bufferID, void *sourceData, size_t sourceSize) {

    /*! Map a mesh buffer from Embree space into user space. */
    void *buffer = rtcxMapBuffer(sceneID, meshID, bufferID);

    /*! Write data into the Embree mesh buffer. */
    memcpy(buffer, sourceData, sourceSize);

    /*! Unmap the Embree mesh buffer. */
    rtcxUnmapBuffer(sceneID, meshID, bufferID);

}

} // namespace OpenSubdiv

