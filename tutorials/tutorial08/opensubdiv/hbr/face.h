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

#include "opensubdiv/hbr/mesh.h"
#include "opensubdiv/hbr/vertex.h"

namespace OpenSubdiv {

template <class T> class HbrFace {
public:

    /*! Subdivision occurs within Embree thus all faces created outside Embree are level 0 faces. */
    int GetDepth() const { return(0); }

    /*! Return the half edge at the given index in the face. */
    HbrHalfedge<T> *GetEdge(int index) const { HbrVertex<T> *vertex = GetVertex(index);  return(vertex->GetEdge(GetVertexID((index + 1) % GetNumVertices()))); }

    /*! Return the index of the face in the mesh. */
    int GetID() const { return(id); }

    /*! Return a pointer to the mesh containing the face. */
    HbrMesh<T> *GetMesh() const { return(mesh); }

    /*! Return the number of vertices in the face. */
    int GetNumVertices() const { return(mesh->offsets[id + 1] - mesh->offsets[id]); }

    /*! Return the vertex at the given index in the face. */
    HbrVertex<T> *GetVertex(int index) const { return(mesh->vertices[GetVertexID(index)]); }

    /*! Return the ID of the vertex at the given index in the face. */
    int GetVertexID(int index) const { assert(index >= 0 && index < GetNumVertices());  return(mesh->indices[mesh->offsets[id] + index]); }

    /*! Subdivision occurs within Embree so all faces in the OpenSubdiv mesh are coarse. */
    bool IsCoarse() const { return(true); }

    /*! Indicates whether the face has been marked as a hole. */
    bool IsHole() const { return(mesh->holes[id] != 0); }

    /*! Mark the face as a hole or not a hole. */
    void SetHole(bool value = true) { mesh->holes[id] = (value) ? 1 : 0; }

private:

    /*! HbrMesh uses the private Initialize method. */
    friend class HbrMesh<T>;

    /*! Index of the face in the mesh. */
    int id;

    /*! Parent mesh. */
    HbrMesh<T> *mesh;

    /*! Add the face to the mesh. */
    void Initialize(int faceIndex, HbrMesh<T> *parentMesh, int vertexCount, const int *vertexIndices);

};

template <typename T> void HbrFace<T>::Initialize(int faceIndex, HbrMesh<T> *parentMesh, int vertexCount, const int *vertexIndices) {

    /*! Index of the face in the mesh. */
    id = faceIndex;

    /*! Parent mesh. */
    mesh = parentMesh;

    /*! Offset into the mesh indices array just past the vertices composing the face. */
    mesh->offsets.push_back(mesh->offsets.back() + vertexCount);

    /*! Indices of the vertices composing the face. */
    for (int i=0 ; i < vertexCount ; i++) mesh->indices.push_back(vertexIndices[i]);

    /*! Vertices keep a list of the adjacent edges. */
    for (int i=0 ; i < vertexCount ; i++) mesh->vertices[vertexIndices[i]]->edges.push_back(new HbrHalfedge<T>(i, this));

    /*! Crease weights for the adjacent half edges are 0 by default. */
    for (int i=0 ; i < vertexCount ; i++) mesh->creases.push_back(0.0f);

    /*! The face is not a hole by default. */
    mesh->holes.push_back(0);

}

} // namespace OpenSubdiv

