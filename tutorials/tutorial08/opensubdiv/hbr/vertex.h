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

#include "opensubdiv/hbr/halfedge.h"

namespace OpenSubdiv {

template <typename T> class HbrVertex {
public:

    /*! Destructor. */
   ~HbrVertex() { for (size_t i=0 ; i < edges.size() ; i++) delete edges[i]; }

    /*! Subdivision occurs within Embree so all adjacent edges are coarse. */
    int GetCoarseValence() const { return(GetValence()); }

    /*! Return the vertex data. */
    T &GetData() { return(mesh->vertices[id]); }

    /*! Return the vertex data. */
    const T &GetData() const { return(mesh->vertices[id]); }

    /*! Return the adjacent half edge denoted by the given destination vertex. */
    HbrHalfedge<T> *GetEdge(const HbrVertex<T> *vertex) const { return(GetEdge(vertex->GetID())); }

    /*! Return the adjacent half edge denoted by the destination vertex at the given index in the mesh. */
    HbrHalfedge<T> *GetEdge(int index) const { for (size_t i=0 ; i < GetValence() ; i++) if (index == edges[i]->GetDestVertexID()) return(edges[i]);  return(NULL); }

    /*! Return any face adjacent to the vertex. */
    HbrFace<T> *GetFace() const { return(IsConnected() ? edges[0]->GetFace() : NULL); }

    /*! Return the index of the vertex in the mesh. */
    int GetID() const { return(id); }

    /*! Return a half edge adjacent to the vertex. */
    HbrHalfedge<T> *GetIncidentEdge() const { return(IsConnected() ? edges[0] : NULL); }

    /*! Return a pointer to the mesh containing the vertex. */
    HbrMesh<T> *GetMesh() const { return(mesh); }

    /*! Return the next edge in counterclockwise order around the vertex. */
    HbrHalfedge<T> *GetNextEdge(const HbrHalfedge<T> *edge) const { return(edge->GetPrev()->GetOpposite()); }

    /*! Return the next edge in clockwise order around the vertex. */
    HbrHalfedge<T> *GetPreviousEdge(const HbrHalfedge<T>* edge) const { return(edge->GetOpposite()->GetNext()); }

    /*! Embree subdivision meshes do not currently support vertex crease weights. */
    float GetSharpness() const { return(0.0f); }

    /*! Return the valence of the vertex. */
    int GetValence() const { return(edges.size()); }

    /*! Indicate if the vertex has at least 1 adjacent edge. */
    bool IsConnected() const { return(edges.size() > 0); }

    /*! Embree subdivision meshes DO allow extraordinary vertices. */
    bool IsExtraordinary() const { return(extraordinary); }

    /*! Embree subdivision meshes do not currently allow singular vertices. */
//  bool IsSingular() const { return(false); }

    /*! Embree subdivision meshes do not currently allow boundaries. */
//  bool OnBoundary() const { return(false); }

    /*! Mark the vertex as extraordinary. */
    void SetExtraordinary() { extraordinary = true; }

private:

    /*! HbrFace accesses private member variables. */
    friend class HbrFace<T>;

    /*! HbrMesh uses the private Initialize method. */
    friend class HbrMesh<T>;

    /*! Pointers to half edges adjacent to the vertex. */
    std::vector<HbrHalfedge<T> *> edges;

    /*! Indicates if the vertex is extraordinary. */
    bool extraordinary;

    /*! Index of the vertex in the mesh. */
    int id;

    /*! Pointer to the mesh containing the vertex. */
    HbrMesh<T> *mesh;

    /*! Add the vertex to the mesh. */
    void Initialize(int vertexIndex, HbrMesh<T> *parentMesh, const T &vertexData);

};

template <typename T> void HbrVertex<T>::Initialize(int vertexIndex, HbrMesh<T> *parentMesh, const T &vertexData) {

    /*! The vertex is not extraordinary by default. */
    extraordinary = false;

    /*! Index of the vertex in the mesh. */
    id = vertexIndex;

    /*! Parent mesh. */
    mesh = parentMesh;

    /*! Store the vertex data in the mesh. */
    mesh->data[id] = vertexData;

}

} // namespace OpenSubdiv

