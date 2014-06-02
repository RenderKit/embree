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

namespace OpenSubdiv {

template <typename T> class HbrHalfedge {
public:

    /*! Return the destination vertex of the half edge. */
    HbrVertex<T> *GetDestVertex() const { HbrHalfedge<T> *edge = GetNext();  return(edge->GetVertex()); }

    /*! Return the destination vertex of the half edge. */
    HbrVertex<T> *GetDestVertex(HbrMesh<T> *parentMesh) const { HbrHalfedge<T> *edge = GetNext();  return(edge->GetVertex(parentMesh)); }

    /*! Return the ID of the destination vertex of the half edge. */
    int GetDestVertexID() const { return(face->GetVertexID((index + 1) % face->GetNumVertices())); }

    /*! Return the face adjacent to the half edge. */
    HbrFace<T> *GetFace() const { return(face); }

    /*! Return the face adjacent to the half edge. */
    HbrFace<T> *GetLeftFace() const { return(GetFace()); }

    /*! Return a pointer to the mesh containing the half edge. */
    HbrMesh<T> *GetMesh() const { return(face->GetMesh()); }

    /*! Return the next edge in clockwise order around a face. */
    HbrHalfedge<T> *GetNext() const { return(face->GetEdge((index + 1) % face->GetNumVertices())); }

    /*! Return the opposing half edge if it exists. */
    HbrHalfedge<T> *GetOpposite() const { HbrVertex<T> *vertex = GetNext()->GetVertex();  return(vertex->GetEdge(GetVertex())); }

    /*! Return the source vertex of the half edge. */
    HbrVertex<T> *GetOrgVertex() const { return(GetVertex()); }

    /*! Return the source vertex of the half edge. */
    HbrVertex<T> *GetOrgVertex(HbrMesh<T> *parentMesh) const { return(GetVertex(parentMesh)); }

    /*! Return the ID of the source vertex of the half edge. */
    int GetOrgVertexID() const { return(GetVertexID()); }

    /*! Return the next edge in counterclockwise order around a face. */
    HbrHalfedge<T> *GetPrev() const { int i = (index == 0) ? face->GetNumVertices() : index;  return(face->GetEdge(i - 1)); }

    /*! Return the face adjacent to the opposite half edge. */
    HbrFace<T> *GetRightFace() const { HbrHalfedge<T> *edge = GetOpposite();  return(edge ? edge->GetFace() : NULL); }

    /*! Return the crease weight for the edge. */
    float GetSharpness() const { HbrMesh<T> *mesh = GetMesh();  return(mesh->creases[GetID()]); }

    /*! Return the vertex adjacent to the half edge. */
    HbrVertex<T> *GetVertex() const { return(face->GetVertex(index)); }

    /*! Return the vertex adjacent to the half edge. */
    HbrVertex<T> *GetVertex(HbrMesh<T> *parentMesh) const { return(parentMesh->GetVertex(GetVertexID())); }

    /*! Return the ID of the vertex adjacent to the half edge. */
    int GetVertexID() const { return(face->GetVertexID(index)); }

    /*! Embree subdivision meshes do not currently allow boundaries. */
//  bool IsBoundary() const { return(false); }

    /*! Subdivision occurs within Embree so all edges in the OpenSubdiv mesh are coarse. */
    bool IsCoarse() const { return(true); }

    /*! Set the crease weight for the edge. */
    void SetSharpness(float creaseWeight) { HbrMesh<T> *mesh = GetMesh();  mesh->creases[GetID()] = creaseWeight; }

private:

    /*! HbrFace uses the private constructor. */
    friend class HbrFace<T>;

    /*! Face adjacent to the half edge. */
    HbrFace<T> *face;

    /*! Index of the half edge in the face. */
    int index;

    /*! Private constructor. */
    HbrHalfedge(int index, HbrFace<T> *face) : face(face), index(index) {}

    /*! Index of the half edge in the mesh. */
    int GetID() const { HbrMesh<T> *mesh = face->GetMesh();  return(mesh->offsets[face->GetID() + index]); }

};

} // namespace OpenSubdiv

