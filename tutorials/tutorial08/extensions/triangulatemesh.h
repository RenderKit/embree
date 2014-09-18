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

#pragma once

#include "tutorial/tutorial_device.h"
#include "subdivisionmesh.h"

/*! Compute the number of vertices in the subdivided and triangulated mesh. */
size_t triangulatedMeshVertices(size_t numFaces, size_t numEdges, size_t numVertices, int subdivisionLevel);

/*! Generate an Embree triangle mesh from a subdivision mesh. */
void triangulateMesh(RTCScene sceneID, unsigned int meshID, SubdivisionMesh &mesh);
void triangulateFace(const SubdivisionMesh::Face &face, Triangle *triangles);

