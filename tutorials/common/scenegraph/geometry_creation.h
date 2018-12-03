// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "scenegraph.h"

namespace embree
{  
  namespace SceneGraph
  {
    Ref<Node> createTrianglePlane (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material = nullptr);
    Ref<Node> createQuadPlane     (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGridPlane     (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material = nullptr);
    Ref<Node> createSubdivPlane   (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, float tessellationRate, Ref<MaterialNode> material = nullptr);
    
    Ref<Node> createTriangleSphere(const Vec3fa& center, const float radius, size_t numPhi, Ref<MaterialNode> material = nullptr);
    Ref<Node> createQuadSphere    (const Vec3fa& center, const float radius, size_t numPhi, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGridSphere    (const Vec3fa& center, const float radius, size_t size, Ref<MaterialNode> material = nullptr);
    Ref<Node> createSubdivSphere  (const Vec3fa& center, const float radius, size_t numPhi, float tessellationRate, Ref<MaterialNode> material = nullptr);
    Ref<Node> createSphereShapedHair(const Vec3fa& center, const float radius, Ref<MaterialNode> material = nullptr);
    Ref<Node> createPointSphere   (const Vec3fa& center, const float radius, const float pointRadius, size_t numPhi, PointSubtype subtype, Ref<MaterialNode> materiall = nullptr);

    Ref<Node> createHairyPlane    (int hash, const Vec3fa& pos, const Vec3fa& dx, const Vec3fa& dy, const float len, const float r, size_t numHairs, CurveSubtype subtype, Ref<MaterialNode> material = nullptr);

    Ref<Node> createGarbageTriangleMesh (int hash, size_t numTriangles, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageQuadMesh (int hash, size_t numQuads, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageGridMesh (int hash, size_t numGrids, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageHair (int hash, size_t numHairs, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageLineSegments (int hash, size_t numLineSegments, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageSubdivMesh (int hash, size_t numFaces, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbagePointSet(int hash, size_t numPoints, bool mblur, Ref<MaterialNode> material = nullptr);
  }
}
