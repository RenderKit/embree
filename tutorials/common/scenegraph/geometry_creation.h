// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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

    Ref<Node> createSphere        (const Vec3fa& center, const float radius, Ref<MaterialNode> materiall = nullptr);
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
