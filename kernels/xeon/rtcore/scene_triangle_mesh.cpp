// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "scene_triangle_mesh.h"
#include "scene.h"

namespace embree
{
  TriangleMeshScene::TriangleMesh::TriangleMesh (Scene* parent, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps)
    : Geometry(parent,TRIANGLE_MESH,numTriangles,flags), mask(-1), built(false),
      triangles(NULL), numTriangles(numTriangles), mappedTriangles(false), needTriangles(false),
      numVertices(numVertices), numTimeSteps(numTimeSteps), needVertices(false)
  {
    mappedVertices[0] = mappedVertices[1] = false;
    vertices_[0] = vertices_[1] = NULL;
    triangles = (Triangle*) alignedMalloc(numTriangles*sizeof(Triangle));
    for (size_t i=0; i<numTimeSteps; i++)
      vertices_[i]  = (Vec3fa*) alignedMalloc(numVertices*sizeof(Vec3fa));
    enabling();
  }
  
  void TriangleMeshScene::TriangleMesh::enabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->numTriangleMeshes ,1); 
    else                   atomic_add(&parent->numTriangleMeshes2,1); 
  }
  
  void TriangleMeshScene::TriangleMesh::disabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->numTriangleMeshes ,-1); 
    else                   atomic_add(&parent->numTriangleMeshes2,-1); 
  }

  TriangleMeshScene::TriangleMesh::~TriangleMesh () 
  {
    alignedFree(triangles);
    for (size_t i=0; i<2; i++)
      alignedFree(vertices_[i]);
  }

  void TriangleMeshScene::TriangleMesh::split (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) const
  {
    const TriangleMeshScene::TriangleMesh::Triangle& tri = triangle(prim.primID());
    const Vec3fa& v0 = vertex(tri.v[0]);
    const Vec3fa& v1 = vertex(tri.v[1]);
    const Vec3fa& v2 = vertex(tri.v[2]);
    splitTriangle(prim,dim,pos,v0,v1,v2,left_o,right_o);
  }
  
  void TriangleMeshScene::TriangleMesh::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    this->mask = mask; 
  }

  void TriangleMeshScene::TriangleMesh::enable () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::enable();
  }

  void TriangleMeshScene::TriangleMesh::update () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::update();
  }

  void TriangleMeshScene::TriangleMesh::disable () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::disable();
  }

  void TriangleMeshScene::TriangleMesh::erase () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::erase();
  }

  void* TriangleMeshScene::TriangleMesh::map(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      recordError(RTC_INVALID_OPERATION);
      return NULL;
    }

    switch (type) {
    case RTC_INDEX_BUFFER : 
    {
      if (mappedTriangles || (built && isDeformable())) {
        recordError(RTC_INVALID_OPERATION);
        return NULL;
      }
      mappedTriangles = true; 
      atomic_add(&parent->numMappedBuffers,1); 
      return triangles;
    }
    case RTC_VERTEX_BUFFER0: 
    {
      if (mappedVertices[0]) {
        recordError(RTC_INVALID_OPERATION);
        return NULL;
      }
      mappedVertices[0] = true; 
      atomic_add(&parent->numMappedBuffers,1); 
      return vertices_[0];
    }
    case RTC_VERTEX_BUFFER1: 
    {
      if (mappedVertices[1]) {
        recordError(RTC_INVALID_OPERATION);
        return NULL;
      }
      mappedVertices[1] = true; 
      atomic_add(&parent->numMappedBuffers,1); 
      return vertices_[1];
    }
    default: 
      recordError(RTC_INVALID_ARGUMENT); 
      return NULL;
    }
  }

  void TriangleMeshScene::TriangleMesh::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }

    switch (type) {
    case RTC_INDEX_BUFFER : 
    {
      if (mappedTriangles) {
        mappedTriangles = false; 
        atomic_add(&parent->numMappedBuffers,-1); 
      } else {
        recordError(RTC_INVALID_OPERATION);
      }
      break;
    }
    case RTC_VERTEX_BUFFER0: 
    {
      if (mappedVertices[0]) {
        mappedVertices[0] = false; 
        atomic_add(&parent->numMappedBuffers,-1); 
      } else {
        recordError(RTC_INVALID_OPERATION);
      }
      break;
    }
    case RTC_VERTEX_BUFFER1: 
    {
      if (mappedVertices[1]) {
        mappedVertices[1] = false; 
        atomic_add(&parent->numMappedBuffers,-1); 
      } else {
        recordError(RTC_INVALID_OPERATION);
      }
      break;
    }
    default: 
      recordError(RTC_INVALID_ARGUMENT);
    }
  }

  void TriangleMeshScene::TriangleMesh::immutable () 
  {
    built = true;
    bool freeTriangles = !(needTriangles || parent->needTriangles);
    bool freeVertices  = !(needVertices  || parent->needVertices);
    if (freeTriangles) {
      if (triangles) alignedFree(triangles); triangles = NULL;
    }
    if (freeVertices) {
      if (vertices_[0]) alignedFree(vertices_[0]); vertices_[0] = NULL; // FIXME: on KNC this does not call our own alignedFree but some other function !!!!!!
      if (vertices_[1]) alignedFree(vertices_[1]); vertices_[1] = NULL;
    }
  }

  bool TriangleMeshScene::TriangleMesh::verify () 
  {
    float range = sqrtf(0.5f*FLT_MAX);
    for (size_t i=0; i<numTriangles; i++) {
      if (triangles[i].v[0] >= numVertices) return false;
      if (triangles[i].v[1] >= numVertices) return false;
      if (triangles[i].v[2] >= numVertices) return false;
    }
    for (size_t j=0; j<2; j++) {
      if (vertices_[j] == NULL) continue;
      Vec3fa* verts = vertices_[j];
      for (size_t i=0; i<numVertices; i++) {
        if (verts[i].x < -range || verts[i].x > range) return false;
        if (verts[i].y < -range || verts[i].y > range) return false;
        if (verts[i].z < -range || verts[i].z > range) return false;
      }
    }
    return true;
  }
}
