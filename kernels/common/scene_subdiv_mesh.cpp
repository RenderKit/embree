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

#include "scene_subdiv_mesh.h"
#include "scene.h"

namespace embree
{
  SubdivMesh::SubdivMesh (Scene* parent, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, size_t numTimeSteps)
    : Geometry(parent,SUBDIV_MESH,numFaces,flags), 
      mask(-1), 
      numTimeSteps(numTimeSteps),
      numFaces(numFaces), 
      numEdges(numEdges), 
      numVertices(numVertices),
      halfEdges(NULL)
  {
    for (size_t i=0; i<numTimeSteps; i++) {
       vertices[i].init(numVertices,sizeof(Vec3fa));
     }

    DBG_PRINT(numFaces);
    DBG_PRINT(numEdges);
    DBG_PRINT(numVertices);

    vertexIndices.init(numEdges,sizeof(unsigned int));
    vertexOffsets.init(numFaces,sizeof(unsigned int));
  }
  
  void SubdivMesh::enabling() 
  { 
    if (numTimeSteps == 1) { atomic_add(&parent->numSubdivPatches ,numFaces); }
    else                   { atomic_add(&parent->numSubdivPatches2,numFaces); }
  }
  
  void SubdivMesh::disabling() 
  { 
    if (numTimeSteps == 1) { atomic_add(&parent->numSubdivPatches ,-(ssize_t)numFaces); }
	else                   { atomic_add(&parent->numSubdivPatches2, -(ssize_t)numFaces); }
  }

  void SubdivMesh::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      process_error(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return;
    }
    this->mask = mask; 
  }

  void SubdivMesh::setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride) 
  { 
    if (parent->isStatic() && parent->isBuild()) {
      process_error(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return;
    }

    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3)) {
      process_error(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");
      return;
    }

    /* verify that all vertex accesses are 16 bytes aligned */
#if defined(__MIC__)
    if (type == RTC_VERTEX_BUFFER0 || type == RTC_VERTEX_BUFFER1) {
      if (((size_t(ptr) + offset) & 0xF) || (stride & 0xF)) {
        process_error(RTC_INVALID_OPERATION,"data must be 16 bytes aligned");
        return;
      }
    }
#endif

    switch (type) {
    case RTC_INDEX_BUFFER  : 
      vertexIndices.set(ptr,offset,stride); 
      break;

    case RTC_OFFSET_BUFFER  : 
      vertexOffsets.set(ptr,offset,stride); 
      break;

    case RTC_VERTEX_BUFFER0: 
      vertices[0].set(ptr,offset,stride); 
      if (numVertices) {
        /* test if array is properly padded */
        volatile int w = *((int*)&vertices[0][numVertices-1]+3); // FIXME: is failing hard avoidable?
      }
      break;
    case RTC_VERTEX_BUFFER1: 
      vertices[1].set(ptr,offset,stride); 
      if (numVertices) {
        /* test if array is properly padded */
        volatile int w = *((int*)&vertices[1][numVertices-1]+3); // FIXME: is failing hard avoidable?
      }
      break;
    default: 
      process_error(RTC_INVALID_ARGUMENT,"unknown buffer type");
      break;
    }
  }

  void* SubdivMesh::map(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      process_error(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return NULL;
    }

    switch (type) {
    case RTC_INDEX_BUFFER   : return vertexIndices.map(parent->numMappedBuffers);
    case RTC_OFFSET_BUFFER  : return vertexOffsets.map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER0 : return vertices[0].map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER1 : return vertices[1].map(parent->numMappedBuffers);
    default                 : process_error(RTC_INVALID_ARGUMENT,"unknown buffer type"); return NULL;
    }
  }

  void SubdivMesh::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      process_error(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return;
    }

    switch (type) {
    case RTC_INDEX_BUFFER   : vertexIndices.unmap(parent->numMappedBuffers); break;
    case RTC_OFFSET_BUFFER  : vertexOffsets.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER0 : vertices[0].unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER1 : vertices[1].unmap(parent->numMappedBuffers); break;
    default                 : process_error(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
    }
  }

  void SubdivMesh::setUserData (void* ptr, bool ispc) {
    userPtr = ptr;
  }

  void SubdivMesh::immutable () 
  {
    bool freeVertices  = !parent->needVertices;
    if (freeVertices ) vertices[0].free();
    if (freeVertices ) vertices[1].free();
  }

  void SubdivMesh::initializeHalfEdgeStructures ()
  {
    numHalfEdges = numFaces * 4;

    halfEdges = (HalfEdge*)os_malloc(numHalfEdges * sizeof(HalfEdge));

    /*! initialize all four half-edges for each face */
    for (size_t i=0;i<numFaces;i++)
      {
        const unsigned int halfEdgeIndex = vertexOffsets[i];
        for (size_t j=0;j<4;j++)
          {
            halfEdges[i*4+j].vtx_index      = vertexIndices[halfEdgeIndex + j];
            halfEdges[i*4+j].halfedge_id    = i*4+j;
            halfEdges[i*4+j].opposite_index = (unsigned int)-1;
          }
      }

    /*! find opposite half-edges */
    std::map<size_t,unsigned int> edgeMap;

    for (size_t i=0;i<numHalfEdges;i++)
      {
        unsigned int start = halfEdges[i].getStartVertexIndex();
        unsigned int end   = halfEdges[i].getEndVertexIndex();
        if (end < start) std::swap(start,end);
        size_t value = ((size_t)start << 32) | (size_t)end; // FIXME: does not work in 32 bit mode
        std::map<size_t,unsigned int>::iterator found = edgeMap.find(value);
        if (found != edgeMap.end())
          {
            halfEdges[i].opposite_index = found->second;
            halfEdges[ found->second ].opposite_index = i;
          }
        else
          {
            edgeMap[value] = i;
          }
      }

#if 0
    for (size_t i=0;i<numHalfEdges;i++)
      std::cout << "Half-Edge " << i << " " << halfEdges[i] << std::endl;
#endif

  }

  bool SubdivMesh::verify () 
  {
    float range = sqrtf(0.5f*FLT_MAX);
    for (size_t j=0; j<numTimeSteps; j++) {
      BufferT<Vec3fa>& verts = vertices[j];
      for (size_t i=0; i<numVertices; i++) {
        if (!(verts[i].x > -range && verts[i].x < range)) return false;
	if (!(verts[i].y > -range && verts[i].y < range)) return false;
	if (!(verts[i].z > -range && verts[i].z < range)) return false;
      }
    }
    return true;
  }
}
