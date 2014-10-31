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
#include "scene_subdivision.h"

namespace embree
{
  SubdivMesh::SubdivMesh (Scene* parent, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, size_t numCreases, size_t numCorners, size_t numHoles, size_t numTimeSteps)
    : Geometry(parent,SUBDIV_MESH,numFaces,flags), 
      mask(-1), 
      numTimeSteps(numTimeSteps),
      numFaces(numFaces), 
      numEdges(numEdges), 
      numVertices(numVertices),
      displFunc(NULL), displBounds(empty),
      halfEdges(NULL)
  {
    for (size_t i=0; i<numTimeSteps; i++)
       vertices[i].init(numVertices,sizeof(Vec3fa));

    vertexIndices.init(numEdges,sizeof(unsigned int));
    faceVertices.init(numFaces,sizeof(unsigned int));
    holes.init(numHoles,sizeof(int));
    creases.init(numCreases,2*sizeof(unsigned int));
    crease_weights.init(numCreases,sizeof(float));
    corners.init(numCorners,sizeof(unsigned int));
    corner_weights.init(numCorners,sizeof(float));
    levels.init(numEdges,sizeof(float));
  }

  SubdivMesh::~SubdivMesh () {
    delete[] halfEdges;
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
    case RTC_INDEX_BUFFER          : vertexIndices.set(ptr,offset,stride); break;
    case RTC_FACE_BUFFER           : faceVertices.set(ptr,offset,stride); break;
    case RTC_HOLE_BUFFER           : holes.set(ptr,offset,stride); break;
    case RTC_CREASE_BUFFER         : creases.set(ptr,offset,stride); break;
    case RTC_CREASE_WEIGHT_BUFFER  : crease_weights.set(ptr,offset,stride); break;
    case RTC_CORNER_BUFFER         : corners.set(ptr,offset,stride); break;
    case RTC_CORNER_WEIGHT_BUFFER  : corner_weights.set(ptr,offset,stride); break;
    case RTC_LEVEL_BUFFER          : levels.set(ptr,offset,stride); break;

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
    case RTC_INDEX_BUFFER          : return vertexIndices.map(parent->numMappedBuffers);
    case RTC_FACE_BUFFER           : return faceVertices.map(parent->numMappedBuffers);
    case RTC_HOLE_BUFFER           : return holes.map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER0        : return vertices[0].map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER1        : return vertices[1].map(parent->numMappedBuffers);
    case RTC_CREASE_BUFFER         : return creases.map(parent->numMappedBuffers); 
    case RTC_CREASE_WEIGHT_BUFFER  : return crease_weights.map(parent->numMappedBuffers); 
    case RTC_CORNER_BUFFER         : return corners.map(parent->numMappedBuffers); 
    case RTC_CORNER_WEIGHT_BUFFER  : return corner_weights.map(parent->numMappedBuffers); 
    case RTC_LEVEL_BUFFER          : return levels.map(parent->numMappedBuffers); 
    default                        : process_error(RTC_INVALID_ARGUMENT,"unknown buffer type"); return NULL;
    }
  }

  void SubdivMesh::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      process_error(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return;
    }

    switch (type) {
    case RTC_INDEX_BUFFER          : vertexIndices.unmap(parent->numMappedBuffers); break;
    case RTC_FACE_BUFFER           : faceVertices.unmap(parent->numMappedBuffers); break;
    case RTC_HOLE_BUFFER           : holes.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER0        : vertices[0].unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER1        : vertices[1].unmap(parent->numMappedBuffers); break;
    case RTC_CREASE_BUFFER         : creases.unmap(parent->numMappedBuffers); break;
    case RTC_CREASE_WEIGHT_BUFFER  : crease_weights.unmap(parent->numMappedBuffers); break;
    case RTC_CORNER_BUFFER         : corners.unmap(parent->numMappedBuffers); break;
    case RTC_CORNER_WEIGHT_BUFFER  : corner_weights.unmap(parent->numMappedBuffers); break;
    case RTC_LEVEL_BUFFER          : levels.unmap(parent->numMappedBuffers); break;
    default                        : process_error(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
    }
  }

  void SubdivMesh::setUserData (void* ptr, bool ispc) {
    userPtr = ptr;
  }

  void SubdivMesh::setDisplacementFunction (RTCDisplacementFunc func, const RTCBounds& bounds) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      process_error(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return;
    }
    this->displFunc   = func;
    this->displBounds = (BBox3fa&)bounds; 
  }

  void SubdivMesh::immutable () 
  {
    bool freeVertices  = !parent->needVertices;
    if (freeVertices ) vertices[0].free();
    if (freeVertices ) vertices[1].free();
  }

  __forceinline int64 pair64(unsigned x, unsigned y) {
    if (x<y) std::swap(x,y);
    return (((int64)x) << 32) | (int64)y;
  }

  void SubdivMesh::initializeHalfEdgeStructures ()
  {
    numHalfEdges = 4*numFaces;
    halfEdges = new HalfEdge[numHalfEdges];

    /* calculate offset buffer */
    vertexOffsets.resize(numFaces);
    size_t ofs = 0;
    for (size_t i=0; i<numFaces; i++) {
      vertexOffsets[i] = ofs; ofs += faceVertices[i];
    }

    /* create map containing all creases */
    std::map<size_t,float> creaseMap;
    for (size_t i=0; i<creases.size(); i++)
      creaseMap[pair64(creases[i].x,creases[i].y)] = crease_weights[i];

    /* calculate corner weight for each vertex */
    full_corner_weights.resize(numVertices);
    for (size_t i=0; i<numVertices; i++)
      full_corner_weights[i] = 0.0f;
    for (size_t i=0; i<corners.size(); i++) {
      full_corner_weights[corners[i]] = corner_weights[i];
    }

    /* calculate full hole vector */
    full_holes.resize(numFaces);
    for (size_t i=0; i<full_holes.size(); i++) full_holes[i] = 0;
    for (size_t i=0; i<holes.size()     ; i++) full_holes[holes[i]] = 1;
    
    /* initialize all half-edges for each face */
    std::map<size_t,unsigned int> edgeMap;

    for (size_t i=0; i<numFaces; i++) 
    {
      const unsigned int halfEdgeIndex = vertexOffsets[i];
      for (size_t j=0; j<4; j++)
      {
        halfEdges[4*i+j].vtx_index      = vertexIndices[halfEdgeIndex + j];
        halfEdges[4*i+j].halfedge_id    = 4*i+j;
        halfEdges[4*i+j].opposite_index = (unsigned int)-1;
        halfEdges[4*i+j].crease_weight  = 0.0f;
        if (levels)  halfEdges[4*i+j].level = levels[4*i+j];
        else         halfEdges[4*i+j].level = 3.0f;

        if (full_holes[i]) continue;
        
        const unsigned int start = vertexIndices[halfEdgeIndex + j + 0];
        const unsigned int end   = vertexIndices[halfEdgeIndex + (j + 1) % 4];
        const int64 value = pair64(start,end);

        if (creaseMap.find(value) != creaseMap.end()) 
          halfEdges[4*i+j].crease_weight = creaseMap[value];

        std::map<size_t,unsigned int>::iterator found = edgeMap.find(value);
        if (found == edgeMap.end()) {
          edgeMap[value] = 4*i+j;
          continue;
        }

        halfEdges[4*i+j].opposite_index = found->second;
        halfEdges[ found->second ].opposite_index = 4*i+j;
      }
    }

    /* print statistics in verbose mode */
    if (g_verbose >= 1) 
    {
      size_t numRegularPatches = 0;
      size_t numIrregularPatches = 0;
      size_t numPatchesWithEdges = 0;

      assert(numHalfEdges % 4 == 0);
      for (size_t i=0; i<numHalfEdges; i+=4)
      {
        if (halfEdges[i].faceHasEdges())
        {
          numIrregularPatches++;
          numPatchesWithEdges++;
        }
        else if (halfEdges[i].isFaceRegular())
          numRegularPatches++;
        else
          numIrregularPatches++;
      }
    
      size_t numPatches = numRegularPatches + numIrregularPatches;

      std::cout << "numPatches " << numPatches 
                << " : regular " << numRegularPatches << " (" << 100.0f * numRegularPatches / numPatches << "%)" 
                << " irregular " << numIrregularPatches << " (" << 100.0f * numIrregularPatches / numPatches << "%) " 
                << " irregular with edges " << numPatchesWithEdges << " (" << 100.0f * numPatchesWithEdges / numPatches << "%) " << std::endl;
    }

    //IrregularCatmullClarkPatch patch0(&halfEdges[0],&vertices[0][0]); // FIXME: remove
    //PRINT(patch0);

    //IrregularCatmullClarkPatch patch0_1[4];
    //patch0.subdivide(patch0_1);
    //PRINT(patch0_1[3]);

    //IrregularCatmullClarkPatch patch8(&halfEdges[8],&vertices[0][0]); // FIXME: remove
    //PRINT(patch8);

    //CatmullClark1Ring ring; 
    //ring.init(&halfEdges[8],&vertices[0][0]);
    //PRINT(ring);
    //PRINT(halfEdges[8]->crease_weight);

    //IrregularCatmullClarkPatch patch8_1[4];
    //patch8.subdivide(patch8_1);
    //PRINT(patch8_1[0]);

    //exit(1); // FIXME: remove
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
