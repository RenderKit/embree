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
  SubdivMesh::SubdivMesh (Scene* parent, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps)
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
    edge_creases.init(numEdgeCreases,2*sizeof(unsigned int));
    edge_crease_weights.init(numEdgeCreases,sizeof(float));
    vertex_creases.init(numVertexCreases,sizeof(unsigned int));
    vertex_crease_weights.init(numVertexCreases,sizeof(float));
    levels.init(numEdges,sizeof(float));
  }

  SubdivMesh::~SubdivMesh () {
    if (halfEdges)
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
    case RTC_INDEX_BUFFER               : vertexIndices.set(ptr,offset,stride); break;
    case RTC_FACE_BUFFER                : faceVertices.set(ptr,offset,stride); break;
    case RTC_HOLE_BUFFER                : holes.set(ptr,offset,stride); break;
    case RTC_EDGE_CREASE_BUFFER         : edge_creases.set(ptr,offset,stride); break;
    case RTC_EDGE_CREASE_WEIGHT_BUFFER  : edge_crease_weights.set(ptr,offset,stride); break;
    case RTC_VERTEX_CREASE_BUFFER       : vertex_creases.set(ptr,offset,stride); break;
    case RTC_VERTEX_CREASE_WEIGHT_BUFFER: vertex_crease_weights.set(ptr,offset,stride); break;
    case RTC_LEVEL_BUFFER               : levels.set(ptr,offset,stride); break;

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
    case RTC_INDEX_BUFFER                : return vertexIndices.map(parent->numMappedBuffers);
    case RTC_FACE_BUFFER                 : return faceVertices.map(parent->numMappedBuffers);
    case RTC_HOLE_BUFFER                 : return holes.map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER0              : return vertices[0].map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER1              : return vertices[1].map(parent->numMappedBuffers);
    case RTC_EDGE_CREASE_BUFFER          : return edge_creases.map(parent->numMappedBuffers); 
    case RTC_EDGE_CREASE_WEIGHT_BUFFER   : return edge_crease_weights.map(parent->numMappedBuffers); 
    case RTC_VERTEX_CREASE_BUFFER        : return vertex_creases.map(parent->numMappedBuffers); 
    case RTC_VERTEX_CREASE_WEIGHT_BUFFER : return vertex_crease_weights.map(parent->numMappedBuffers); 
    case RTC_LEVEL_BUFFER                : return levels.map(parent->numMappedBuffers); 
    default                              : process_error(RTC_INVALID_ARGUMENT,"unknown buffer type"); return NULL;
    }
  }

  void SubdivMesh::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      process_error(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return;
    }

    switch (type) {
    case RTC_INDEX_BUFFER               : vertexIndices.unmap(parent->numMappedBuffers); break;
    case RTC_FACE_BUFFER                : faceVertices.unmap(parent->numMappedBuffers); break;
    case RTC_HOLE_BUFFER                : holes.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER0             : vertices[0].unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER1             : vertices[1].unmap(parent->numMappedBuffers); break;
    case RTC_EDGE_CREASE_BUFFER         : edge_creases.unmap(parent->numMappedBuffers); break;
    case RTC_EDGE_CREASE_WEIGHT_BUFFER  : edge_crease_weights.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_CREASE_BUFFER       : vertex_creases.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_CREASE_WEIGHT_BUFFER: vertex_crease_weights.unmap(parent->numMappedBuffers); break;
    case RTC_LEVEL_BUFFER               : levels.unmap(parent->numMappedBuffers); break;
    default                             : process_error(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
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
    /* allocate half edge array */
    delete halfEdges;
    halfEdges = new HalfEdge[numEdges];

    /* calculate start edge of each face */
    faceStartEdge.resize(numFaces);
    for (size_t f=0, ofs=0; f<numFaces; ofs+=faceVertices[f++])
      faceStartEdge[f] = &halfEdges[ofs];

    /* create map containing all edge_creases */
    std::map<size_t,float> creaseMap;
    for (size_t i=0; i<edge_creases.size(); i++)
      creaseMap[pair64(edge_creases[i].x,edge_creases[i].y)] = edge_crease_weights[i];

    /* calculate vertex_crease weight for each vertex */
    std::vector<float> full_vertex_crease_weights(numVertices);
    for (size_t i=0; i<numVertices; i++) 
      full_vertex_crease_weights[i] = 0.0f;
    for (size_t i=0; i<vertex_creases.size(); i++) 
      full_vertex_crease_weights[vertex_creases[i]] = vertex_crease_weights[i];

    /* calculate full hole vector */
    full_holes.resize(numFaces);
    for (size_t i=0; i<full_holes.size(); i++) full_holes[i] = 0;
    for (size_t i=0; i<holes.size()     ; i++) full_holes[holes[i]] = 1;
    
    /* initialize all half-edges for each face */
    std::map<size_t,HalfEdge*> edgeMap;
    std::map<size_t,bool> nonManifoldEdges;

    for (size_t i=0, j=0; i<numFaces; i++) 
    {
      const ssize_t N = faceVertices[i];
      
      for (size_t dj=0; dj<N; dj++)
      {
        HalfEdge* edge0 = &halfEdges[j+dj];
        const unsigned int startVertex = vertexIndices[j+dj];
        const unsigned int endVertex   = vertexIndices[j + (dj+1) % N];
        const int64 value = pair64(startVertex,endVertex);

        float edge_crease_weight = 0.0f;
        if (creaseMap.find(value) != creaseMap.end()) 
          edge_crease_weight = creaseMap[value];

        float edge_level = 1.0f;
        if (levels) edge_level = levels[j+dj];
        
        edge0->vtx_index = startVertex;
        edge0->next_half_edge_ofs = (dj == (N-1)) ? -(N-1) : +1;
        edge0->prev_half_edge_ofs = (dj ==     0) ? +(N-1) : -1;
        edge0->opposite_half_edge_ofs = 0;
        edge0->edge_crease_weight = edge_crease_weight;
        edge0->vertex_crease_weight = full_vertex_crease_weights[startVertex];
        edge0->edge_level = edge_level;
        if (full_holes[i]) continue;
        
        std::map<size_t,HalfEdge*>::iterator found = edgeMap.find(value);
        if (found == edgeMap.end()) {
          edgeMap[value] = edge0;
          continue;
        }

        HalfEdge* edge1 = found->second;
        if (unlikely(edge1->hasOpposite())) 
          nonManifoldEdges[value] = true;

        edge0->opposite_half_edge_ofs = edge1 - edge0;
        edge1->opposite_half_edge_ofs = edge0 - edge1;
      }
      j+=N;
    }

    for (size_t i=0, j=0; i<numFaces; i++) 
    {
      ssize_t N = faceVertices[i];
      for (size_t dj=0; dj<N; dj++)
      {
        HalfEdge* edge = &halfEdges[j+dj];
        const unsigned int startVertex = vertexIndices[j+dj];
        const unsigned int endVertex   = vertexIndices[j + (dj + 1) % N];
        const int64 value = pair64(startVertex,endVertex);

        if (nonManifoldEdges.find(value) != nonManifoldEdges.end()) {
          edge->opposite_half_edge_ofs = 0;
          edge->vertex_crease_weight = inf;
          edge->next()->vertex_crease_weight = inf;
          continue;
        }
      }
      j+=N;
    }

    /* print statistics in verbose mode */
    if (g_verbose >= 1) 
    {
      size_t numRegularPatches = 0;
      size_t numIrregularPatches = 0;
      size_t numPatchesWithEdges = 0;

      assert(numEdges % 4 == 0);
      for (size_t i=0; i<numEdges; i+=4)
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
