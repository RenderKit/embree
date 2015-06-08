// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "../algorithms/sort.h"
#include "../algorithms/prefix.h"
#include "../algorithms/parallel_for.h"

#if !defined(__MIC__)
#include "subdiv/feature_adaptive_eval.h"
#include "subdiv/patch.h"
#endif

namespace embree
{
  SubdivMesh::SubdivMesh (Scene* parent, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, 
			  size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps)
    : Geometry(parent,SUBDIV_MESH,numFaces,numTimeSteps,flags), 
      numFaces(numFaces), 
      numEdges(numEdges), 
      numHalfEdges(0),
      numVertices(numVertices),
      displFunc(nullptr), 
      displBounds(empty),
      levelUpdate(false)
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
    enabling();
  }

  void SubdivMesh::enabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->numSubdivPatches ,numFaces); 
    else                   atomic_add(&parent->numSubdivPatches2,numFaces); 
  }
  
  void SubdivMesh::disabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->numSubdivPatches ,-(ssize_t)numFaces); 
    else                   atomic_add(&parent->numSubdivPatches2,-(ssize_t)numFaces);
  }

  void SubdivMesh::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
  }

  void SubdivMesh::setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride) 
  { 
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3)) 
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

    /* verify that all vertex accesses are 16 bytes aligned */
#if defined(__MIC__)
    if (type == RTC_VERTEX_BUFFER0 || type == RTC_VERTEX_BUFFER1) {
      if (((size_t(ptr) + offset) & 0xF) || (stride & 0xF))
        throw_RTCError(RTC_INVALID_OPERATION,"data must be 16 bytes aligned");
    }
#endif

    switch (type) {
    case RTC_INDEX_BUFFER               : vertexIndices.set(ptr,offset,stride); break;
    case RTC_FACE_BUFFER                : faceVertices.set(ptr,offset,stride); break;
    case RTC_HOLE_BUFFER                : holes.set(ptr,offset,stride); break;
    case RTC_EDGE_CREASE_INDEX_BUFFER   : edge_creases.set(ptr,offset,stride); break;
    case RTC_EDGE_CREASE_WEIGHT_BUFFER  : edge_crease_weights.set(ptr,offset,stride); break;
    case RTC_VERTEX_CREASE_INDEX_BUFFER : vertex_creases.set(ptr,offset,stride); break;
    case RTC_VERTEX_CREASE_WEIGHT_BUFFER: vertex_crease_weights.set(ptr,offset,stride); break;
    case RTC_LEVEL_BUFFER               : levels.set(ptr,offset,stride); break;

    case RTC_VERTEX_BUFFER0: 
      vertices[0].set(ptr,offset,stride); 
      if (numVertices) {
        /* test if array is properly padded */
        volatile int w = *((int*)vertices[0].getPtr(numVertices-1)+3); // FIXME: is failing hard avoidable?
      }
      break;

    case RTC_VERTEX_BUFFER1: 
      vertices[1].set(ptr,offset,stride); 
      if (numVertices) {
        /* test if array is properly padded */
        volatile int w = *((int*)vertices[1].getPtr(numVertices-1)+3); // FIXME: is failing hard avoidable?
      }
      break;

    case RTC_USER_VERTEX_BUFFER0: if (userbuffers[0] == nullptr) userbuffers[0].reset(new Buffer(numVertices,stride)); userbuffers[0]->set(ptr,offset,stride);  break;
    case RTC_USER_VERTEX_BUFFER1: if (userbuffers[1] == nullptr) userbuffers[1].reset(new Buffer(numVertices,stride)); userbuffers[1]->set(ptr,offset,stride);  break;

    default: 
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type");
    }
  }

  void* SubdivMesh::map(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    switch (type) {
    case RTC_INDEX_BUFFER                : return vertexIndices.map(parent->numMappedBuffers);
    case RTC_FACE_BUFFER                 : return faceVertices.map(parent->numMappedBuffers);
    case RTC_HOLE_BUFFER                 : return holes.map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER0              : return vertices[0].map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER1              : return vertices[1].map(parent->numMappedBuffers);
    case RTC_EDGE_CREASE_INDEX_BUFFER    : return edge_creases.map(parent->numMappedBuffers); 
    case RTC_EDGE_CREASE_WEIGHT_BUFFER   : return edge_crease_weights.map(parent->numMappedBuffers); 
    case RTC_VERTEX_CREASE_INDEX_BUFFER  : return vertex_creases.map(parent->numMappedBuffers); 
    case RTC_VERTEX_CREASE_WEIGHT_BUFFER : return vertex_crease_weights.map(parent->numMappedBuffers); 
    case RTC_LEVEL_BUFFER                : return levels.map(parent->numMappedBuffers); 
    default                              : throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); return nullptr;
    }
  }

  void SubdivMesh::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    switch (type) {
    case RTC_INDEX_BUFFER               : vertexIndices.unmap(parent->numMappedBuffers); break;
    case RTC_FACE_BUFFER                : faceVertices.unmap(parent->numMappedBuffers); break;
    case RTC_HOLE_BUFFER                : holes.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER0             : vertices[0].unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER1             : vertices[1].unmap(parent->numMappedBuffers); break;
    case RTC_EDGE_CREASE_INDEX_BUFFER   : edge_creases.unmap(parent->numMappedBuffers); break;
    case RTC_EDGE_CREASE_WEIGHT_BUFFER  : edge_crease_weights.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_CREASE_INDEX_BUFFER : vertex_creases.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_CREASE_WEIGHT_BUFFER: vertex_crease_weights.unmap(parent->numMappedBuffers); break;
    case RTC_LEVEL_BUFFER               : levels.unmap(parent->numMappedBuffers); break;
    default                             : throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
    }
  }

  void SubdivMesh::update ()
  {
    faceVertices.setModified(true);
    vertexIndices.setModified(true); 
    holes.setModified(true);
    vertices[0].setModified(true); 
    vertices[1].setModified(true); 
    edge_creases.setModified(true);
    edge_crease_weights.setModified(true);
    vertex_creases.setModified(true);
    vertex_crease_weights.setModified(true); 
    levels.setModified(true);
    Geometry::update();
  }

  void SubdivMesh::updateBuffer (RTCBufferType type)
  {
    switch (type) {
    case RTC_INDEX_BUFFER               : vertexIndices.setModified(true); break;
    case RTC_FACE_BUFFER                : faceVertices.setModified(true); break;
    case RTC_HOLE_BUFFER                : holes.setModified(true); break;
    case RTC_VERTEX_BUFFER0             : vertices[0].setModified(true); break;
    case RTC_VERTEX_BUFFER1             : vertices[1].setModified(true); break;
    case RTC_EDGE_CREASE_INDEX_BUFFER   : edge_creases.setModified(true); break;
    case RTC_EDGE_CREASE_WEIGHT_BUFFER  : edge_crease_weights.setModified(true); break;
    case RTC_VERTEX_CREASE_INDEX_BUFFER : vertex_creases.setModified(true); break;
    case RTC_VERTEX_CREASE_WEIGHT_BUFFER: vertex_crease_weights.setModified(true); break;
    case RTC_LEVEL_BUFFER               : levels.setModified(true); break;
    default                             : throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
    }
    Geometry::update();
  }

  void SubdivMesh::setDisplacementFunction (RTCDisplacementFunc func, RTCBounds* bounds) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->displFunc   = func;
    if (bounds) this->displBounds = *(BBox3fa*)bounds; 
    else        this->displBounds = empty;
  }

  void SubdivMesh::immutable () 
  {
    const bool freeIndices = !parent->needSubdivIndices;
    const bool freeVertices = !parent->needSubdivVertices;
    faceVertices.free();
    if (freeIndices) vertexIndices.free();
    if (freeVertices) vertices[0].free();
    if (freeVertices) vertices[1].free();
    edge_creases.free();
    edge_crease_weights.free();
    vertex_creases.free();
    vertex_crease_weights.free();
    levels.free();
    holes.free();
  }

  __forceinline uint64_t pair64(unsigned int x, unsigned int y) 
  {
    if (x<y) std::swap(x,y);
    return (((uint64_t)x) << 32) | (uint64_t)y;
  }

  void SubdivMesh::calculateHalfEdges()
  {
#if defined(__MIC__)
    size_t blockSize = 1;
#else
    size_t blockSize = 4096;
#endif

    /* allocate temporary array */
    halfEdges0.resize(numEdges);
    halfEdges1.resize(numEdges);

    /* create all half edges */
    parallel_for( size_t(0), numFaces, blockSize, [&](const range<size_t>& r) 
    {
      for (size_t f=r.begin(); f<r.end(); f++) 
      {
	// FIXME: use 'stride' to reduce imuls on MIC !!!

#if defined(__MIC__)
	prefetch<PFHINT_L2>((char*)&faceVertices[f]  + 4*64);
	prefetch<PFHINT_L2>((char*)&faceStartEdge[f] + 4*64);
#endif

	const size_t N = faceVertices[f];
	const size_t e = faceStartEdge[f];

	for (size_t de=0; de<N; de++)
	{
	  HalfEdge* edge = &halfEdges[e+de];

#if defined(__MIC__)
	  prefetch<PFHINT_L1>(edge + 2);
	  prefetch<PFHINT_L1EX>(&halfEdges1[e+de+2]);
#endif

	  const unsigned int startVertex = vertexIndices[e+de];
	  unsigned int nextIndex = de + 1;
	  if (unlikely(nextIndex >= N)) nextIndex -= N; 
	  const unsigned int endVertex = vertexIndices[e + nextIndex]; 
	  const uint64_t key = SubdivMesh::Edge(startVertex,endVertex);
	  
	  float edge_level = 1.0f;
	  if (levels) edge_level = levels[e+de];
	  edge_level = clamp(edge_level,1.0f,4096.0f); // FIXME: do we want to limit edge level?
	  
	  edge->vtx_index              = startVertex;
	  edge->next_half_edge_ofs     = (de == (N-1)) ? -(N-1) : +1;
	  edge->prev_half_edge_ofs     = (de ==     0) ? +(N-1) : -1;
	  edge->opposite_half_edge_ofs = 0;
	  edge->edge_crease_weight     = edgeCreaseMap.lookup(key,0.0f);
	  edge->vertex_crease_weight   = vertexCreaseMap.lookup(startVertex,0.0f);
	  edge->edge_level             = edge_level;
          edge->type                   = COMPLEX_PATCH; // type gets updated below

	  if (unlikely(holeSet.lookup(f))) 
	    halfEdges1[e+de] = SubdivMesh::KeyHalfEdge(-1,edge);
	  else
	    halfEdges1[e+de] = SubdivMesh::KeyHalfEdge(key,edge);
	}
      }
    });

    /* sort half edges to find adjacent edges */
    radix_sort_u64(halfEdges1.data(),halfEdges0.data(),numHalfEdges);

    /* link all adjacent pairs of edges */
    parallel_for( size_t(0), numHalfEdges, blockSize, [&](const range<size_t>& r) 
    {
      /* skip if start of adjacent edges was not in our range */
      size_t e=r.begin();
      if (e != 0 && (halfEdges1[e].key == halfEdges1[e-1].key)) {
	const uint64_t key = halfEdges1[e].key;
	while (e<r.end() && halfEdges1[e].key == key) e++;
      }

      /* process all adjacent edges starting in our range */
      while (e<r.end())
      {
	const uint64_t key = halfEdges1[e].key;
	if (key == -1) break;
	int N=1; while (e+N<numHalfEdges && halfEdges1[e+N].key == key) N++;

        /* border edges are identified by not having an opposite edge set */
	if (N == 1) {
	}

        /* standard edge shared between two faces */
	else if (N == 2) {
	  halfEdges1[e+0].edge->setOpposite(halfEdges1[e+1].edge);
	  halfEdges1[e+1].edge->setOpposite(halfEdges1[e+0].edge);
	}

        /* non-manifold geometry is handled by keeping vertices fixed during subdivision */
        else {
	  for (size_t i=0; i<N; i++) {
	    halfEdges1[e+i].edge->vertex_crease_weight = inf;
	    halfEdges1[e+i].edge->next()->vertex_crease_weight = inf;
	  }
	}
	e+=N;
      }
    });

    /* calculate type of each patch */
    parallel_for( size_t(0), numFaces, blockSize, [&](const range<size_t>& r) 
    {
      for (size_t f=r.begin(); f<r.end(); f++) 
      {
        HalfEdge* edge = &halfEdges[faceStartEdge[f]];
        PatchType type = edge->patchType();
        for (size_t i=0; i<faceVertices[f]; i++)
          edge[i].type = type;
      }
    });
  }

  void SubdivMesh::updateHalfEdges()
  {
    /* assume we do no longer recalculate in the future and clear these arrays */
    halfEdges0.clear();
    halfEdges1.clear();

    /* calculate which data to update */
    const bool updateEdgeCreases = edge_creases.isModified() || edge_crease_weights.isModified();
    const bool updateVertexCreases = vertex_creases.isModified() || vertex_crease_weights.isModified(); 
    const bool updateLevels = levels.isModified();

    /* parallel loop over all half edges */
    parallel_for( size_t(0), numHalfEdges, size_t(4096), [&](const range<size_t>& r) 
    {
      for (size_t i=r.begin(); i!=r.end(); i++)
      {
	HalfEdge& edge = halfEdges[i];
	const unsigned int startVertex = edge.vtx_index;
 
	if (updateLevels) {
	  float edge_level = 1.0f;
	  if (levels) edge_level = levels[i];
	  edge.edge_level = clamp(edge_level,1.0f,4096.0f); // FIXME: do we want to limit edge levels?
	}
        
	if (updateEdgeCreases) {
	  const unsigned int endVertex   = edge.next()->vtx_index;
	  const uint64_t key = SubdivMesh::Edge(startVertex,endVertex);
	  edge.edge_crease_weight = edgeCreaseMap.lookup(key,0.0f);
	}

	if (updateVertexCreases)
	  edge.vertex_crease_weight = vertexCreaseMap.lookup(startVertex,0.0f);

        if (updateEdgeCreases || updateVertexCreases) {
          edge.type = edge.patchType();
        }
      }
    });
  }

  void SubdivMesh::initializeHalfEdgeStructures ()
  {
    double t0 = getSeconds();

    /* allocate half edge array */
    halfEdges.resize(numEdges);
    
    /* calculate start edge of each face */
    faceStartEdge.resize(numFaces);
    if (faceVertices.isModified()) 
      numHalfEdges = parallel_prefix_sum(faceVertices,faceStartEdge,numFaces);

    /* create set with all holes */
    if (holes.isModified())
      holeSet.init(holes);

    /* create set with all vertex creases */
    if (vertex_creases.isModified() || vertex_crease_weights.isModified())
      vertexCreaseMap.init(vertex_creases,vertex_crease_weights);
    
    /* create map with all edge creases */
    if (edge_creases.isModified() || edge_crease_weights.isModified())
      edgeCreaseMap.init(edge_creases,edge_crease_weights);

    /* check if we have to recalculate the half edges */
    bool recalculate = false;
    recalculate |= vertexIndices.isModified(); 
    recalculate |= faceVertices.isModified();
    recalculate |= holes.isModified();

    /* check if we can simply update the half edges */
    bool update = false;
    update |= edge_creases.isModified();
    update |= edge_crease_weights.isModified();
    update |= vertex_creases.isModified();
    update |= vertex_crease_weights.isModified(); 
    update |= levels.isModified();
    
    /* check whether we can simply update the bvh in cached mode */
    levelUpdate = !recalculate && edge_creases.size() == 0 && vertex_creases.size() == 0 && levels.isModified();

    /* now either recalculate or update the half edges */
    if (recalculate) calculateHalfEdges();
    else if (update) updateHalfEdges();

    /* create interpolation cache mapping for interpolation scenes */
    if (parent->isInterpolatable()) 
    {
      for (size_t i=0; i<2; i++) {
        if (vertices[i]) vertex_buffer_tags[i].resize(numFaces);
        if (userbuffers[i]) user_buffer_tags[i].resize(numFaces);
      }
    }

    /* cleanup some state for static scenes */
    if (parent->isStatic()) 
    {
      holeSet.cleanup();
      halfEdges0.clear();
      halfEdges1.clear();
      vertexCreaseMap.clear();
      edgeCreaseMap.clear();
    }

    /* clear modified state of all buffers */
    vertexIndices.setModified(false); 
    faceVertices.setModified(false);
    holes.setModified(false);
    vertices[0].setModified(false); 
    vertices[1].setModified(false); 
    edge_creases.setModified(false);
    edge_crease_weights.setModified(false);
    vertex_creases.setModified(false);
    vertex_crease_weights.setModified(false); 
    levels.setModified(false);

    double t1 = getSeconds();

    /* print statistics in verbose mode */
    if (State::instance()->verbosity(2)) 
    {
      size_t numRegularQuadFaces = 0;
      size_t numIrregularQuadFaces = 0;
      size_t numComplexFaces = 0;

      for (size_t e=0, f=0; f<numFaces; e+=faceVertices[f++]) 
      {
        switch (halfEdges[e].type) {
        case REGULAR_QUAD_PATCH  : numRegularQuadFaces++;   break;
        case IRREGULAR_QUAD_PATCH: numIrregularQuadFaces++; break;
        case COMPLEX_PATCH       : numComplexFaces++;   break;
        }
      }
    
      std::cout << "half edge generation = " << 1000.0*(t1-t0) << "ms, " << 1E-6*double(numHalfEdges)/(t1-t0) << "M/s" << std::endl;
      std::cout << "numFaces = " << numFaces << ", " 
                << "numRegularQuadFaces = " << numRegularQuadFaces << " (" << 100.0f * numRegularQuadFaces / numFaces << "%), " 
                << "numIrregularQuadFaces " << numIrregularQuadFaces << " (" << 100.0f * numIrregularQuadFaces / numFaces << "%) " 
                << "numComplexFaces " << numComplexFaces << " (" << 100.0f * numComplexFaces / numFaces << "%) " 
                << std::endl;
    }
  }

  bool SubdivMesh::verify () 
  {
    if (numTimeSteps == 2 && vertices[0].size() != vertices[1].size())
      return false;
    
    /*! verify vertex indices */
    size_t ofs = 0;
    for (size_t i=0; i<faceVertices.size(); i++) 
    {
      int valence = faceVertices[i];
      for (size_t j=ofs; j<ofs+valence; j++) 
      {
        if (j >= vertexIndices.size())
          return false;

        if (vertexIndices[j] >= numVertices)
          return false; 
      }
      ofs += valence;
    }

    /*! verify vertices */
    for (size_t j=0; j<numTimeSteps; j++) {
      BufferT<Vec3fa>& verts = vertices[j];
      for (size_t i=0; i<numVertices; i++) {
        if (!isvalid(verts[i])) return false;
      }
    }
    return true;
  }

  void SubdivMesh::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats) 
  {
#if defined(DEBUG) // FIXME: use function pointers and also throw error in release mode
    if ((parent->aflags & RTC_INTERPOLATE) == 0) 
      throw_RTCError(RTC_INVALID_OPERATION,"rtcInterpolate can only get called when RTC_INTERPOLATE is enabled for the scene");
#endif

#if !defined(__MIC__) // FIXME: not working on MIC yet

    /* calculate base pointer and stride */
    assert((buffer >= RTC_VERTEX_BUFFER0 && buffer <= RTC_VERTEX_BUFFER1) ||
           (buffer >= RTC_USER_VERTEX_BUFFER0 && buffer <= RTC_USER_VERTEX_BUFFER1));
    const char* src = nullptr; 
    size_t stride = 0;
    size_t bufID = buffer&0xFFFF;
    CacheEntry* entry;
    if (buffer >= RTC_USER_VERTEX_BUFFER0) {
      src    = userbuffers[bufID]->getPtr();
      stride = userbuffers[bufID]->getStride();
      entry = &user_buffer_tags[bufID][primID];
    } else {
      src    = vertices[bufID].getPtr();
      stride = vertices[bufID].getStride();
      entry = &vertex_buffer_tags[bufID][primID];
    }

    const size_t threadIndex = SharedLazyTessellationCache::threadIndex();

    for (size_t i=0; i<numFloats; i+=4) // FIXME: implement AVX path
    {
      size_t ofs = i*sizeof(float);
      float4 Pt, dPdut, dPdvt; 
#if 0
      Patch<float4> patch(getHalfEdge(primID),src+i*sizeof(float),stride);
      patch.eval(u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
#endif

#if 1
      Patch<float4>* patch = nullptr;
      
      while(1)
      {
        SharedLazyTessellationCache::sharedLazyTessellationCache.lockThreadLoop(threadIndex);
       
        patch = (Patch<float4>*) SharedLazyTessellationCache::lookup(&entry->tag);

        if (patch) break;

        if (entry->mutex.try_write_lock())
        {
          void* ptr = SharedLazyTessellationCache::sharedLazyTessellationCache.allocLoop(threadIndex,sizeof(Patch<float4>));
          patch = new (ptr) Patch<float4>(getHalfEdge(primID),src+i*sizeof(float),stride);

          __memory_barrier();
          entry->tag = SharedLazyTessellationCache::Tag(ptr);
          entry->mutex.write_unlock();
          break;
        }
        else
        {
          SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadIndex);
          continue;
        }
      }

      patch->eval(u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
      
      SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadIndex);		  
#endif

#if 0
      auto load = [&](const SubdivMesh::HalfEdge* p) -> float4 { 
        const unsigned vtx = p->getStartVertexIndex();
        return float4::loadu((float*)&src[vtx*stride+ofs]);  // FIXME: reads behind the end of the array
      };
      feature_adaptive_point_eval<float4>(getHalfEdge(primID),load,u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
#endif
      if (P   ) for (size_t j=i; j<min(i+4,numFloats); j++) P[j] = Pt[j-i];
      if (dPdu) for (size_t j=i; j<min(i+4,numFloats); j++) dPdu[j] = dPdut[j-i];
      if (dPdv) for (size_t j=i; j<min(i+4,numFloats); j++) dPdv[j] = dPdvt[j-i];
    }
#endif
  }
}
