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
#include "subdiv/patch.h"

namespace embree
{
  SubdivMeshAVX::SubdivMeshAVX(Scene* parent, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, 
                               size_t numCreases, size_t numCorners, size_t numHoles, size_t numTimeSteps)
                               : SubdivMesh(parent,flags,numFaces,numEdges,numVertices,numCreases,numCorners,numHoles,numTimeSteps) {}
    
  void SubdivMeshAVX::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats) 
  {
#if defined(DEBUG) // FIXME: use function pointers and also throw error in release mode
    if ((parent->aflags & RTC_INTERPOLATE) == 0) 
      throw_RTCError(RTC_INVALID_OPERATION,"rtcInterpolate can only get called when RTC_INTERPOLATE is enabled for the scene");
#endif

    /* calculate base pointer and stride */
    assert((buffer >= RTC_VERTEX_BUFFER0 && buffer <= RTC_VERTEX_BUFFER1) ||
           (buffer >= RTC_USER_VERTEX_BUFFER0 && buffer <= RTC_USER_VERTEX_BUFFER1));
    const char* src = nullptr; 
    size_t stride = 0;
    size_t bufID = buffer&0xFFFF;
    std::vector<SharedLazyTessellationCache::CacheEntry>* baseEntry = nullptr;
    if (buffer >= RTC_USER_VERTEX_BUFFER0) {
      src    = userbuffers[buffer&0xFFFF]->getPtr();
      stride = userbuffers[buffer&0xFFFF]->getStride();
      baseEntry = &user_buffer_tags[bufID];
    } else {
      src    = vertices[buffer&0xFFFF].getPtr();
      stride = vertices[buffer&0xFFFF].getStride();
      baseEntry = &vertex_buffer_tags[bufID];
    }

    auto alloc = [](size_t bytes) { return SharedLazyTessellationCache::malloc(bytes); };
    
    for (size_t i=0,slot=0; i<numFloats; slot++)
    {
      if (i+8 > numFloats)
      {
        SharedLazyTessellationCache::CacheEntry& entry = baseEntry->at(interpolationSlot8(primID,slot,stride));
        Patch<float4>* patch = SharedLazyTessellationCache::lookup(entry,parent->commitCounter,[&] () {
            return Patch<float4>::create(alloc,getHalfEdge(primID),src+i*sizeof(float),stride);
          });
        float4 Pt, dPdut, dPdvt; 
        patch->eval(u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
        SharedLazyTessellationCache::unlock();
        if (P   ) for (size_t j=i; j<min(i+4,numFloats); j++) P[j] = Pt[j-i];
        if (dPdu) for (size_t j=i; j<min(i+4,numFloats); j++) dPdu[j] = dPdut[j-i];
        if (dPdv) for (size_t j=i; j<min(i+4,numFloats); j++) dPdv[j] = dPdvt[j-i];
        i+=4;
      }
      else
      {
        SharedLazyTessellationCache::CacheEntry& entry = baseEntry->at(interpolationSlot8(primID,slot,stride));
        Patch<float8>* patch = SharedLazyTessellationCache::lookup(entry,parent->commitCounter,[&] () {
            return Patch<float8>::create(alloc,getHalfEdge(primID),src+i*sizeof(float),stride);
          });
        float8 Pt, dPdut, dPdvt; 
        patch->eval(u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
        SharedLazyTessellationCache::unlock();
        if (P   ) for (size_t j=i; j<i+8; j++) P[j] = Pt[j-i];
        if (dPdu) for (size_t j=i; j<i+8; j++) dPdu[j] = dPdut[j-i];
        if (dPdv) for (size_t j=i; j<i+8; j++) dPdv[j] = dPdvt[j-i];
        i+=8;
      }
    }
    AVX_ZERO_UPPER();
  }
}
