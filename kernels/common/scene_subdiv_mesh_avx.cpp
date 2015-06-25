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
#include "subdiv/patch_eval.h"
#include "subdiv/patch_eval_simd.h"

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
        float4 Pt, dPdut, dPdvt; 
        PatchEval<float4>::eval(baseEntry->at(interpolationSlot8(primID,slot,stride)),parent->commitCounter,
                                getHalfEdge(primID),src+i*sizeof(float),stride,u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);

        if (P   ) for (size_t j=i; j<min(i+4,numFloats); j++) P[j] = Pt[j-i];
        if (dPdu) for (size_t j=i; j<min(i+4,numFloats); j++) dPdu[j] = dPdut[j-i];
        if (dPdv) for (size_t j=i; j<min(i+4,numFloats); j++) dPdv[j] = dPdvt[j-i];
        i+=4;
      }
      else
      {
        float8 Pt, dPdut, dPdvt; 
        PatchEval<float8>::eval(baseEntry->at(interpolationSlot8(primID,slot,stride)),parent->commitCounter,
                                getHalfEdge(primID),src+i*sizeof(float),stride,u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
                                     
        if (P   ) for (size_t j=i; j<i+8; j++) P[j] = Pt[j-i];
        if (dPdu) for (size_t j=i; j<i+8; j++) dPdu[j] = dPdut[j-i];
        if (dPdv) for (size_t j=i; j<i+8; j++) dPdv[j] = dPdvt[j-i];
        i+=8;
      }
    }
    AVX_ZERO_UPPER();
  }

  void SubdivMeshAVX::interpolateN(const void* valid_i, const unsigned* primIDs, const float* u, const float* v, size_t numUVs, 
                                   RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats)
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
      src    = userbuffers[bufID]->getPtr();
      stride = userbuffers[bufID]->getStride();
      baseEntry = &user_buffer_tags[bufID];
    } else {
      src    = vertices[bufID].getPtr();
      stride = vertices[bufID].getStride();
      baseEntry = &vertex_buffer_tags[bufID];
    }

    const int* valid = (const int*) valid_i;
    
    for (size_t i=0; i<numUVs; i+=8) 
    {
      const size_t L = min(size_t(8),numUVs-i);
      bool8 valid1 = valid ? int8::loadu(&valid[i]) == int8(-1) : bool4(false);
      valid1 &= int8(i)+int8(step) < int8(numUVs);
      if (none(valid1)) continue;
      
      const int8 primID = int8::loadu(&primIDs[i]);
      const float8 uu = float8::loadu(&u[i]);
      const float8 vv = float8::loadu(&v[i]);

      foreach_unique(valid1,primID,[&](const bool8& valid1, const int primID) 
      {
        for (size_t j=0,slot=0; j<numFloats; slot++)
        {
          if (j+8 > numFloats)
          {
            const size_t M = min(size_t(4),numFloats-j);
            PatchEvalSimd<bool8,int8,float8,float4>::eval(baseEntry->at(interpolationSlot8(primID,slot,stride)),parent->commitCounter,
                                                          getHalfEdge(primID),src+j*sizeof(float),stride,valid1,uu,vv,
                                                          P ? P+j*numUVs+i : nullptr,dPdu ? dPdu+j*numUVs+i : nullptr,dPdv ? dPdv+j*numUVs+i : nullptr,numUVs,M);
            
            j+=4;
          }
          else
          {
            const size_t M = min(size_t(8),numFloats-j);
            PatchEvalSimd<bool8,int8,float8,float8>::eval(baseEntry->at(interpolationSlot8(primID,slot,stride)),parent->commitCounter,
                                                          getHalfEdge(primID),src+j*sizeof(float),stride,valid1,uu,vv,
                                                          P ? P+j*numUVs+i : nullptr,dPdu ? dPdu+j*numUVs+i : nullptr,dPdv ? dPdv+j*numUVs+i : nullptr,numUVs,M);
            
            j+=8;
          }
        }
      });
    }
    AVX_ZERO_UPPER();
  }
}
