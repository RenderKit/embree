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

#if !defined(__MIC__)
#include "subdiv/feature_adaptive_eval.h"
#endif

namespace embree
{
  void SubdivMeshAVX::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats) 
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
    if (buffer >= RTC_USER_VERTEX_BUFFER0) {
      src    = userbuffers[buffer&0xFFFF]->getPtr();
      stride = userbuffers[buffer&0xFFFF]->getStride();
    } else {
      src    = vertices[buffer&0xFFFF].getPtr();
      stride = vertices[buffer&0xFFFF].getStride();
    }
    
    for (size_t i=0; i<numFloats; i+=8)
    {
      if (i+4 > numFloats)
      {
        const size_t n = numFloats-i;
        auto load = [&](const SubdivMesh::HalfEdge* p) { 
          const unsigned vtx = p->getStartVertexIndex();
          return float4::loadu((float*)&src[vtx*stride],n); 
        };
        float4 Pt, dPdut, dPdvt; 
        feature_adaptive_point_eval<float4>(getHalfEdge(primID),load,u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
        if (P   ) for (size_t j=i; j<numFloats; j++) P[j] = Pt[j-i];
        if (dPdu) for (size_t j=i; j<numFloats; j++) dPdu[j] = dPdut[j-i];
        if (dPdv) for (size_t j=i; j<numFloats; j++) dPdv[j] = dPdvt[j-i];
      }
      else if (i+8 > numFloats) 
      {
        const size_t n = numFloats-i;
        auto load = [&](const SubdivMesh::HalfEdge* p) { 
          const unsigned vtx = p->getStartVertexIndex();
          return float8::loadu((float*)&src[vtx*stride],n); 
        };
        float8 Pt, dPdut, dPdvt; 
        feature_adaptive_point_eval<float8>(getHalfEdge(primID),load,u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
        if (P   ) for (size_t j=i; j<numFloats; j++) P[j] = Pt[j-i];
        if (dPdu) for (size_t j=i; j<numFloats; j++) dPdu[j] = dPdut[j-i];
        if (dPdv) for (size_t j=i; j<numFloats; j++) dPdv[j] = dPdvt[j-i];
      } 
      else
      {
        auto load = [&](const SubdivMesh::HalfEdge* p) { 
          const unsigned vtx = p->getStartVertexIndex();
          return float8::loadu((float*)&src[vtx*stride]);
        };
        float8 Pt, dPdut, dPdvt; 
        feature_adaptive_point_eval<float8>(getHalfEdge(primID),load,u,v,P ? &Pt : nullptr, dPdu ? &dPdut : nullptr, dPdv ? &dPdvt : nullptr);
        if (P   ) for (size_t j=i; j<i+8; j++) P[j] = Pt[j-i];
        if (dPdu) for (size_t j=i; j<i+8; j++) dPdu[j] = dPdut[j-i];
        if (dPdv) for (size_t j=i; j<i+8; j++) dPdv[j] = dPdvt[j-i];
      }
    }
    AVX_ZERO_UPPER();
#endif
  }
}
