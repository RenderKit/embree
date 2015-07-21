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

#pragma once

#include "default.h"
#include "accel.h"

namespace embree
{
#define MODE_HIGH_QUALITY (1<<8)
#define LeafMode 0 // FIXME: remove

  /*! virtual interface for all hierarchy builders */
  class Builder : public RefCount {
  public:
    /*! initiates the hierarchy builder */
    virtual void build(size_t threadIndex = 0, size_t threadCount = 0) = 0;

    /*! clears internal builder state */
    virtual void clear() = 0;
  };

  /*! virtual interface for progress monitor class */
  struct BuildProgressMonitor {
    virtual void operator() (size_t dn) = 0;
  };

  /*! build the progress monitor interface from a closure */
  template<typename Closure>
    struct ProgressMonitorClosure : BuildProgressMonitor
  {
  public:
    ProgressMonitorClosure (const Closure& closure) : closure(closure) {}
    void operator() (size_t dn) { closure(dn); }
  private:
    const Closure& closure;
  };
  template<typename Closure> __forceinline const ProgressMonitorClosure<Closure> BuildProgressMonitorFromClosure(const Closure& closure) {
    return ProgressMonitorClosure<Closure>(closure);
  }

// FIXME: simplify ISA selection
#define DECLARE_BUILDER(Accel,Mesh,Args,symbol)                         \
  typedef Builder* (*symbol##Func)(Accel* accel, Mesh* mesh, Args args); \
  namespace isa   { extern Builder* symbol(Accel* accel, Mesh* scene, Args args); } \
  namespace sse41 { extern Builder* symbol(Accel* accel, Mesh* scene, Args args); } \
  namespace avx   { extern Builder* symbol(Accel* accel, Mesh* scene, Args args); } \
  namespace avx2  { extern Builder* symbol(Accel* accel, Mesh* scene, Args args); } \
  namespace avx512  { extern Builder* symbol(Accel* accel, Mesh* scene, Args args); } \
  void symbol##_error() { throw_RTCError(RTC_UNSUPPORTED_CPU,"builder " TOSTRING(symbol) " not supported by your CPU"); } \
  symbol##Func symbol = (symbol##Func) symbol##_error;

  struct TriangleMesh;
  class Scene;
  typedef void     (*createTriangleMeshAccelTy)(TriangleMesh* mesh, AccelData*& accel, Builder*& builder); 

  typedef Builder* (*BVH4BuilderTopLevelFunc  )(void* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel);

#define DECLARE_TOPLEVEL_BUILDER(symbol)                                         \
  namespace isa   { extern Builder* symbol(void* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  namespace sse41 { extern Builder* symbol(void* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  namespace avx   { extern Builder* symbol(void* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  namespace avx2  { extern Builder* symbol(void* accel, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel); } \
  void symbol##_error() { throw_RTCError(RTC_UNSUPPORTED_CPU,"builder " TOSTRING(symbol) " not supported by your CPU"); } \
  BVH4BuilderTopLevelFunc symbol = (BVH4BuilderTopLevelFunc) symbol##_error;
}
