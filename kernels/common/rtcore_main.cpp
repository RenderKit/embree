// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#ifdef _WIN32
#  define RTCORE_API extern "C" __declspec(dllexport)
#else
#  define RTCORE_API extern "C" __attribute__ ((visibility ("default")))
#endif

#include "default.h"
#include "device.h"
#include "scene.h"
#include "context.h"
#include "../../include/embree2/rtcore_ray.h"

#define DECLARE_SYMBOL(fun)                                     \
  namespace sse2      { extern fun; }                           \
  namespace sse41     { extern fun; }                           \
  namespace sse42     { extern fun; }                           \
  namespace avx       { extern fun; }                           \
  namespace avx2      { extern fun; }                           \
  namespace avx512knl { extern fun; }                           \
  namespace avx512skx { extern fun; }

#if defined(__TARGET_SSE2__)
#define SELECT_SSE2(f,name) if (hasISA(SSE2)) f = &sse2::name;
#else
#define SELECT_SSE2(f,name)
#endif

#if defined(__TARGET_SSE42__)
#define SELECT_SSE42(f,name) if (hasISA(SSE42)) f = &sse42::name;
#else
#define SELECT_SSE42(f,name)
#endif

#if defined(__TARGET_AVX__)
#define SELECT_AVX(f,name) if (hasISA(AVX)) f = &avx::name;
#else
#define SELECT_AVX(f,name)
#endif

#if defined(__TARGET_AVX2__)
#define SELECT_AVX2(f,name) if (hasISA(AVX2)) f = &avx2::name;
#else
#define SELECT_AVX2(f,name)
#endif

#if defined(__TARGET_AVX512KNL__)
#define SELECT_AVX512KNL(f,name) if (hasISA(AVX512KNL)) f = &avx512knl::name;
#else
#define SELECT_AVX512KNL(f,name)
#endif

#if defined(__TARGET_AVX512SKX__)
#define SELECT_AVX512SKX(f,name) if (hasISA(AVX512SKX)) f = &avx512skx::name;
#else
#define SELECT_AVX512SKX(f,name)
#endif

#define SELECT_SYMBOL(f,name)                   \
  decltype(name)* f = nullptr;                  \
  SELECT_AVX512SKX(f,name);                     \
  SELECT_AVX512KNL(f,name);                     \
  SELECT_AVX2(f,name);                          \
  SELECT_AVX(f,name);                           \
  SELECT_SSE42(f,name);                         \
  SELECT_SSE2(f,name);

#define NARGS(...) NARGS_(__VA_ARGS__,RSEQ())
#define NARGS_(...) ARGS_N(__VA_ARGS__)
#define ARGS_N(\
  _1, _2, _3, _4, _5, _6, _7, _8, _9,_10,           \
  _11,_12,_13,_14,_15,_16,_17,_18,_19,_20,        \
  _21,_22,_23,_24,_25,_26,_27,_28,_29,_30,        \
  _31,_32,_33,_34,_35,_36,_37,_38,_39,_40,        \
  _41,_42,_43,_44,_45,_46,_47,_48,_49,_50,        \
  _51,_52,_53,_54,_55,_56,_57,_58,_59,_60,        \
  _61,_62,_63,N,...) N
#define RSEQ() \
  63,62,61,60,                          \
  59,58,57,56,55,54,53,52,51,50,      \
  49,48,47,46,45,44,43,42,41,40,      \
  39,38,37,36,35,34,33,32,31,30,      \
  29,28,27,26,25,24,23,22,21,20,      \
  19,18,17,16,15,14,13,12,11,10,      \
  9,8,7,6,5,4,3,2,1,0

#define CALL1(first,...)
#define CALL2(first,second,...) second
#define CALL4(first,second,...) second, CALL2(__VA_ARGS__)
#define CALL6(first,second,...) second, CALL4(__VA_ARGS__)
#define CALL8(first,second,...) second, CALL6(__VA_ARGS__)
#define CALL10(first,second,...) second, CALL8(__VA_ARGS__)
#define CALL12(first,second,...) second, CALL10(__VA_ARGS__)
#define CALL14(first,second,...) second, CALL12(__VA_ARGS__)
#define CALL16(first,second,...) second, CALL14(__VA_ARGS__)
#define CALL18(first,second,...) second, CALL16(__VA_ARGS__)
#define CALL20(first,second,...) second, CALL18(__VA_ARGS__)
#define CALL22(first,second,...) second, CALL20(__VA_ARGS__)
#define CALL24(first,second,...) second, CALL22(__VA_ARGS__)
#define CALL26(first,second,...) second, CALL24(__VA_ARGS__)
#define CALL28(first,second,...) second, CALL26(__VA_ARGS__)
#define CALL__(N,...) CALL##N(__VA_ARGS__,__end__)
#define CALL_(N,...) CALL__(N,__VA_ARGS__)
#define CALL(...) CALL_(NARGS(__VA_ARGS__),__VA_ARGS__)

#define PAIRS1(first,...)
#define PAIRS2(first,second,...) first second
#define PAIRS4(first,second,...) first second, PAIRS2(__VA_ARGS__)
#define PAIRS6(first,second,...) first second, PAIRS4(__VA_ARGS__)
#define PAIRS8(first,second,...) first second, PAIRS6(__VA_ARGS__)
#define PAIRS10(first,second,...) first second, PAIRS8(__VA_ARGS__)
#define PAIRS12(first,second,...) first second, PAIRS10(__VA_ARGS__)
#define PAIRS14(first,second,...) first second, PAIRS12(__VA_ARGS__)
#define PAIRS16(first,second,...) first second, PAIRS14(__VA_ARGS__)
#define PAIRS18(first,second,...) first second, PAIRS16(__VA_ARGS__)
#define PAIRS20(first,second,...) first second, PAIRS18(__VA_ARGS__)
#define PAIRS22(first,second,...) first second, PAIRS20(__VA_ARGS__)
#define PAIRS24(first,second,...) first second, PAIRS22(__VA_ARGS__)
#define PAIRS26(first,second,...) first second, PAIRS24(__VA_ARGS__)
#define PAIRS28(first,second,...) first second, PAIRS26(__VA_ARGS__)
#define PAIRS__(N,...) PAIRS##N(__VA_ARGS__,__end__)
#define PAIRS_(N,...) PAIRS__(N,__VA_ARGS__)
#define PAIRS(...) PAIRS_(NARGS(__VA_ARGS__),__VA_ARGS__)

#define BIND_FUNCTION(dty,name,...) \
  DECLARE_SYMBOL(dty name(PAIRS(__VA_ARGS__)));                         \
  RTCORE_API dty name(PAIRS(__VA_ARGS__)) {                             \
    SELECT_SYMBOL(f,name);                                              \
    return f(CALL(__VA_ARGS__));                                        \
  }

namespace embree
{  
  __forceinline bool hasISA(int _isa) {
    return (getCPUFeatures() & _isa) == _isa;
  }

  BIND_FUNCTION(RTCDevice,rtcNewDevice,const char*,cfg);
  BIND_FUNCTION(void,rtcDeleteDevice,RTCDevice,device);
  BIND_FUNCTION(void,rtcInit,const char*,cfg);
  BIND_FUNCTION(void,rtcExit,void);
  BIND_FUNCTION(void,rtcSetParameter1i,const RTCParameter,parm,ssize_t,val);
  BIND_FUNCTION(ssize_t,rtcGetParameter1i,const RTCParameter,parm);
  BIND_FUNCTION(void,rtcDeviceSetParameter1i,RTCDevice,hdevice,const RTCParameter,parm,ssize_t,val);
  BIND_FUNCTION(ssize_t,rtcDeviceGetParameter1i,RTCDevice,hdevice,const RTCParameter,parm);
  BIND_FUNCTION(RTCError,rtcGetError,void);
  BIND_FUNCTION(RTCError,rtcDeviceGetError,RTCDevice,hdevice);
  BIND_FUNCTION(void,rtcSetErrorFunction,RTCErrorFunc,func);
  BIND_FUNCTION(void,rtcDeviceSetErrorFunction,RTCDevice,hdevice,RTCErrorFunc,func);
  BIND_FUNCTION(void,rtcDeviceSetErrorFunction2,RTCDevice,hdevice,RTCErrorFunc2,func,void*,userPtr);
  BIND_FUNCTION(void,rtcSetMemoryMonitorFunction,RTCMemoryMonitorFunc,func);
  BIND_FUNCTION(void,rtcDeviceSetMemoryMonitorFunction,RTCDevice,hdevice,RTCMemoryMonitorFunc,func);
  BIND_FUNCTION(void,rtcDeviceSetMemoryMonitorFunction2,RTCDevice,hdevice,RTCMemoryMonitorFunc2,func,void*,userPtr);
  BIND_FUNCTION(void,rtcDebug,void);
  BIND_FUNCTION(RTCScene,rtcNewScene,RTCSceneFlags,flags,RTCAlgorithmFlags,aflags);
  BIND_FUNCTION(RTCScene,rtcDeviceNewScene,RTCDevice,device,RTCSceneFlags,flags,RTCAlgorithmFlags,aflags);
  BIND_FUNCTION(void,rtcSetProgressMonitorFunction,RTCScene,hscene,RTCProgressMonitorFunc,func,void*,ptr);
  BIND_FUNCTION(void,rtcCommit,RTCScene,hscene);
  BIND_FUNCTION(void,rtcCommitJoin,RTCScene,hscene);
  BIND_FUNCTION(void,rtcCommitThread,RTCScene,hscene,unsigned int,threadID,unsigned int,numThreads);
  BIND_FUNCTION(void,rtcGetBounds,RTCScene,hscene,RTCBounds&,bounds_o);
  BIND_FUNCTION(void,rtcGetLinearBounds,RTCScene,hscene,RTCBounds*,bounds_o);
  BIND_FUNCTION(void,rtcIntersect,RTCScene,hscene,RTCRay&,ray);
  BIND_FUNCTION(void,rtcIntersect1Ex,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay&,ray);
  BIND_FUNCTION(void,rtcIntersect4,const void*,valid,RTCScene,hscene,RTCRay4&,ray);
  BIND_FUNCTION(void,rtcIntersect4Ex,const void*,valid,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay4&,ray);
  BIND_FUNCTION(void,rtcIntersect8,const void*,valid,RTCScene,hscene,RTCRay8&,ray);
  BIND_FUNCTION(void,rtcIntersect8Ex,const void*,valid,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay8&,ray);
  BIND_FUNCTION(void,rtcIntersect16,const void*,valid,RTCScene,hscene,RTCRay16&,ray);
  BIND_FUNCTION(void,rtcIntersect16Ex,const void*,valid,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay16&,ray);
  BIND_FUNCTION(void,rtcIntersect1M,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay*,rays,const size_t,M,const size_t,stride);
  BIND_FUNCTION(void,rtcIntersect1Mp,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay**,rays,const size_t,M);
  BIND_FUNCTION(void,rtcIntersectNM,RTCScene,hscene,const RTCIntersectContext*,user_context,struct RTCRayN*,rays,const size_t,N,const size_t,M,const size_t,stride);
  BIND_FUNCTION(void,rtcIntersectNp,RTCScene,hscene,const RTCIntersectContext*,user_context,const RTCRayNp&,rays,const size_t,N);
  BIND_FUNCTION(void,rtcOccluded,RTCScene,hscene,RTCRay&,ray);
  BIND_FUNCTION(void,rtcOccluded1Ex,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay&,ray);
  BIND_FUNCTION(void,rtcOccluded4,const void*,valid,RTCScene,hscene,RTCRay4&,ray);
  BIND_FUNCTION(void,rtcOccluded4Ex,const void*,valid,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay4&,ray);
  BIND_FUNCTION(void,rtcOccluded8,const void*,valid,RTCScene,hscene,RTCRay8&,ray);
  BIND_FUNCTION(void,rtcOccluded8Ex,const void*,valid,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay8&,ray);
  BIND_FUNCTION(void,rtcOccluded16,const void*,valid,RTCScene,hscene,RTCRay16&,ray);
  BIND_FUNCTION(void,rtcOccluded16Ex,const void*,valid,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay16&,ray);
  BIND_FUNCTION(void,rtcOccluded1M,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay*,rays,const size_t,M,const size_t,stride);
  BIND_FUNCTION(void,rtcOccluded1Mp,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRay**,rays,const size_t,M);
  BIND_FUNCTION(void,rtcOccludedNM,RTCScene,hscene,const RTCIntersectContext*,user_context,RTCRayN*,rays,const size_t,N,const size_t,M,const size_t,stride);
  BIND_FUNCTION(void,rtcOccludedNp,RTCScene,hscene,const RTCIntersectContext*,user_context,const RTCRayNp&,rays,const size_t,N);
  BIND_FUNCTION(void,rtcDeleteScene,RTCScene,hscene);
  BIND_FUNCTION(unsigned,rtcNewInstance,RTCScene,htarget,RTCScene,hsource);
  BIND_FUNCTION(unsigned,rtcNewInstance2,RTCScene,htarget,RTCScene,hsource,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewGeometryInstance,RTCScene,hscene,unsigned,geomID);
  BIND_FUNCTION(unsigned,rtcNewGeometryGroup,RTCScene,hscene,RTCGeometryFlags,flags,unsigned*,geomIDs,size_t,N);
  BIND_FUNCTION(void,rtcSetTransform,RTCScene,hscene,unsigned,geomID,RTCMatrixType,layout,const float*,xfm);
  BIND_FUNCTION(void,rtcSetTransform2,RTCScene,hscene,unsigned,geomID,RTCMatrixType,layout,const float*,xfm,size_t,timeStep);
  BIND_FUNCTION(unsigned,rtcNewUserGeometry,RTCScene,hscene,size_t,numItems);
  BIND_FUNCTION(unsigned,rtcNewUserGeometry2,RTCScene,hscene,size_t,numItems,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewUserGeometry3,RTCScene,hscene,RTCGeometryFlags,gflags,size_t,numItems,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewTriangleMesh,RTCScene,hscene,RTCGeometryFlags,flags,size_t,numTriangles,size_t,numVertices,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewQuadMesh,RTCScene,hscene,RTCGeometryFlags,flags,size_t,numQuads,size_t,numVertices,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewHairGeometry,RTCScene,hscene,RTCGeometryFlags,flags,size_t,numCurves,size_t,numVertices,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewBezierHairGeometry,RTCScene,hscene,RTCGeometryFlags,flags,unsigned int,numCurves,unsigned int,numVertices,unsigned int,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewBSplineHairGeometry,RTCScene,hscene,RTCGeometryFlags,flags,unsigned int,numCurves,unsigned int,numVertices,unsigned int,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewCurveGeometry,RTCScene,hscene,RTCGeometryFlags,flags,size_t,numCurves,size_t,numVertices,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewBezierCurveGeometry,RTCScene,hscene,RTCGeometryFlags,flags,unsigned int,numCurves,unsigned int,numVertices,unsigned int,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewBSplineCurveGeometry,RTCScene,hscene,RTCGeometryFlags,flags,unsigned int,numCurves,unsigned int,numVertices,unsigned int,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewLineSegments,RTCScene,hscene,RTCGeometryFlags,flags,size_t,numSegments,size_t,numVertices,size_t,numTimeSteps);
  BIND_FUNCTION(unsigned,rtcNewSubdivisionMesh,RTCScene,hscene,RTCGeometryFlags,flags,size_t,numFaces,size_t,numEdges,size_t,numVertices,size_t,numEdgeCreases,size_t,numVertexCreases,size_t,numHoles,size_t,numTimeSteps);
  BIND_FUNCTION(void,rtcSetMask,RTCScene,hscene,unsigned,geomID,int,mask);
  BIND_FUNCTION(void,rtcSetBoundaryMode,RTCScene,hscene,unsigned,geomID,RTCBoundaryMode,mode);
  BIND_FUNCTION(void,rtcSetSubdivisionMode,RTCScene,hscene,unsigned,geomID,unsigned,topologyID,RTCSubdivisionMode,mode);
  BIND_FUNCTION(void,rtcSetIndexBuffer,RTCScene,hscene,unsigned,geomID,RTCBufferType,vertexBuffer,RTCBufferType,indexBuffer);
  BIND_FUNCTION(void*,rtcMapBuffer,RTCScene,hscene,unsigned,geomID,RTCBufferType,type);
  BIND_FUNCTION(void,rtcUnmapBuffer,RTCScene,hscene,unsigned,geomID,RTCBufferType,type);
  BIND_FUNCTION(void,rtcSetBuffer,RTCScene,hscene,unsigned,geomID,RTCBufferType,type,const void*,ptr,size_t,offset,size_t,stride);
  BIND_FUNCTION(void,rtcSetBuffer2,RTCScene,hscene,unsigned,geomID,RTCBufferType,type,const void*,ptr,size_t,offset,size_t,stride,size_t,size);
  BIND_FUNCTION(void,rtcEnable,RTCScene,hscene,unsigned,geomID);
  BIND_FUNCTION(void,rtcUpdate,RTCScene,hscene,unsigned,geomID);
  BIND_FUNCTION(void,rtcUpdateBuffer,RTCScene,hscene,unsigned,geomID,RTCBufferType,type);
  BIND_FUNCTION(void,rtcDisable,RTCScene,hscene,unsigned,geomID);
  BIND_FUNCTION(void,rtcDeleteGeometry,RTCScene,hscene,unsigned,geomID);
  BIND_FUNCTION(void,rtcSetTessellationRate,RTCScene,hscene,unsigned,geomID,float,tessellationRate);
  BIND_FUNCTION(void,rtcSetUserData,RTCScene,hscene,unsigned,geomID,void*,ptr);
  BIND_FUNCTION(void*,rtcGetUserData,RTCScene,hscene,unsigned,geomID);
  BIND_FUNCTION(void,rtcSetBoundsFunction,RTCScene,hscene,unsigned,geomID,RTCBoundsFunc,bounds);
  BIND_FUNCTION(void,rtcSetBoundsFunction2,RTCScene,hscene,unsigned,geomID,RTCBoundsFunc2,bounds,void*,userPtr);
  BIND_FUNCTION(void,rtcSetBoundsFunction3,RTCScene,hscene,unsigned,geomID,RTCBoundsFunc3,bounds,void*,userPtr);
  BIND_FUNCTION(void,rtcSetDisplacementFunction,RTCScene,hscene,unsigned,geomID,RTCDisplacementFunc,func,RTCBounds*,bounds);
  BIND_FUNCTION(void,rtcSetDisplacementFunction2,RTCScene,hscene,unsigned,geomID,RTCDisplacementFunc2,func,RTCBounds*,bounds);
  BIND_FUNCTION(void,rtcSetIntersectFunction,RTCScene,hscene,unsigned,geomID,RTCIntersectFunc,intersect);
  BIND_FUNCTION(void,rtcSetIntersectFunction4,RTCScene,hscene,unsigned,geomID,RTCIntersectFunc4,intersect4);
  BIND_FUNCTION(void,rtcSetIntersectFunction8,RTCScene,hscene,unsigned,geomID,RTCIntersectFunc8,intersect8);
  BIND_FUNCTION(void,rtcSetIntersectFunction16,RTCScene,hscene,unsigned,geomID,RTCIntersectFunc16,intersect16);
  BIND_FUNCTION(void,rtcSetIntersectFunction1Mp,RTCScene,hscene,unsigned,geomID,RTCIntersectFunc1Mp,intersect);
  BIND_FUNCTION(void,rtcSetIntersectFunctionN,RTCScene,hscene,unsigned,geomID,RTCIntersectFuncN,intersect);
  BIND_FUNCTION(void,rtcSetOccludedFunction,RTCScene,hscene,unsigned,geomID,RTCOccludedFunc,occluded);
  BIND_FUNCTION(void,rtcSetOccludedFunction4,RTCScene,hscene,unsigned,geomID,RTCOccludedFunc4,occluded4);
  BIND_FUNCTION(void,rtcSetOccludedFunction8,RTCScene,hscene,unsigned,geomID,RTCOccludedFunc8,occluded8);
  BIND_FUNCTION(void,rtcSetOccludedFunction16,RTCScene,hscene,unsigned,geomID,RTCOccludedFunc16,occluded16);
  BIND_FUNCTION(void,rtcSetOccludedFunction1Mp,RTCScene,hscene,unsigned,geomID,RTCOccludedFunc1Mp,occluded);
  BIND_FUNCTION(void,rtcSetOccludedFunctionN,RTCScene,hscene,unsigned,geomID,RTCOccludedFuncN,occluded);
  BIND_FUNCTION(void,rtcSetIntersectionFilterFunction,RTCScene,hscene,unsigned,geomID,RTCFilterFunc,intersect);
  BIND_FUNCTION(void,rtcSetIntersectionFilterFunction4,RTCScene,hscene,unsigned,geomID,RTCFilterFunc4,filter4);
  BIND_FUNCTION(void,rtcSetIntersectionFilterFunction8,RTCScene,hscene,unsigned,geomID,RTCFilterFunc8,filter8);
  BIND_FUNCTION(void,rtcSetIntersectionFilterFunction16,RTCScene,hscene,unsigned,geomID,RTCFilterFunc16,filter16);
  BIND_FUNCTION(void,rtcSetIntersectionFilterFunctionN,RTCScene,hscene,unsigned,geomID,RTCFilterFuncN,filterN);
  BIND_FUNCTION(void,rtcSetOcclusionFilterFunction,RTCScene,hscene,unsigned,geomID,RTCFilterFunc,intersect);
  BIND_FUNCTION(void,rtcSetOcclusionFilterFunction4,RTCScene,hscene,unsigned,geomID,RTCFilterFunc4,filter4);
  BIND_FUNCTION(void,rtcSetOcclusionFilterFunction8,RTCScene,hscene,unsigned,geomID,RTCFilterFunc8,filter8);
  BIND_FUNCTION(void,rtcSetOcclusionFilterFunction16,RTCScene,hscene,unsigned,geomID,RTCFilterFunc16,filter16);
  BIND_FUNCTION(void,rtcSetOcclusionFilterFunctionN,RTCScene,hscene,unsigned,geomID,RTCFilterFuncN,filterN);
  BIND_FUNCTION(void,rtcInterpolate,RTCScene,hscene,unsigned,geomID,unsigned,primID,float,u,float,v,RTCBufferType,buffer,float*,P,float*,dPdu,float*,dPdv,size_t,numFloats);
  BIND_FUNCTION(void,rtcInterpolate2,RTCScene,hscene,unsigned,geomID,unsigned,primID,float,u,float,v,RTCBufferType,buffer,float*,P,float*,dPdu,float*,dPdv,float*,ddPdudu,float*,ddPdvdv,float*,ddPdudv,size_t,numFloats);
#if defined(EMBREE_RAY_PACKETS)
  BIND_FUNCTION(void,rtcInterpolateN,RTCScene,hscene,unsigned,geomID,const void*,valid_i,const unsigned*,primIDs,const float*,u,const float*,v,size_t,numUVs,RTCBufferType,buffer,float*,P,float*,dPdu,float*,dPdv,size_t,numFloats);
  BIND_FUNCTION(void,rtcInterpolateN2,RTCScene,hscene,unsigned,geomID,const void*,valid_i,const unsigned*,primIDs,const float*,u,const float*,v,size_t,numUVs,RTCBufferType,buffer,float*,P,float*,dPdu,float*,dPdv,float*,ddPdudu,float*,ddPdvdv,float*,ddPdudv,size_t,numFloats);
#endif
}
