
////////////////////////////////////////////////////////////////////////////////////////
// rtcore.h

@@
@@
-rtcNewDevice ()
+rtcNewDevice (NULL)

@@
expression e0;
@@
-rtcDeleteDevice (e0)
+rtcReleaseDevice (e0)

@@
@@
-rtcInit ()
+rtcInit (NULL)

@@
expression e0;
@@
-rtcInit (e0);
+RTCDevice device = rtcInit (e0);

@@
@@
-rtcExit ()
+rtcReleaseDevice (device)

@@
expression e0,e1;
@@
-rtcSetParameter1i (e0,e1)
+rtcDeviceSetParameter1i (device,e0,e1)

@@
expression e0;
@@
-rtcGetParameter1i (e0)
+rtcDeviceGetParameter1i (device,e0)

@@
@@
-rtcGetError ()
+rtcDeviceGetError (device)

@@
@@
-RTC_ERROR_FUNCTION
+RTCErrorFunc

@@
@@
-RTCErrorFunc2
+RTCErrorFunc

@@
expression e0;
@@
-rtcSetErrorFunction(e0)
+rtcDeviceSetErrorFunction(device,e0,NULL)

@@
expression e0,e1,e2;
@@
-rtcDeviceSetErrorFunction2(e0,e1,e2)
+rtcDeviceSetErrorFunction(e0,e1,e2)

@@
@@
-RTC_MEMORY_MONITOR_FUNCTION
+RTCMemoryMonitorFunc

@@
@@
-RTCMemoryMonitorFunc2
+RTCMemoryMonitorFunc

@@
expression e0;
@@
-rtcSetMemoryMonitorFunction(e0)
+rtcDeviceSetMemoryMonitorFunction(device,e0,nullptr)

@@
expression e0,e1,e2;
@@
-rtcDeviceSetMemoryMonitorFunction2(e0,e1,e2)
+rtcDeviceSetMemoryMonitorFunction(e0,e1,e2)

@@
@@
-rtcDebug();

////////////////////////////////////////////////////////////////////////////////////////
// rtcore_scene.h

//@@
//@@
//-RTC_SCENE_STATIC

@@
@@
-RTC_SCENE_DYNAMIC
+RTC_SCENE_FLAG_DYNAMIC

@@
@@
-RTC_SCENE_COMPACT
+RTC_ACCEL_COMPACT

//@@
//@@
//-RTC_SCENE_COHERENT

//@@
//@@
//-RTC_SCENE_INCOHERENT

@@
@@
-RTC_SCENE_HIGH_QUALITY
+RTC_BUILD_QUALITY_HIGH

@@
@@
-RTC_SCENE_ROBUST
+RTC_ACCEL_ROBUST

@@
identifier id;
@@
{
-RTCAlgorithmFlags id = ...;
<+...
-id = ...;
...+>
}

@@
identifier context;
@@
RTCIntersectContext context;
+rtcInitIntersectionContext(&context);

@@
expression e0,e1;
@@
-rtcNewScene(e0,e1)
+rtcDeviceNewScene(device,e0,e1)

@@
identifier scene;
expression device,sflags,aflags;
@@
-RTCScene scene = rtcDeviceNewScene(device,sflags,aflags);
+RTCScene scene = rtcDeviceNewScene(device);
+rtcSetAccelFlags(scene,sflags);
+rtcSetBuildQuality(scene,sflags);
+rtcSetSceneFlags(scene,sflags);

@@
expression scene;
@@
-rtcDeleteScene(scene)
+rtcReleaseScene(scene)

@@
@@
-RTC_PROGRESS_MONITOR_FUNCTION
+RTCProgressMonitorFunc

@@
expression scene,threadID,numThreads;
@@
-rtcCommitThread(scene,threadID,numThreads)
+rtcCommitJoin(scene)

@@
expression scene,bounds;
@@
-rtcGetBounds(scene,bounds)
+rtcGetBounds(scene,&bounds)

@@
expression scene,ray;
@@
-rtcIntersect(scene,ray);
+RTCIntersectContext context;
+rtcInitIntersectContext(&context);
+rtcIntersect1(scene,&context,&ray);

@@
expression scene,context,ray;
@@
-rtcIntersect1Ex(scene,context,ray);
+rtcIntersect1(scene,context,&ray);

@@
expression valid,scene,ray;
@@
-rtcIntersect4(valid,scene,ray);
+RTCIntersectContext context;
+rtcInitIntersectContext(&context);
+rtcIntersect4(valid,scene,&context,&ray);

@@
expression valid,scene,context,ray;
@@
-rtcIntersect4Ex(valid,scene,context,ray);
+rtcIntersect4(valid,scene,context,&ray);

@@
expression valid,scene,ray;
@@
-rtcIntersect8(valid,scene,ray);
+RTCIntersectContext context;
+rtcInitIntersectContext(&context);
+rtcIntersect8(valid,scene,&context,&ray);

@@
expression valid,scene,context,ray;
@@
-rtcIntersect8Ex(valid,scene,context,ray);
+rtcIntersect8(valid,scene,context,&ray);

@@
expression valid,scene,ray;
@@
-rtcIntersect16(valid,scene,ray);
+RTCIntersectContext context;
+rtcInitIntersectContext(&context);
+rtcIntersect16(valid,scene,&context,&ray);

@@
expression valid,scene,context,ray;
@@
-rtcIntersect16Ex(valid,scene,context,ray);
+rtcIntersect16(valid,scene,context,&ray);

@@
expression scene,context,ray,N;
@@
-rtcIntersectNp(scene,context,ray,N);
+rtcIntersectNp(scene,context,&ray,N);


@@
expression scene,ray;
@@
-rtcOccluded(scene,ray);
+RTCOccludedContext context;
+rtcInitOccludedContext(&context);
+rtcOccluded1(scene,&context,&ray);

@@
expression scene,context,ray;
@@
-rtcOccluded1Ex(scene,context,ray);
+rtcOccluded1(scene,context,&ray);

@@
expression valid,scene,ray;
@@
-rtcOccluded4(valid,scene,ray);
+RTCOccludedContext context;
+rtcInitOccludedContext(&context);
+rtcOccluded4(valid,scene,&context,&ray);

@@
expression valid,scene,context,ray;
@@
-rtcOccluded4Ex(valid,scene,context,ray);
+rtcOccluded4(valid,scene,context,&ray);

@@
expression valid,scene,ray;
@@
-rtcOccluded8(valid,scene,ray);
+RTCOccludedContext context;
+rtcInitOccludedContext(&context);
+rtcOccluded8(valid,scene,&context,&ray);

@@
expression valid,scene,context,ray;
@@
-rtcOccluded8Ex(valid,scene,context,ray);
+rtcOccluded8(valid,scene,context,&ray);

@@
expression valid,scene,ray;
@@
-rtcOccluded16(valid,scene,ray);
+RTCOccludedContext context;
+rtcInitOccludedContext(&context);
+rtcOccluded16(valid,scene,&context,&ray);

@@
expression valid,scene,context,ray;
@@
-rtcOccluded16Ex(valid,scene,context,ray);
+rtcOccluded16(valid,scene,context,&ray);

@@
expression scene,context,ray,N;
@@
-rtcOccludedNp(scene,context,ray,N);
+rtcOccludedNp(scene,context,&ray,N);


////////////////////////////////////////////////////////////////////////////////////////
// rtcore_geometry.h

@@
@@
-RTC_GEOMETRY_STATIC
+RTC_BUILD_QUALITY_MEDIUM

@@
@@
-RTC_GEOMETRY_DYNAMIC
+RTC_BUILD_QUALITY_LOW

@@
@@
-RTC_GEOMETRY_DEFORMABLE
+RTC_BUILD_QUALITY_REFIT

@@
@@
-RTC_BOUNDARY_NONE
+RTC_SUBDIV_NO_BOUNDARY

@@
@@
-RTC_BOUNDARY_SMOOTH
+RTC_SUBDIV_SMOOTH_BOUNDARY

@@
@@
-RTC_BOUNDARY_EDGE_ONLY
+RTC_SUBDIV_SMOOTH_BOUNDARY

@@
@@
-RTC_BOUNDARY_EDGE_AND_CORNER
+RTC_SUBDIV_PIN_CORNERS

@@
@@
-RTCBoundaryMode
+RTCSubdivisionMode

@@
expression scene,geomID,mode;
@@
-rtcSetBoundaryMode(scene,geomID,mode)
+rtcSetSubdivisionMode(scene,geomID,mode)


////////////////////////////////////////////////////////////////////////////////////////
// geometry modification functions

@@
expression scene,geomID,rate;
@@
-rtcSetTessellationRate(scene,geomID,rate)
+rtcSetTessellationRate(rtcGetGeometry(scene,geomID),rate)

@@
expression scene,geomID,mask;
@@
-rtcSetMask(scene,geomID,mask)
+rtcSetMask(rtcGetGeometry(scene,geomID),mask)

@@
expression scene,geomID,topology,mode;
@@
-rtcSetSubdivisionMode(scene,geomID,topology,mode)
+rtcSetSubdivisionMode(rtcGetGeometry(scene,geomID),topology,mode)

@@
expression scene,geomID,buf0,buf1;
@@
-rtcSetIndexBuffer(scene,geomID,buf0,buf1)
+rtcSetIndexBuffer(rtcGetGeometry(scene,geomID),buf0,buf1)

@@
expression scene,geomID,type;
@@
-rtcMapBuffer(scene,geomID,type)
+rtcMapBuffer(rtcGetGeometry(scene,geomID),type)

@@
expression scene,geomID,type;
@@
-rtcUnmapBuffer(scene,geomID,type)
+rtcUnmapBuffer(rtcGetGeometry(scene,geomID),type)

@@
expression scene,geomID,type,ptr,offset,stride;
@@
-rtcSetBuffer(scene,geomID,type,ptr,offset,stride)
+rtcSetBuffer(rtcGetGeometry(scene,geomID),type,ptr,offset,stride)

@@
expression scene,geomID,type,ptr,offset,stride,size;
@@
-rtcSetBuffer2(scene,geomID,type,ptr,offset,stride,size)
+rtcSetBuffer(rtcGetGeometry(scene,geomID),type,ptr,offset,stride,size)

@@
expression scene,geomID;
@@
-rtcEnable(scene,geomID)
+rtcEnable(rtcGetGeometry(scene,geomID))

@@
expression scene,geomID;
@@
-rtcDisable(scene,geomID)
+rtcDisable(rtcGetGeometry(scene,geomID))

@@
expression scene,geomID;
@@
-rtcUpdate(scene,geomID)
+rtcCommitGeometry(rtcGetGeometry(scene,geomID))

@@
expression scene,geomID,type;
@@
-rtcUpdateBuffer(scene,geomID,type)
+rtcUpdateBuffer(rtcGetGeometry(scene,geomID),type)


@@
expression scene,geomID,func,bounds;
@@
-rtcSetDisplacementFunction(scene,geomID,func,bounds)
+rtcSetDisplacementFunction(rtcGetGeometry(scene,geomID),func,bounds)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetDisplacementFunction2(scene,geomID,func,bounds)
+rtcSetDisplacementFunction(rtcGetGeometry(scene,geomID),func,bounds)


@@
expression scene,geomID,func,bounds;
@@
-rtcSetIntersectionFilterFunction(scene,geomID,func)
+rtcSetIntersectionFilterFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetIntersectionFilterFunction4(scene,geomID,func)
+rtcSetIntersectionFilterFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetIntersectionFilterFunction8(scene,geomID,func)
+rtcSetIntersectionFilterFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetIntersectionFilterFunction16(scene,geomID,func)
+rtcSetIntersectionFilterFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetOcclusionFilterFunction(scene,geomID,func)
+rtcSetOcclusionFilterFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetOcclusionFilterFunction4(scene,geomID,func)
+rtcSetOcclusionFilterFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetOcclusionFilterFunction8(scene,geomID,func)
+rtcSetOcclusionFilterFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func,bounds;
@@
-rtcSetOcclusionFilterFunction16(scene,geomID,func)
+rtcSetOcclusionFilterFunctionN(rtcGetGeometry(scene,geomID),func)


@@
expression scene,geomID,ptr;
@@
-rtcSetUserData(scene,geomID,ptr)
+rtcSetUserData(rtcGetGeometry(scene,geomID),ptr)

@@
expression scene,geomID;
@@
-rtcGetUserData(scene,geomID)
+rtcGetUserData(rtcGetGeometry(scene,geomID))


@@
expression scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,numFloats;
@@
-rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,numFloats)
+rtcInterpolate(rtcGetGeometry(scene,geomID),primID,u,v,buffer,P,dPdu,dPdv,NULL,NULL,NULL,numFloats)

@@
expression scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats;
@@
-rtcInterpolate2(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats)
+rtcInterpolate(rtcGetGeometry(scene,geomID),primID,u,v,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats)


@@
expression scene,geomID,valid,primID,u,v,numUVs,buffer,P,dPdu,dPdv,numFloats;
@@
-rtcInterpolateN(scene,geomID,valid,primID,u,v,numUVs,buffer,P,dPdu,dPdv,numFloats)
+rtcInterpolateN(rtcGetGeometry(scene,geomID),valid,primID,u,v,numUVs,buffer,P,dPdu,dPdv,NULL,NULL,NULL,numFloats)

@@
expression scene,geomID,valid,primID,u,v,numUVs,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats;
@@
-rtcInterpolateN2(scene,geomID,valid,primID,u,v,numUVs,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats)
+rtcInterpolateN(rtcGetGeometry(scene,geomID),valid,primID,u,v,numUVs,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats)



@@
expression scene,geomID,func;
@@
-rtcSetBoundsFunction(scene,geomID,func)
+rtcSetBoundsFunctionX(rtcGetGeometry(scene,geomID),func,NULL)

@@
expression scene,geomID,func,userPtr;
@@
-rtcSetBoundsFunction2(scene,geomID,func,userPtr)
+rtcSetBoundsFunctionX(rtcGetGeometry(scene,geomID),func,userPtr)

@@
expression scene,geomID,func,userPtr;
@@
-rtcSetBoundsFunction3(scene,geomID,func,userPtr)
+rtcSetBoundsFunctionX(rtcGetGeometry(scene,geomID),func,userPtr)

@@
expression geom,func,userPtr;
@@
-rtcSetBoundsFunctionX(geom,func,userPtr)
+rtcSetBoundsFunction(geom,func,userPtr)


@@
expression scene,geomID,func;
@@
-rtcSetIntersectFunction(scene,geomID,func)
+rtcSetIntersectFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetIntersectFunction4(scene,geomID,func)
+rtcSetIntersectFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetIntersectFunction8(scene,geomID,func)
+rtcSetIntersectFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetIntersectFunction16(scene,geomID,func)
+rtcSetIntersectFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetIntersectFunction1Mp(scene,geomID,func)
+rtcSetIntersectFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetIntersectFunctionN(scene,geomID,func)
+rtcSetIntersectFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetOccludedFunction(scene,geomID,func)
+rtcSetOccludedFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetOccludedFunction4(scene,geomID,func)
+rtcSetOccludedFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetOccludedFunction8(scene,geomID,func)
+rtcSetOccludedFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetOccludedFunction16(scene,geomID,func)
+rtcSetOccludedFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetOccludedFunction1Mp(scene,geomID,func)
+rtcSetOccludedFunctionN(rtcGetGeometry(scene,geomID),func)

@@
expression scene,geomID,func;
@@
-rtcSetOccludedFunctionN(scene,geomID,func)
+rtcSetOccludedFunctionN(rtcGetGeometry(scene,geomID),func)





@@
expression scene,geomID;
@@
-rtcDeleteGeometry(scene,geomID)
+rtcReleaseGeometry(rtcGetGeometry(scene,geomID))



////////////////////////////////////////////////////////////////////////////////////////
// instance

@@
expression target,source;
@@
-rtcNewInstance(target,source)
+rtcNewInstance3(target,source,1)

@@
expression target,source;
@@
-rtcNewInstance2(target,source)
+rtcNewInstance3(target,source,1)

@@
expression target,source;
@@
-rtcNewInstance3(target,source)
+rtcNewInstance3(target,source,1)

@@
identifier geomID;
expression target,source,numTimeSteps;
@@
{
...
-unsigned int geomID = rtcNewInstance3(target,source,numTimeSteps);
+RTCGeometry geom = rtcNewInstance (device,source,numTimeSteps);
+geomID = rtcAttachGeometry(target,geom);
}

@@
expression target,source,numTimeSteps,geomID;
@@
{
...
-rtcNewInstance3(target,source,numTimeSteps,geomID);
+RTCGeometry geom = rtcNewInstance (device,source,numTimeSteps);
+rtcAttachGeometryById(target,geom,geomID);
}

@@
expression scene,geomID,layout,xfm;
@@
-rtcSetTransform(scene,geomID,layout,xfm)
+rtcSetTransform2(scene,geomID,layout,xfm,0)

@@
expression scene,geomID,layout,xfm,timeStep;
@@
-rtcSetTransform2(scene,geomID,layout,xfm,timeStep)
+rtcSetTransform(rtcGetGeometry(scene,geomID),layout,xfm,0)


////////////////////////////
// rtcNewTriangleMesh

@@
expression scene,flags,numTriangles,numVertices;
@@
-rtcNewTriangleMesh (scene,flags,numTriangles,numVertices)
+rtcNewTriangleMesh2(scene,flags,numTriangles,numVertices,1)

@@
expression scene,flags,numTriangles,numVertices,numTimeSteps;
@@
-rtcNewTriangleMesh2(scene,flags,numTriangles,numVertices)
+rtcNewTriangleMesh2(scene,flags,numTriangles,numVertices,1)

@@
identifier geomID;
expression scene,flags,numTriangles,numVertices,numTimeSteps;
@@
{
...
-unsigned int geomID = rtcNewTriangleMesh2 (scene,flags,numTriangles,numVertices,numTimeSteps);
+RTCGeometry geom = rtcNewTriangleMesh (device,flags,numTriangles,numVertices,numTimeSteps);
+geomID = rtcAttachGeometry(scene,geom);
}

@@
expression scene,flags,numTriangles,numVertices,numTimeSteps,geomID;
@@
{
...
-rtcNewTriangleMesh2 (scene,flags,numTriangles,numVertices,numTimeSteps,geomID);
+RTCGeometry geom = rtcNewTriangleMesh (device,flags,numTriangles,numVertices,numTimeSteps);
+rtcAttachGeometryById(scene,geom,geomID);
}

@@
identifier geom;
expression device,flags,numTriangles,numVertices,numTimeSteps;
@@
-RTCGeometry geom = rtcNewTriangleMesh (device,flags,numTriangles,numVertices,numTimeSteps);
+RTCGeometry geom = rtcNewTriangleMesh (device,flags,numTriangles,numVertices,numTimeSteps);
+rtcSetBuildQuality(geom,flags);
+rtcSetNumTimeSteps(geom,numTimeSteps);


////////////////////////////
// rtcNewQuadMesh

@@
expression scene,flags,numQuads,numVertices;
@@
-rtcNewQuadMesh (scene,flags,numQuads,numVertices)
+rtcNewQuadMesh2(scene,flags,numQuads,numVertices,1)

@@
expression scene,flags,numQuads,numVertices;
@@
-rtcNewQuadMesh2(scene,flags,numQuads,numVertices)
+rtcNewQuadMesh2(scene,flags,numQuads,numVertices,1)

@@
identifier geomID;
expression scene,flags,numQuads,numVertices,numTimeSteps;
@@
{
...
-unsigned int geomID = rtcNewQuadMesh2(scene,flags,numQuads,numVertices,numTimeSteps);
+RTCGeometry geom = rtcNewQuadMesh (device,flags,numQuads,numVertices,numTimeSteps);
+geomID = rtcAttachGeometry(scene,geom);
}

@@
expression scene,flags,numQuads,numVertices,numTimeSteps,geomID;
@@
{
...
-rtcNewQuadMesh2(scene,flags,numQuads,numVertices,numTimeSteps,geomID);
+RTCGeometry geom = rtcNewQuadMesh (device,flags,numQuads,numVertices,numTimeSteps);
+rtcAttachGeometryById(scene,geom,geomID);
}

@@
identifier geom;
expression device,flags,numQuads,numVertices,numTimeSteps;
@@
-RTCGeometry geom = rtcNewQuadMesh (device,flags,numQuads,numVertices,numTimeSteps);
+RTCGeometry geom = rtcNewQuadMesh (device,flags,numQuads,numVertices,numTimeSteps);
+rtcSetBuildQuality(geom,flags);
+rtcSetNumTimeSteps(geom,numTimeSteps);


////////////////////////////
// rtcNewSubdivisionMesh

@@
expression scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
-rtcNewSubdivisionMesh (scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles)
+rtcNewSubdivisionMesh2(scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,1)

@@
expression scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
-rtcNewSubdivisionMesh2(scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles)
+rtcNewSubdivisionMesh2(scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,1)

@@
identifier geomID;
expression scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
-unsigned int geomID = rtcNewSubdivisionMesh2(scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
+RTCGeometry geom = rtcNewSubdivisionMesh (device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
+geomID = rtcAttachGeometry(scene,geom);
}

@@
expression scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,geomID;
@@
{
...
-rtcNewSubdivisionMesh2(scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,geomID);
+RTCGeometry geom = rtcNewSubdivisionMesh (device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
+rtcAttachGeometryById(scene,geom,geomID);
}

@@
identifier geom;
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
-RTCGeometry geom = rtcNewSubdivisionMesh (device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
+RTCGeometry geom = rtcNewSubdivisionMesh (device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
+rtcSetBuildQuality(geom,flags);
+rtcSetNumTimeSteps(geom,numTimeSteps);


////////////////////////////
// rtcNewCurveGeometry

@@
expression scene,flags,numCurves,numVertices;
@@
-rtcNewHairGeometry (scene,flags,numCurves,numVertices)
+rtcNewHairGeometry (scene,flags,numCurves,numVertices,1)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewHairGeometry (scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewBezierHairGeometry (scene,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices;
@@
-rtcNewBezierHairGeometry (scene,flags,numCurves,numVertices)
+rtcNewBezierHairGeometry (scene,flags,numCurves,numVertices,1)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBezierHairGeometry (scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewBezierHairGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBezierHairGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BEZIER,RTC_GEOMETRY_INTERSECTOR_RIBBON,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps,geomID;
@@
-rtcNewBezierHairGeometry2(scene,flags,numCurves,numVertices,numTimeSteps,geomID)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BEZIER,RTC_GEOMETRY_INTERSECTOR_RIBBON,flags,numCurves,numVertices,numTimeSteps,geomID)

@@
expression scene,flags,numCurves,numVertices;
@@
-rtcNewBSplineHairGeometry (scene,flags,numCurves,numVertices)
+rtcNewBSplineHairGeometry (scene,flags,numCurves,numVertices,1)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBSplineHairGeometry (scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewBSplineHairGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBSplineHairGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BSPLINE,RTC_GEOMETRY_INTERSECTOR_RIBBON,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps,geomID;
@@
-rtcNewBSplineHairGeometry2(scene,flags,numCurves,numVertices,numTimeSteps,geomID)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BSPLINE,RTC_GEOMETRY_INTERSECTOR_RIBBON,flags,numCurves,numVertices,numTimeSteps,geomID)




@@
expression scene,flags,numCurves,numVertices;
@@
-rtcNewCurveGeometry (scene,flags,numCurves,numVertices)
+rtcNewCurveGeometry (scene,flags,numCurves,numVertices,1)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewCurveGeometry (scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewBezierCurveGeometry (scene,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices;
@@
-rtcNewBezierCurveGeometry (scene,flags,numCurves,numVertices)
+rtcNewBezierCurveGeometry (scene,flags,numCurves,numVertices,1)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBezierCurveGeometry (scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewBezierCurveGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBezierCurveGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BEZIER,RTC_GEOMETRY_INTERSECTOR_SURFACE,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps,geomID;
@@
-rtcNewBezierCurveGeometry2(scene,flags,numCurves,numVertices,numTimeSteps,geomID)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BEZIER,RTC_GEOMETRY_INTERSECTOR_SURFACE,flags,numCurves,numVertices,numTimeSteps,geomID)

@@
expression scene,flags,numCurves,numVertices;
@@
-rtcNewBSplineCurveGeometry (scene,flags,numCurves,numVertices)
+rtcNewBSplineCurveGeometry (scene,flags,numCurves,numVertices,1)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBSplineCurveGeometry (scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewBSplineCurveGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps;
@@
-rtcNewBSplineCurveGeometry2(scene,flags,numCurves,numVertices,numTimeSteps)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BSPLINE,RTC_GEOMETRY_INTERSECTOR_SURFACE,flags,numCurves,numVertices,numTimeSteps)

@@
expression scene,flags,numCurves,numVertices,numTimeSteps,geomID;
@@
-rtcNewBSplineCurveGeometry2(scene,flags,numCurves,numVertices,numTimeSteps,geomID)
+rtcNewCurveGeometryX(scene,RTC_BASIS_BSPLINE,RTC_GEOMETRY_INTERSECTOR_SURFACE,flags,numCurves,numVertices,numTimeSteps,geomID)


@@
expression scene,flags,numSegments,numVertices;
@@
-rtcNewLineSegments (scene,flags,numSegments,numVertices)
+rtcNewLineSegments (scene,flags,numSegments,numVertices,1)

@@
expression scene,flags,numSegments,numVertices,numTimeSteps;
@@
-rtcNewLineSegments (scene,flags,numSegments,numVertices,numTimeSteps)
+rtcNewLineSegments2(scene,flags,numSegments,numVertices,numTimeSteps)

@@
expression scene,flags,numSegments,numVertices,numTimeSteps;
@@
-rtcNewLineSegments (scene,flags,numSegments,numVertices,numTimeSteps)
+rtcCurveGeometryX (scene,RTC_BASIS_LINEAR,RTC_GEOMETRY_INTERSECTOR_RIBBON,flags,numSegments,numVertices,numTimeSteps)


@@
identifier geomID;
expression scene,basis,intersector,flags,numCurves,numVertices,numTimeSteps;
@@
{
...
-unsigned int geomID = rtcNewCurveGeometryX (scene,basis,intersector,flags,numCurves,numVertices,numTimeSteps);
+RTCGeometry geom = rtcNewCurveGeometry (device,basis,intersector,flags,numCurves,numVertices,numTimeSteps);
+geomID = rtcAttachGeometry(scene,geom);
}

@@
expression scene,basis,intersector,flags,numCurves,numVertices,numTimeSteps,geomID;
@@
{
...
-rtcNewCurveGeometryX (scene,basis,intersector,flags,numCurves,numVertices,numTimeSteps,geomID);
+RTCGeometry geom = rtcNewCurveGeometry (device,basis,intersector,flags,numCurves,numVertices,numTimeSteps);
+rtcAttachGeometryById(scene,geom,geomID);
}

@@
identifier geom;
expression device,basis,intersector,flags,numCurves,numVertices,numTimeSteps;
@@
-RTCGeometry geom = rtcNewCurveGeometry (device,basis,intersector,flags,numCurves,numVertices,numTimeSteps);
+RTCGeometry geom = rtcNewCurveGeometry (device,basis,intersector,flags,numCurves,numVertices,numTimeSteps);
+rtcSetGeometryIntersector(geom,intersector);
+rtcSetBuildQuality(geom,flags);
+rtcSetNumTimeSteps(geom,numTimeSteps);


/////////////////////////////////
// optimize away rtcGetGeometry

@@
identifier geomID;
expression scene,geom;
@@
{
...
geomID = rtcAttachGeometry(scene,geom);
<+...
-rtcGetGeometry (scene,geomID)
+geom
...+>
}

@@
identifier geomID;
expression scene,geom;
@@
{
...
rtcAttachGeometryById(scene,geom,geomID);
<+...
-rtcGetGeometry (scene,geomID)
+geom
...+>
}

/////////////////////////////////
// buffers

@@
identifier geom;
expression device,flags,numTriangles,numVertices,numTimeSteps;
@@
{
...
RTCGeometry geom = rtcNewTriangleMesh(device,flags,numTriangles,numVertices,numTimeSteps);
<+...
-rtcMapBuffer(geom,RTC_INDEX_BUFFER)
+rtcNewBuffer(geom,RTC_INDEX_BUFFER,3*sizeof(int),numTriangles)
...+>
}

@@
expression device,flags,numTriangles,numVertices,numTimeSteps;
@@
{
...
RTCGeometry geom = rtcNewTriangleMesh(device,flags,numTriangles,numVertices,numTimeSteps);
<+...
-rtcMapBuffer(geom,RTC_VERTEX_BUFFER)
+rtcNewBuffer(geom,RTC_VERTEX_BUFFER,4*sizeof(float),numVertices)
...+>
}

@@
identifier geom;
expression device,flags,numQuads,numVertices,numTimeSteps;
@@
{
...
RTCGeometry geom = rtcNewQuadMesh(device,flags,numQuads,numVertices,numTimeSteps);
<+...
-rtcMapBuffer(geom,RTC_INDEX_BUFFER)
+rtcNewBuffer(geom,RTC_INDEX_BUFFER,3*sizeof(int),numQuads)
...+>
}

@@
expression device,flags,numQuads,numVertices,numTimeSteps;
@@
{
...
RTCGeometry geom = rtcNewQuadMesh(device,flags,numQuads,numVertices,numTimeSteps);
<+...
-rtcMapBuffer(geom,RTC_VERTEX_BUFFER)
+rtcNewBuffer(geom,RTC_VERTEX_BUFFER,4*sizeof(float),numVertices)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_FACE_BUFFER)
+rtcNewBuffer(geom,RTC_FACE_BUFFER,sizeof(int),numFaces)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_INDEX_BUFFER)
+rtcNewBuffer(geom,RTC_INDEX_BUFFER,sizeof(int),numEdges)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_VERTEX_BUFFER)
+rtcNewBuffer(geom,RTC_VERTEX_BUFFER,4*sizeof(float),numVertices)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_LEVEL_BUFFER)
+rtcNewBuffer(geom,RTC_LEVEL_BUFFER,sizeof(int),numEdges)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_EDGE_CREASE_INDEX_BUFFER)
+rtcNewBuffer(geom,RTC_EDGE_CREASE_INDEX_BUFFER,2*sizeof(int),numEdgesCreases)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_EDGE_CREASE_WEIGHT_BUFFER)
+rtcNewBuffer(geom,RTC_EDGE_CREASE_WEIGHT_BUFFER,sizeof(float),numEdgesCreases)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_VERTEX_CREASE_INDEX_BUFFER)
+rtcNewBuffer(geom,RTC_VERTEX_CREASE_INDEX_BUFFER,sizeof(int),numVertexCreases)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_VERTEX_CREASE_WEIGHT_BUFFER)
+rtcNewBuffer(geom,RTC_VERTEX_CREASE_WEIGHT_BUFFER,sizeof(float),numVertexCreases)
...+>
}

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
{
...
RTCGeometry geom = rtcNewSubdivisionMesh(device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
<+...
-rtcMapBuffer(geom,RTC_HOLE_BUFFER)
+rtcNewBuffer(geom,RTC_HOLE_BUFFER,sizeof(int),numFaces)
...+>
}

@@
identifier geom;
expression device,basis,intersector,flags,numCurves,numVertices,numTimeSteps;
@@
{
...
RTCGeometry geom = rtcNewCurveGeometry (device,basis,intersector,flags,numCurves,numVertices,numTimeSteps);
<+...
-rtcMapBuffer(geom,RTC_INDEX_BUFFER)
+rtcNewBuffer(geom,RTC_INDEX_BUFFER,sizeof(int),numCurves)
...+>
}

@@
identifier geom;
expression device,basis,intersector,flags,numCurves,numVertices,numTimeSteps;
@@
{
...
RTCGeometry geom = rtcNewCurveGeometry (device,basis,intersector,flags,numCurves,numVertices,numTimeSteps);
<+...
-rtcMapBuffer(geom,RTC_VERTEX_BUFFER)
+rtcNewBuffer(geom,RTC_VERTEX_BUFFER,4*sizeof(float),numVertices)
...+>
}


@@
@@
-rtcUnmapBuffer(...);

@@
expression scene,geom,type;
@@
-rtcMapBuffer(scene,geom,type)
+rtcGetBuffer(geom,type)


////////////////////////////////////////////////
// drop extra arguments of geometry creation


@@
expression device,flags,numTriangles,numVertices,numTimeSteps;
@@
-rtcNewTriangleMesh (device,flags,numTriangles,numVertices,numTimeSteps)
+rtcNewTriangleMesh (device)

@@
expression device,flags,numQuads,numVertices,numTimeSteps;
@@
-rtcNewQuadMesh (device,flags,numQuads,numVertices,numTimeSteps)
+rtcNewQuadMesh (device)

@@
expression device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles;
@@
-rtcNewSubdivisionMesh (device,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles);
+rtcNewSubdivisionMesh (device);

