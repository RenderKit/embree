#include "pyembree.h"
#include "intersect_callback_guard.h"

namespace py = pybind11;


void bind_rtcore_geometry(py::module &m) {
    py::class_<RTCSceneWrapper>(m, "RTCSceneWrapper");
    py::class_<RTCGeometryWrapper>(m, "RTCGeometryWrapper");

    py::enum_<RTCGeometryType>(m, "RTCGeometryType")
        .value("RTC_GEOMETRY_TYPE_TRIANGLE", RTC_GEOMETRY_TYPE_TRIANGLE)
        .value("RTC_GEOMETRY_TYPE_QUAD", RTC_GEOMETRY_TYPE_QUAD)
        .value("RTC_GEOMETRY_TYPE_GRID", RTC_GEOMETRY_TYPE_GRID)

        .value("RTC_GEOMETRY_TYPE_SUBDIVISION", RTC_GEOMETRY_TYPE_SUBDIVISION)

        .value("RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE", RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE)
        .value("RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE", RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE", RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE)

        .value("RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE", RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE", RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE)
        .value("RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE)
  
        .value("RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE", RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE", RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE)
        .value("RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE)

        .value("RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE", RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE", RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE)
        .value("RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE)

        .value("RTC_GEOMETRY_TYPE_SPHERE_POINT", RTC_GEOMETRY_TYPE_SPHERE_POINT)
        .value("RTC_GEOMETRY_TYPE_DISC_POINT", RTC_GEOMETRY_TYPE_DISC_POINT)
        .value("RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT", RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT)

        .value("RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE", RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE", RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE)
        .value("RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE)

        .value("RTC_GEOMETRY_TYPE_USER", RTC_GEOMETRY_TYPE_USER)
        .value("RTC_GEOMETRY_TYPE_INSTANCE", RTC_GEOMETRY_TYPE_INSTANCE)
        .export_values();

    /* Interpolation modes for subdivision surfaces */
    py::enum_<RTCSubdivisionMode>(m, "RTCSubdivisionMode")
        .value("RTC_SUBDIVISION_MODE_NO_BOUNDARY", RTC_SUBDIVISION_MODE_NO_BOUNDARY)
        .value("RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY", RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY)
        .value("RTC_SUBDIVISION_MODE_PIN_CORNERS", RTC_SUBDIVISION_MODE_PIN_CORNERS)
        .value("RTC_SUBDIVISION_MODE_PIN_BOUNDARY", RTC_SUBDIVISION_MODE_PIN_BOUNDARY)
        .value("RTC_SUBDIVISION_MODE_PIN_ALL", RTC_SUBDIVISION_MODE_PIN_ALL)
        .export_values();

    /* Curve segment flags */
    py::enum_<RTCCurveFlags>(m, "RTCCurveFlags")
        .value("RTC_CURVE_FLAG_NEIGHBOR_LEFT", RTC_CURVE_FLAG_NEIGHBOR_LEFT)
        .value("RTC_CURVE_FLAG_NEIGHBOR_RIGHT", RTC_CURVE_FLAG_NEIGHBOR_RIGHT)
        .export_values();

    /* Arguments for RTCBoundsFunction */
    py::class_<RTCBoundsFunctionArguments>(m, "RTCBoundsFunctionArguments")
        .def_readwrite("geometryUserPtr", &RTCBoundsFunctionArguments::geometryUserPtr)
        .def_readwrite("primID", &RTCBoundsFunctionArguments::primID)
        .def_readwrite("timeStep", &RTCBoundsFunctionArguments::timeStep)
        .def_readwrite("bounds_o", &RTCBoundsFunctionArguments::bounds_o);

    /* Arguments for RTCIntersectFunctionN */
    py::class_<RTCIntersectFunctionNArguments>(m, "RTCIntersectFunctionNArguments")
        .def_readwrite("valid", &RTCIntersectFunctionNArguments::valid)
        .def_readwrite("geometryUserPtr", &RTCIntersectFunctionNArguments::geometryUserPtr)
        .def_readwrite("primID", &RTCIntersectFunctionNArguments::primID)
        .def_readwrite("context", &RTCIntersectFunctionNArguments::context)
        .def_property("rayhit", [](RTCIntersectFunctionNArguments& c)->RTCRayHitNWrapper { return {c.rayhit};}, [](RTCIntersectFunctionNArguments& c, RTCRayHitNWrapper rh){ c.rayhit = rh.p; })
        .def_readwrite("N", &RTCIntersectFunctionNArguments::N)
        .def_readwrite("geomID", &RTCIntersectFunctionNArguments::geomID);

    /* Arguments for RTCOccludedFunctionN */
    py::class_<RTCOccludedFunctionNArguments>(m, "RTCOccludedFunctionNArguments")
        .def_readwrite("valid", &RTCOccludedFunctionNArguments::valid)
        .def_readwrite("geometryUserPtr", &RTCOccludedFunctionNArguments::geometryUserPtr)
        .def_readwrite("primID", &RTCOccludedFunctionNArguments::primID)
        .def_readwrite("context", &RTCOccludedFunctionNArguments::context)
        .def_property("ray", [](RTCOccludedFunctionNArguments& c)->RTCRayNWrapper { return {c.ray};}, [](RTCOccludedFunctionNArguments& c, RTCRayNWrapper r){ c.ray = r.p; })
        .def_readwrite("N", &RTCOccludedFunctionNArguments::N)
        .def_readwrite("geomID", &RTCOccludedFunctionNArguments::geomID);

    /* Arguments for RTCDisplacementFunctionN */
    py::class_<RTCDisplacementFunctionNArguments>(m, "RTCDisplacementFunctionNArguments")
        .def_readwrite("geometryUserPtr", &RTCDisplacementFunctionNArguments::geometryUserPtr)
        .def_property("geometry", [](RTCDisplacementFunctionNArguments& c)->RTCGeometryWrapper { return {c.geometry};}, [](RTCDisplacementFunctionNArguments& c, RTCGeometryWrapper g){ c.geometry = g.g; })
        .def_readwrite("primID", &RTCDisplacementFunctionNArguments::primID)
        .def_readwrite("timeStep", &RTCDisplacementFunctionNArguments::timeStep)
        .def_readwrite("u", &RTCDisplacementFunctionNArguments::u)
        .def_readwrite("v", &RTCDisplacementFunctionNArguments::v)
        .def_readwrite("Ng_x", &RTCDisplacementFunctionNArguments::Ng_x)
        .def_readwrite("Ng_y", &RTCDisplacementFunctionNArguments::Ng_y)
        .def_readwrite("Ng_z", &RTCDisplacementFunctionNArguments::Ng_z)
        .def_readwrite("P_x", &RTCDisplacementFunctionNArguments::P_x)
        .def_readwrite("P_y", &RTCDisplacementFunctionNArguments::P_y)
        .def_readwrite("P_z", &RTCDisplacementFunctionNArguments::P_z)
        .def_readwrite("N", &RTCDisplacementFunctionNArguments::N);

    /* Creates a new geometry of specified type. */
    m.def("rtcNewGeometry", [](RTCDeviceWrapper device, enum RTCGeometryType type){ return RTCGeometryWrapper{rtcNewGeometry(device.d, type)}; });
    /* Retains the geometry (increments the reference count). */
    m.def("rtcRetainGeometry", [](RTCGeometryWrapper geometry){rtcRetainGeometry(geometry.g);});
    /* Releases the geometry (decrements the reference count) */
    m.def("rtcReleaseGeometry", [](RTCGeometryWrapper geometry){rtcReleaseGeometry(geometry.g);});
    /* Commits the geometry. */
    m.def("rtcCommitGeometry", [](RTCGeometryWrapper geometry){rtcCommitGeometry(geometry.g);});

    /* Enables the geometry. */
    m.def("rtcEnableGeometry", [](RTCGeometryWrapper geometry){rtcEnableGeometry(geometry.g);});
    /* Disables the geometry. */
    m.def("rtcDisableGeometry", [](RTCGeometryWrapper geometry){rtcDisableGeometry(geometry.g);});


    /* Sets the number of motion blur time steps of the geometry. */
    m.def("rtcSetGeometryTimeStepCount", [](RTCGeometryWrapper geometry, unsigned int timeStepCount){rtcSetGeometryTimeStepCount(geometry.g, timeStepCount);});

    /* Sets the motion blur time range of the geometry. */
    m.def("rtcSetGeometryTimeRange", [](RTCGeometryWrapper geometry, float startTime, float endTime){rtcSetGeometryTimeRange(geometry.g, startTime, endTime);});
    
    /* Sets the number of vertex attributes of the geometry. */
    m.def("rtcSetGeometryVertexAttributeCount", [](RTCGeometryWrapper geometry, unsigned int vertexAttributeCount){rtcSetGeometryVertexAttributeCount(geometry.g, vertexAttributeCount);});

    /* Sets the ray mask of the geometry. */
    m.def("rtcSetGeometryMask", [](RTCGeometryWrapper geometry, unsigned int mask){rtcSetGeometryMask(geometry.g, mask);});

    /* Sets the build quality of the geometry. */
    m.def("rtcSetGeometryBuildQuality", [](RTCGeometryWrapper geometry, enum RTCBuildQuality quality){rtcSetGeometryBuildQuality(geometry.g, quality);});

    /* Sets the maximal curve or point radius scale allowed by min-width feature. */
    m.def("rtcSetGeometryMaxRadiusScale", [](RTCGeometryWrapper geometry, float maxRadiusScale){rtcSetGeometryMaxRadiusScale(geometry.g, maxRadiusScale);});


    /* Sets a geometry buffer. */
    m.def("rtcSetGeometryBuffer", [](RTCGeometryWrapper geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format, RTCBufferWrapper buffer, size_t byteOffset, size_t byteStride, size_t itemCount){rtcSetGeometryBuffer(geometry.g, type, slot, format, buffer.b, byteOffset, byteStride, itemCount);});

    /* Sets a shared geometry buffer. */
    m.def("rtcSetSharedGeometryBuffer", [](RTCGeometryWrapper geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format, const void* ptr, size_t byteOffset, size_t byteStride, size_t itemCount){rtcSetSharedGeometryBuffer(geometry.g, type, slot, format, ptr, byteOffset, byteStride, itemCount);});

    /* Creates and sets a new geometry buffer. */
    m.def("rtcSetNewGeometryBuffer", [](RTCGeometryWrapper geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format, size_t byteStride, size_t itemCount){
        return EmbreeVoidPtr(rtcSetNewGeometryBuffer(geometry.g, type, slot, format, byteStride, itemCount));
    });

    /* Creates and sets a new geometry buffer. */
    m.def("rtcSetNewSharedGeometryBuffer", [](RTCGeometryWrapper geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format, size_t byteStride, size_t itemCount){
        // TODO
        return EmbreeVoidPtr(rtcSetNewGeometryBuffer(geometry.g, type, slot, format, byteStride, itemCount));
    });

    /* Returns the pointer to the data of a buffer. */
    m.def("rtcGetGeometryBufferData", [](RTCGeometryWrapper geometry, enum RTCBufferType type, unsigned int slot){
        return EmbreeVoidPtr(rtcGetGeometryBufferData(geometry.g, type, slot));
    });

    /* Updates a geometry buffer. */
    m.def("rtcUpdateGeometryBuffer", [](RTCGeometryWrapper geometry, enum RTCBufferType type, unsigned int slot){rtcUpdateGeometryBuffer(geometry.g, type, slot);});


    /* Sets the intersection filter callback function of the geometry. */
    m.def("rtcSetGeometryIntersectFilterFunction", [](RTCGeometryWrapper geometry, PyRTCFilterFunctionN filter){
        if (filter) {
            FilterFunctionGuard::get().assign(filter, geometry.g);
            rtcSetGeometryIntersectFilterFunction(geometry.g, pyembree_rtcfilter_functionn_callback);
        }
        else {
            FilterFunctionGuard::get().clear(geometry.g);
            rtcSetGeometryIntersectFilterFunction(geometry.g, nullptr);
        }
    });
    
    /* Sets the occlusion filter callback function of the geometry. */
    m.def("rtcSetGeometryOccludedFilterFunction", [](RTCGeometryWrapper geometry, RTCFilterFunctionN filter){rtcSetGeometryOccludedFilterFunction(geometry.g, filter);});
    
    /* Enables argument version of intersection or occlusion filter function. */
    m.def("rtcSetGeometryEnableFilterFunctionFromArguments", [](RTCGeometryWrapper geometry, bool enable){rtcSetGeometryEnableFilterFunctionFromArguments(geometry.g, enable);});
    
    /* Sets the user-defined data pointer of the geometry. */
    m.def("rtcSetGeometryUserData", [](RTCGeometryWrapper geometry, void* ptr){rtcSetGeometryUserData(geometry.g, ptr);});
    
    /* Gets the user-defined data pointer of the geometry. */
    m.def("rtcGetGeometryUserData", [](RTCGeometryWrapper geometry){
        return EmbreeVoidPtr(rtcGetGeometryUserData(geometry.g));
    });

    /* Set the point query callback function of a geometry. */
    m.def("rtcSetGeometryPointQueryFunction", [](RTCGeometryWrapper geometry, RTCPointQueryFunction pointQuery){rtcSetGeometryPointQueryFunction(geometry.g, pointQuery);});

    /* Sets the number of primitives of a user geometry. */
    m.def("rtcSetGeometryUserPrimitiveCount", [](RTCGeometryWrapper geometry, unsigned int userPrimitiveCount){rtcSetGeometryUserPrimitiveCount(geometry.g, userPrimitiveCount);});

    /* Sets the bounding callback function to calculate bounding boxes for user primitives. */
    m.def("rtcSetGeometryBoundsFunction", [](RTCGeometryWrapper geometry, RTCBoundsFunction bounds, void* userPtr){rtcSetGeometryBoundsFunction(geometry.g, bounds, userPtr);});

    /* Set the intersect callback function of a user geometry. */
    m.def("rtcSetGeometryIntersectFunction", [](RTCGeometryWrapper geometry, RTCIntersectFunctionN intersect){rtcSetGeometryIntersectFunction(geometry.g, intersect);});

    /* Set the occlusion callback function of a user geometry. */
    m.def("rtcSetGeometryOccludedFunction", [](RTCGeometryWrapper geometry, RTCOccludedFunctionN occluded){rtcSetGeometryOccludedFunction(geometry.g, occluded);});

    /* Invokes the intersection filter from the intersection callback function. */
    m.def("rtcInvokeIntersectFilterFromGeometry", [](const struct RTCIntersectFunctionNArguments* args, const struct RTCFilterFunctionNArguments* filterArgs){rtcInvokeIntersectFilterFromGeometry(args, filterArgs);});

    /* Invokes the occlusion filter from the occlusion callback function. */
    m.def("rtcInvokeOccludedFilterFromGeometry", [](const struct RTCOccludedFunctionNArguments* args, const struct RTCFilterFunctionNArguments* filterArgs){rtcInvokeOccludedFilterFromGeometry(args, filterArgs);});

    /* Sets the instanced scene of an instance geometry. */
    m.def("rtcSetGeometryInstancedScene", [](RTCGeometryWrapper geometry, RTCSceneWrapper scene){rtcSetGeometryInstancedScene(geometry.g, scene.s);});

    /* Sets the transformation of an instance for the specified time step. */
    m.def("rtcSetGeometryTransform", [](RTCGeometryWrapper geometry, unsigned int timeStep, enum RTCFormat format, const void* xfm){rtcSetGeometryTransform(geometry.g, timeStep, format, xfm);});

    /* Sets the transformation quaternion of an instance for the specified time step. */
    m.def("rtcSetGeometryTransformQuaternion", [](RTCGeometryWrapper geometry, unsigned int timeStep, const struct RTCQuaternionDecomposition* qd){rtcSetGeometryTransformQuaternion(geometry.g, timeStep, qd);});

    /* Returns the interpolated transformation of an instance for the specified time. */
    m.def("rtcGetGeometryTransform", [](RTCGeometryWrapper geometry, float time, enum RTCFormat format, void* xfm){rtcGetGeometryTransform(geometry.g, time, format, xfm);});


    /* Sets the uniform tessellation rate of the geometry. */
    m.def("rtcSetGeometryTessellationRate", [](RTCGeometryWrapper geometry, float tessellationRate){rtcSetGeometryTessellationRate(geometry.g, tessellationRate);});

    /* Sets the number of topologies of a subdivision surface. */
    m.def("rtcSetGeometryTopologyCount", [](RTCGeometryWrapper geometry, unsigned int topologyCount){rtcSetGeometryTopologyCount(geometry.g, topologyCount);});

    /* Sets the subdivision interpolation mode. */
    m.def("rtcSetGeometrySubdivisionMode", [](RTCGeometryWrapper geometry, unsigned int topologyID, enum RTCSubdivisionMode mode){rtcSetGeometrySubdivisionMode(geometry.g, topologyID, mode);});

    /* Binds a vertex attribute to a topology of the geometry. */
    m.def("rtcSetGeometryVertexAttributeTopology", [](RTCGeometryWrapper geometry, unsigned int vertexAttributeID, unsigned int topologyID){rtcSetGeometryVertexAttributeTopology(geometry.g, vertexAttributeID, topologyID);});

    /* Sets the displacement callback function of a subdivision surface. */
    m.def("rtcSetGeometryDisplacementFunction", [](RTCGeometryWrapper geometry, RTCDisplacementFunctionN displacement){rtcSetGeometryDisplacementFunction(geometry.g, displacement);});


    /* Returns the first half edge of a face. */
    m.def("rtcGetGeometryFirstHalfEdge", [](RTCGeometryWrapper geometry, unsigned int faceID){ return rtcGetGeometryFirstHalfEdge(geometry.g, faceID); });

    /* Returns the face the half edge belongs to. */
    m.def("rtcGetGeometryFace", [](RTCGeometryWrapper geometry, unsigned int faceID){return rtcGetGeometryFace(geometry.g, faceID); });

    /* Returns next half edge. */
    m.def("rtcGetGeometryNextHalfEdge", [](RTCGeometryWrapper geometry, unsigned int edgeID){return rtcGetGeometryNextHalfEdge(geometry.g, edgeID); });

    /* Returns previous half edge. */
    m.def("rtcGetGeometryPreviousHalfEdge", [](RTCGeometryWrapper geometry, unsigned int edgeID){return rtcGetGeometryPreviousHalfEdge(geometry.g, edgeID); });

    /* Returns opposite half edge. */
    m.def("rtcGetGeometryOppositeHalfEdge", [](RTCGeometryWrapper geometry, unsigned int topologyID, unsigned int edgeID){return rtcGetGeometryOppositeHalfEdge(geometry.g, topologyID, edgeID); });


    /* Arguments for rtcInterpolate */
    py::class_<RTCInterpolateArguments>(m, "RTCInterpolateArguments")
        .def(py::init<>())
        .def_property("geometry", [](RTCInterpolateArguments& c)->RTCGeometryWrapper { return {c.geometry};}, [](RTCInterpolateArguments& c, RTCGeometryWrapper g){ c.geometry = g.g; })
        .def_readwrite("primID", &RTCInterpolateArguments::primID)
        .def_readwrite("u", &RTCInterpolateArguments::u)
        .def_readwrite("v", &RTCInterpolateArguments::v)
        .def_readwrite("bufferType", &RTCInterpolateArguments::bufferType)
        .def_readwrite("bufferSlot", &RTCInterpolateArguments::bufferSlot)
        .def_readwrite("P", &RTCInterpolateArguments::P)
        .def_readwrite("dPdu", &RTCInterpolateArguments::dPdu)
        .def_readwrite("dPdv", &RTCInterpolateArguments::dPdv)
        .def_readwrite("ddPdudu", &RTCInterpolateArguments::ddPdudu)
        .def_readwrite("ddPdvdv", &RTCInterpolateArguments::ddPdvdv)
        .def_readwrite("ddPdudv", &RTCInterpolateArguments::ddPdudv)
        .def_readwrite("valueCount", &RTCInterpolateArguments::valueCount);

    /* Interpolates vertex data to some u/v location and optionally calculates all derivatives. */
    m.def("rtcInterpolate", &rtcInterpolate);

    /* Interpolates vertex data to some u/v location. */
    m.def("rtcInterpolate0", [](RTCGeometryWrapper geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot, float* P, unsigned int valueCount){rtcInterpolate0(geometry.g, primID, u, v, bufferType, bufferSlot, P, valueCount);});

    /* Interpolates vertex data to some u/v location and calculates first order derivatives. */
    m.def("rtcInterpolate1", [](RTCGeometryWrapper geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot, float* P, float* dPdu, float* dPdv, unsigned int valueCount){
        rtcInterpolate1(geometry.g, primID, u, v, bufferType, bufferSlot, P, dPdu, dPdv, valueCount);
        });

    /* Interpolates vertex data to some u/v location and calculates first and second order derivatives. */
    m.def("rtcInterpolate2", [](RTCGeometryWrapper geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, unsigned int valueCount){
        rtcInterpolate2(geometry.g, primID, u, v, bufferType, bufferSlot, P, dPdu, dPdv, ddPdudu, ddPdvdv, ddPdudv, valueCount);
        });

    /* Arguments for rtcInterpolateN */
    py::class_<RTCInterpolateNArguments>(m, "RTCInterpolateNArguments")
        .def(py::init<>())
        .def_property("geometry", [](RTCInterpolateNArguments& c)->RTCGeometryWrapper { return {c.geometry};}, [](RTCInterpolateNArguments& c, RTCGeometryWrapper g){ c.geometry = g.g; })
        .def_readwrite("valid", &RTCInterpolateNArguments::valid)
        .def_readwrite("primIDs", &RTCInterpolateNArguments::primIDs)
        .def_readwrite("u", &RTCInterpolateNArguments::u)
        .def_readwrite("v", &RTCInterpolateNArguments::v)
        .def_readwrite("N", &RTCInterpolateNArguments::N)
        .def_readwrite("bufferType", &RTCInterpolateNArguments::bufferType)
        .def_readwrite("bufferSlot", &RTCInterpolateNArguments::bufferSlot)
        .def_readwrite("P", &RTCInterpolateNArguments::P)
        .def_readwrite("dPdu", &RTCInterpolateNArguments::dPdu)
        .def_readwrite("dPdv", &RTCInterpolateNArguments::dPdv)
        .def_readwrite("ddPdudu", &RTCInterpolateNArguments::ddPdudu)
        .def_readwrite("ddPdvdv", &RTCInterpolateNArguments::ddPdvdv)
        .def_readwrite("ddPdudv", &RTCInterpolateNArguments::ddPdudv)
        .def_readwrite("valueCount", &RTCInterpolateNArguments::valueCount);

    /* Interpolates vertex data to an array of u/v locations. */
    m.def("rtcInterpolateN", &rtcInterpolateN);
    
    /* RTCGrid primitive for grid mesh */
    py::class_<RTCGrid>(m, "RTCGrid")
        .def_readwrite("startVertexID", &RTCGrid::startVertexID)
        .def_readwrite("stride", &RTCGrid::stride)
        .def_readwrite("width", &RTCGrid::width)
        .def_readwrite("height", &RTCGrid::height);

}
