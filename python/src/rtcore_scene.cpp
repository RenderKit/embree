#include "pyembree.h"
#include "intersect_callback_guard.h"

namespace py = pybind11;

void bind_rtcore_scene(py::module &m) {
    /* Scene flags */
    py::enum_<RTCSceneFlags>(m, "RTCSceneFlags")
        .value("RTC_SCENE_FLAG_NONE", RTC_SCENE_FLAG_NONE)
        .value("RTC_SCENE_FLAG_DYNAMIC", RTC_SCENE_FLAG_DYNAMIC)
        .value("RTC_SCENE_FLAG_COMPACT", RTC_SCENE_FLAG_COMPACT)
        .value("RTC_SCENE_FLAG_ROBUST", RTC_SCENE_FLAG_ROBUST)
        .value("RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS", RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS)
        .export_values();

    /* Additional arguments for rtcIntersect1/4/8/16 calls */
    py::class_<PyRTCIntersectArguments>(m, "RTCIntersectArguments")
        .def(py::init<>())
        .def_readwrite("flags", &PyRTCIntersectArguments::flags)
        .def_readwrite("feature_mask", &PyRTCIntersectArguments::feature_mask)
        .def_readwrite("context", &PyRTCIntersectArguments::context)
        .def_readwrite("filter", &PyRTCIntersectArguments::filter)
        .def_readwrite("intersect", &PyRTCIntersectArguments::intersect)
        #if RTC_MIN_WIDTH
        .def_readwrite("minWidthDistanceFactor"), &PyRTCInterpolateArguments::minWidthDistanceFactor)
        #endif
        ;
    
    /* Initializes intersection arguments. */
    m.def("rtcInitIntersectArguments", &rtcInitIntersectArguments);
    
    /* Additional arguments for rtcOccluded1/4/8/16 calls */
    py::class_<RTCOccludedArguments>(m, "RTCOccludedArguments")
        .def_readwrite("flags", &RTCOccludedArguments::flags)
        .def_readwrite("feature_mask", &RTCOccludedArguments::feature_mask)
        .def_readwrite("context", &RTCOccludedArguments::context)
        //.def_readwrite("filter", &RTCOccludedArguments::filter)
        //.def_readwrite("occluded", &RTCOccludedArguments::occluded)
        #if RTC_MIN_WIDTH
        .def_readwrite("minWidthDistanceFactor", &RTCOccludedArguments::minWidthDistanceFactor)
        #endif
        ;

    /* Initializes an intersection arguments. */
    m.def("rtcInitOccludedArguments", &rtcInitOccludedArguments);

    /* Creates a new scene. */
    m.def("rtcNewScene", [](RTCDeviceWrapper device){ return RTCSceneWrapper{rtcNewScene(device.d)}; });

    /* Returns the device the scene got created in. The reference count of
     * the device is incremented by this function. */
    m.def("rtcGetSceneDevice", [](RTCSceneWrapper scene)->RTCDeviceWrapper { return RTCDeviceWrapper{rtcGetSceneDevice(scene.s)}; });
    
    /* Retains the scene (increments the reference count). */
    m.def("rtcRetainScene", [](RTCSceneWrapper scene) {rtcRetainScene(scene.s);});

    /* Releases the scene (decrements the reference count). */
    m.def("rtcReleaseScene", [](RTCSceneWrapper scene){rtcReleaseScene(scene.s);});

    /* Attaches the geometry to a scene. */
    m.def("rtcAttachGeometry", [](RTCSceneWrapper scene, RTCGeometryWrapper geometry){ return rtcAttachGeometry(scene.s, geometry.g); });

    /* Attaches the geometry to a scene using the specified geometry ID. */
    m.def("rtcAttachGeometryByID", [](RTCSceneWrapper scene, RTCGeometryWrapper geometry, unsigned int geomID){rtcAttachGeometryByID(scene.s, geometry.g, geomID);});

    /* Detaches the geometry from the scene. */
    m.def("rtcDetachGeometry", [](RTCSceneWrapper scene, unsigned int geomID){rtcDetachGeometry(scene.s, geomID);});

    /* Gets a geometry handle from the scene. This function is not thread safe and should get used during rendering. */
    m.def("rtcGetGeometry", [](RTCSceneWrapper scene, unsigned int geomID)->RTCGeometryWrapper { return RTCGeometryWrapper{rtcGetGeometry(scene.s, geomID)}; });

    /* Gets a geometry handle from the scene. This function is thread safe and should NOT get used during rendering. */
    m.def("rtcGetGeometryThreadSafe", [](RTCSceneWrapper scene, unsigned int geomID)->RTCGeometryWrapper { return RTCGeometryWrapper{rtcGetGeometryThreadSafe(scene.s, geomID)}; });

    /* Gets the user-defined data pointer of the geometry. This function is not thread safe and should get used during rendering. */
    m.def("rtcGetGeometryUserDataFromScene", [](RTCSceneWrapper scene, unsigned int geomID) { 
        return EmbreeVoidPtr(rtcGetGeometryUserDataFromScene(scene.s, geomID));
    });

    /* Returns the interpolated transformation of an instance for the specified time. */
    m.def("rtcGetGeometryTransformFromScene", [](RTCSceneWrapper scene, unsigned int geomID, float time, enum RTCFormat format, void* xfm) {rtcGetGeometryTransformFromScene(scene.s, geomID, time, format, xfm);});

    /* Commits the scene. */
    m.def("rtcCommitScene", [](RTCSceneWrapper scene){rtcCommitScene(scene.s);});

    /* Commits the scene from multiple threads. */
    m.def("rtcJoinCommitScene", [](RTCSceneWrapper scene){rtcJoinCommitScene(scene.s);});


    /* Sets the progress monitor callback function of the scene. */
    m.def("rtcSetSceneProgressMonitorFunction", [](RTCSceneWrapper scene, RTCProgressMonitorFunction progress, void* ptr){rtcSetSceneProgressMonitorFunction(scene.s, progress, ptr);});

    /* Sets the build quality of the scene. */
    m.def("rtcSetSceneBuildQuality", [](RTCSceneWrapper scene, enum RTCBuildQuality quality){rtcSetSceneBuildQuality(scene.s, quality);});

    /* Sets the scene flags. */
    m.def("rtcSetSceneFlags", [](RTCSceneWrapper scene, enum RTCSceneFlags flags){rtcSetSceneFlags(scene.s, flags);});

    /* Returns the scene flags. */
    m.def("rtcGetSceneFlags", [](RTCSceneWrapper scene){ return rtcGetSceneFlags(scene.s); });
    
    /* Returns the axis-aligned bounds of the scene. */
    m.def("rtcGetSceneBounds", [](RTCSceneWrapper scene, struct RTCBounds* bounds_o){rtcGetSceneBounds(scene.s, bounds_o); });

    /* Returns the linear axis-aligned bounds of the scene. */
    m.def("rtcGetSceneLinearBounds", [](RTCSceneWrapper scene, struct RTCLinearBounds* bounds_o){rtcGetSceneLinearBounds(scene.s, bounds_o);});


    /* Perform a closest point query of the scene. */
    m.def("rtcPointQuery", [](RTCSceneWrapper scene, struct RTCPointQuery* query, struct RTCPointQueryContext* context, RTCPointQueryFunction queryFunc, void* userPtr){
        return rtcPointQuery(scene.s, query, context, queryFunc, userPtr);
    });
    
    /* Perform a closest point query with a packet of 4 points with the scene. */
    //m.def("rtcPointQuery4", [](const int* valid, RTCSceneWrapper scene, struct RTCPointQuery4* query, struct RTCPointQueryContext* context, RTCPointQueryFunction queryFunc, void** userPtr){return rtcPointQuery4(valid, scene.s, query, context, queryFunc, userPtr);});
    
    /* Perform a closest point query with a packet of 4 points with the scene. */
    //m.def("rtcPointQuery8", [](const int* valid, RTCSceneWrapper scene, struct RTCPointQuery8* query, struct RTCPointQueryContext* context, RTCPointQueryFunction queryFunc, void** userPtr){return rtcPointQuery8(valid, scene.s, query, context, queryFunc, userPtr);});
    
    /* Perform a closest point query with a packet of 4 points with the scene. */
    //m.def("rtcPointQuery16", [](const int* valid, RTCSceneWrapper scene, struct RTCPointQuery16* query, struct RTCPointQueryContext* context, RTCPointQueryFunction queryFunc, void** userPtr){return rtcPointQuery16(valid, scene.s, query, context, queryFunc, userPtr);});
    
    
    /* Intersects a single ray with the scene. */
    m.def("rtcIntersect1", [](RTCSceneWrapper scene, struct RTCRayHit* rayhit, struct PyRTCIntersectArguments* args RTC_OPTIONAL_ARGUMENT){
        auto pargs = FilterFunctionGuard::get().begin_intersect(scene.s, args);
        rtcIntersect1(scene.s, rayhit, pargs.get());
        FilterFunctionGuard::get().end_intersect();
    });

    /* Intersects a packet of 4 rays with the scene. */
    m.def("rtcIntersect4", [](py::array_t<int> valid, RTCSceneWrapper scene, struct RTCRayHit4* rayhit, struct PyRTCIntersectArguments* args RTC_OPTIONAL_ARGUMENT){
        int* valid_ptr = (int*)valid.request().ptr;
        auto pargs = FilterFunctionGuard::get().begin_intersect(scene.s, args);
        rtcIntersect4(valid_ptr, scene.s, rayhit, pargs.get());
        FilterFunctionGuard::get().end_intersect();
    });

    /* Intersects a packet of 8 rays with the scene. */
    m.def("rtcIntersect8", [](py::array_t<int> valid, RTCSceneWrapper scene, struct RTCRayHit8* rayhit, struct PyRTCIntersectArguments* args RTC_OPTIONAL_ARGUMENT){
        int* valid_ptr = (int*)valid.request().ptr;
        auto pargs = FilterFunctionGuard::get().begin_intersect(scene.s, args);
        rtcIntersect8(valid_ptr, scene.s, rayhit, pargs.get());
        FilterFunctionGuard::get().end_intersect();
    });

    /* Intersects a packet of 16 rays with the scene. */
    m.def("rtcIntersect16", [](py::array_t<int> valid, RTCSceneWrapper scene, struct RTCRayHit16* rayhit, struct PyRTCIntersectArguments* args RTC_OPTIONAL_ARGUMENT){
        int* valid_ptr = (int*)valid.request().ptr;
        auto pargs = FilterFunctionGuard::get().begin_intersect(scene.s, args);
        rtcIntersect16(valid_ptr, scene.s, rayhit, pargs.get());
        FilterFunctionGuard::get().end_intersect();
    });

    /* Intersects multiple rays with the scene. */
    m.def("rtcIntersectN", [](RTCSceneWrapper scene, py::list rayhits, struct PyRTCIntersectArguments* args RTC_OPTIONAL_ARGUMENT, size_t nThreads = 0){
        if (nThreads == 0) nThreads = std::thread::hardware_concurrency();

        auto exe_async = [](RTCScene scene, py::list rayhits, struct RTCIntersectArguments* args, size_t nThreads) {
            std::vector<std::thread> threads;
            for (unsigned int i = 0; i < nThreads; i++) {
                auto t = std::thread([&scene, i, &rayhits, args, nThreads]() {
                    auto b = rayhits.begin();
                    auto e = b + (rayhits.size() - (rayhits.size() % nThreads)) / nThreads;
                    if (i == nThreads - 1) e = rayhits.end();
                    for (auto it = b; it != e; ++it) {
                        RTCRayHit* rh = it->cast<RTCRayHit*>();
                        rtcIntersect1(scene, rh, args);
                    }
                });
                threads.push_back(std::move(t));
            }

            for (auto &t : threads) {
                t.join();
            }
        };

        auto pargs = FilterFunctionGuard::get().begin_intersect(scene.s, args);
        exe_async(scene.s, rayhits, pargs.get(), nThreads);
        FilterFunctionGuard::get().end_intersect();
    });

    /* Forwards ray inside user geometry callback. */
    m.def("rtcForwardIntersect1", [](const struct RTCIntersectFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay* ray, unsigned int instID){
        rtcForwardIntersect1(args, scene.s, ray, instID);
    });

    /* Forwards ray packet of size 4 inside user geometry callback. */
    m.def("rtcForwardIntersect4", [](const int* valid, const struct RTCIntersectFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay4* ray, unsigned int instID){
        rtcForwardIntersect4(valid, args, scene.s, ray, instID);
    });

    /* Forwards ray packet of size 8 inside user geometry callback. */
    m.def("rtcForwardIntersect8", [](const int* valid, const struct RTCIntersectFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay8* ray, unsigned int instID){
        rtcForwardIntersect8(valid, args, scene.s, ray, instID);
    });

    /* Forwards ray packet of size 16 inside user geometry callback. */
    m.def("rtcForwardIntersect16", [](const int* valid, const struct RTCIntersectFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay16* ray, unsigned int instID){
        rtcForwardIntersect16(valid, args, scene.s, ray, instID);
    });


    /* Tests a single ray for occlusion with the scene. */
    m.def("rtcOccluded1", [](RTCSceneWrapper scene, struct RTCRay* ray, struct RTCOccludedArguments* args RTC_OPTIONAL_ARGUMENT){
        rtcOccluded1(scene.s, ray, args);
    });

    /* Tests a packet of 4 rays for occlusion occluded with the scene. */
    m.def("rtcOccluded4", [](const int* valid, RTCSceneWrapper scene, struct RTCRay4* ray, struct RTCOccludedArguments* args RTC_OPTIONAL_ARGUMENT){
        rtcOccluded4(valid, scene.s, ray, args);
    });

    /* Tests a packet of 8 rays for occlusion with the scene. */
    m.def("rtcOccluded8", [](const int* valid, RTCSceneWrapper scene, struct RTCRay8* ray, struct RTCOccludedArguments* args RTC_OPTIONAL_ARGUMENT){
        rtcOccluded8(valid, scene.s, ray, args);
    });

    /* Tests a packet of 16 rays for occlusion with the scene. */
    m.def("rtcOccluded16", [](const int* valid, RTCSceneWrapper scene, struct RTCRay16* ray, struct RTCOccludedArguments* args RTC_OPTIONAL_ARGUMENT){
        rtcOccluded16(valid, scene.s, ray, args);
    });


    /* Forwards single occlusion ray inside user geometry callback. */
    m.def("rtcForwardOccluded1", [](const struct RTCOccludedFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay* ray, unsigned int instID){
        rtcForwardOccluded1(args, scene.s, ray, instID);
    });

    /* Forwards occlusion ray packet of size 4 inside user geometry callback. */
    m.def("rtcForwardOccluded4", [](const int* valid, const struct RTCOccludedFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay4* ray, unsigned int instID){
        rtcForwardOccluded4(valid, args, scene.s, ray, instID);
    });

    /* Forwards occlusion ray packet of size 8 inside user geometry callback. */
    m.def("rtcForwardOccluded8", [](const int* valid, const struct RTCOccludedFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay8* ray, unsigned int instID){
        rtcForwardOccluded8(valid, args, scene.s, ray, instID);
    });

    /* Forwards occlusion ray packet of size 16 inside user geometry callback. */
    m.def("rtcForwardOccluded16", [](const int* valid, const struct RTCOccludedFunctionNArguments* args, RTCSceneWrapper scene, struct RTCRay16* ray, unsigned int instID){
        rtcForwardOccluded16(valid, args, scene.s, ray, instID);
    });


    /*! collision callback */
    py::class_<RTCCollision>(m, "RTCCollision")
        .def_readwrite("geomID0", &RTCCollision::geomID0)
        .def_readwrite("primID0", &RTCCollision::primID0)
        .def_readwrite("geomID1", &RTCCollision::geomID1)
        .def_readwrite("primID1", &RTCCollision::primID1);

    /*! Performs collision detection of two scenes */
    m.def("rtcCollide", [](RTCSceneWrapper scene0, RTCSceneWrapper scene1, RTCCollideFunc callback, void* userPtr){rtcCollide(scene0.s, scene1.s, callback, userPtr);});

}