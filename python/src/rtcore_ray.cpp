#include "pyembree.h"

namespace py = pybind11;


void bind_rtcore_ray(pybind11::module &m) {
    py::class_<RTCRay>(m, "RTCRay")
        .def(py::init<>())
        .def_readwrite("org_x", &RTCRay::org_x)
        .def_readwrite("org_y", &RTCRay::org_y)
        .def_readwrite("org_z", &RTCRay::org_z)
        .def_readwrite("tnear", &RTCRay::tnear)
        .def_readwrite("dir_x", &RTCRay::dir_x)
        .def_readwrite("dir_y", &RTCRay::dir_y)
        .def_readwrite("dir_z", &RTCRay::dir_z)
        .def_readwrite("time", &RTCRay::time)
        .def_readwrite("tfar", &RTCRay::tfar)
        .def_readwrite("mask", &RTCRay::mask)
        .def_readwrite("id", &RTCRay::id)
        .def_readwrite("flags", &RTCRay::flags);

    PYBIND11_NUMPY_DTYPE(RTCRay, org_x, org_y, org_z, tnear, dir_x, dir_y, dir_z, time, tfar, mask, id, flags);
    
    py::class_<RTCHit>(m, "RTCHit")
        .def(py::init<>())
        .def_readwrite("Ng_x", &RTCHit::Ng_x)
        .def_readwrite("Ng_y", &RTCHit::Ng_y)
        .def_readwrite("Ng_z", &RTCHit::Ng_z)
        .def_readwrite("u", &RTCHit::u)
        .def_readwrite("v", &RTCHit::v)
        .def_readwrite("primID", &RTCHit::primID)
        .def_readwrite("geomID", &RTCHit::geomID)
        .def_property("instID", [](RTCHit &h)->pybind11::array {
            auto dtype = pybind11::dtype(pybind11::format_descriptor<unsigned int>::format());
            return pybind11::array(dtype, { RTC_MAX_INSTANCE_LEVEL_COUNT }, { sizeof(unsigned int) }, h.instID, nullptr);
            }, [](RTCHit& h) {});

    PYBIND11_NUMPY_DTYPE(RTCHit, Ng_x, Ng_y, Ng_z, u, v, primID, geomID, instID);

    py::class_<RTCRayHit>(m, "RTCRayHit")
        .def(py::init<>())
        .def_readwrite("ray", &RTCRayHit::ray)
        .def_readwrite("hit", &RTCRayHit::hit);

    PYBIND11_NUMPY_DTYPE(RTCRayHit, ray, hit);

#define DEFINE_RAY_HIT(count)                                                                               \
    /* Ray structure for a packet of rays */                                                                \
    py::class_<RTCRay##count>(m, "RTCRay" STRINGIFY(count))                                                 \
        .def(py::init<>())                                                                                  \
        BIND_PROPERTY_ARRAY(RTCRay##count, org_x, float, count)                                             \
        BIND_PROPERTY_ARRAY(RTCRay##count, org_y, float, count)                                             \
        BIND_PROPERTY_ARRAY(RTCRay##count, org_z, float, count)                                             \
        BIND_PROPERTY_ARRAY(RTCRay##count, tnear, float, count)                                             \
                                                                                                            \
        BIND_PROPERTY_ARRAY(RTCRay##count, dir_x, float, count)                                             \
        BIND_PROPERTY_ARRAY(RTCRay##count, dir_y, float, count)                                             \
        BIND_PROPERTY_ARRAY(RTCRay##count, dir_z, float, count)                                             \
        BIND_PROPERTY_ARRAY(RTCRay##count, time,  float, count)                                             \
                                                                                                            \
        BIND_PROPERTY_ARRAY(RTCRay##count, tfar,  float, count)                                             \
        BIND_PROPERTY_ARRAY(RTCRay##count, mask,  unsigned int, count)                                      \
        BIND_PROPERTY_ARRAY(RTCRay##count, id,    unsigned int, count)                                      \
        BIND_PROPERTY_ARRAY(RTCRay##count, flags, unsigned int, count);                                     \
                                                                                                            \
    /* Hit structure for a packet of rays */                                                                \
    py::class_<RTCHit##count>(m, "RTCHit" STRINGIFY(count))                                                 \
        .def(py::init<>())                                                                                  \
        BIND_PROPERTY_ARRAY(RTCHit##count, Ng_x, float, count)                                              \
        BIND_PROPERTY_ARRAY(RTCHit##count, Ng_y, float, count)                                              \
        BIND_PROPERTY_ARRAY(RTCHit##count, Ng_z, float, count)                                              \
                                                                                                            \
        BIND_PROPERTY_ARRAY(RTCHit##count, u, float, count)                                                 \
        BIND_PROPERTY_ARRAY(RTCHit##count, v, float, count)                                                 \
                                                                                                            \
        BIND_PROPERTY_ARRAY(RTCHit##count, primID,  unsigned int, count)                                    \
        BIND_PROPERTY_ARRAY(RTCHit##count, geomID,  unsigned int, count)                                    \
        BIND_PROPERTY_ARRAY_2D(RTCHit##count, instID,  unsigned int, RTC_MAX_INSTANCE_LEVEL_COUNT, count);  \
                                                                                                            \
    /* Combined ray/hit structure for a packet of rays */                                                   \
    py::class_<RTCRayHit##count>(m, "RTCRayHit" STRINGIFY(count))                                           \
        .def(py::init<>())                                                                                  \
        .def_readwrite("ray", &RTCRayHit##count::ray)                                                       \
        .def_readwrite("hit", &RTCRayHit##count::hit);


    ///* Ray structure for a packet of rays */
    //py::class_<RTCRay4>(m, "RTCRay4")
    //    .def(py::init<>())
    //    BIND_PROPERTY_ARRAY(RTCRay4, org_x, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, org_y, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, org_z, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, tnear, float, 4)

    //    BIND_PROPERTY_ARRAY(RTCRay4, dir_x, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, dir_y, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, dir_z, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, time,  float, 4)

    //    BIND_PROPERTY_ARRAY(RTCRay4, tfar,  float, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, mask,  unsigned int, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, id,    unsigned int, 4)
    //    BIND_PROPERTY_ARRAY(RTCRay4, flags, unsigned int, 4);

    ///* Hit structure for a packet of rays */
    //py::class_<RTCHit4>(m, "RTCHit4")
    //    .def(py::init<>())
    //    BIND_PROPERTY_ARRAY(RTCHit4, Ng_x, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCHit4, Ng_y, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCHit4, Ng_z, float, 4)

    //    BIND_PROPERTY_ARRAY(RTCHit4, u, float, 4)
    //    BIND_PROPERTY_ARRAY(RTCHit4, v, float, 4)

    //    BIND_PROPERTY_ARRAY(RTCHit4, primID,  unsigned int, 4)
    //    BIND_PROPERTY_ARRAY(RTCHit4, geomID,  unsigned int, 4)
    //    BIND_PROPERTY_ARRAY_2D(RTCHit4, instID,  unsigned int, RTC_MAX_INSTANCE_LEVEL_COUNT, 4);

    ///* Combined ray/hit structure for a packet of rays */
    //py::class_<RTCRayHit4>(m, "RTCRayHit4")
    //    .def(py::init<>())
    //    .def_readwrite("ray", &RTCRayHit4::ray)
    //    .def_readwrite("hit", &RTCRayHit4::hit);


    DEFINE_RAY_HIT(4)
    DEFINE_RAY_HIT(8)
    DEFINE_RAY_HIT(16)
    
    py::class_<RTCRayNWrapper>(m, "RTCRayNWrapper");
    py::class_<RTCHitNWrapper>(m, "RTCHitNWrapper");
    py::class_<RTCRayHitNWrapper>(m, "RTCRayHitNWrapper");

    #if defined(__cplusplus)
    
    /* Helper functions to access ray packets of runtime size N */
    m.def("RTCRayN_org_x", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_org_x(r.p, N, i);});
    m.def("RTCRayN_org_y", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_org_y(r.p, N, i);});
    m.def("RTCRayN_org_z", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_org_z(r.p, N, i);});
    m.def("RTCRayN_tnear", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_tnear(r.p, N, i);});

    m.def("RTCRayN_dir_x", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_dir_x(r.p, N, i);});
    m.def("RTCRayN_dir_y", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_dir_y(r.p, N, i);});
    m.def("RTCRayN_dir_z", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_dir_z(r.p, N, i);});
    m.def("RTCRayN_time" , [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_time (r.p, N, i);});
    
    m.def("RTCRayN_tfar",  [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_tfar (r.p, N, i);});
    m.def("RTCRayN_mask",  [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_mask (r.p, N, i);});
    m.def("RTCRayN_id",    [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_id   (r.p, N, i);});
    m.def("RTCRayN_flags", [](RTCRayNWrapper r, unsigned int N, unsigned int i){return RTCRayN_flags(r.p, N, i);});
    
    /* Helper functions to access hit packets of runtime size N */
    m.def("RTCHitN_Ng_x",  [](RTCHitNWrapper h, unsigned int N, unsigned int i){return RTCHitN_Ng_x (h.p, N, i);});
    m.def("RTCHitN_Ng_y",  [](RTCHitNWrapper h, unsigned int N, unsigned int i){return RTCHitN_Ng_y (h.p, N, i);});
    m.def("RTCHitN_Ng_z",  [](RTCHitNWrapper h, unsigned int N, unsigned int i){return RTCHitN_Ng_z (h.p, N, i);});
    
    m.def("RTCHitN_u",  [](RTCHitNWrapper h, unsigned int N, unsigned int i){return RTCHitN_u (h.p, N, i);});
    m.def("RTCHitN_v",  [](RTCHitNWrapper h, unsigned int N, unsigned int i){return RTCHitN_v (h.p, N, i);});
    
    m.def("RTCHitN_primID",  [](RTCHitNWrapper h, unsigned int N, unsigned int i){return RTCHitN_primID (h.p, N, i);});
    m.def("RTCHitN_geomID",  [](RTCHitNWrapper h, unsigned int N, unsigned int i){return RTCHitN_geomID (h.p, N, i);});
    m.def("RTCHitN_instID",  [](RTCHitNWrapper h, unsigned int N, unsigned int i, unsigned int l){return RTCHitN_instID (h.p, N, i, l);});

    /* Helper functions to extract RTCRayN and RTCHitN from RTCRayHitN */
    m.def("RTCRayHitN_RayN",  [](RTCRayHitNWrapper rh, unsigned int N)->RTCRayNWrapper {return {RTCRayHitN_RayN (rh.p, N)};});
    m.def("RTCRayHitN_HitN",  [](RTCRayHitNWrapper rh, unsigned int N)->RTCHitNWrapper {return {RTCRayHitN_HitN (rh.p, N)};});
    
    // TODO: not sure if this has a good python representation...

    ///* Helper structure for a ray packet of compile-time size N */
    //template<int N>
    //struct RTCRayNt
    //{
    //  float org_x[N];
    //  float org_y[N];
    //  float org_z[N];
    //  float tnear[N];
    //
    //  float dir_x[N];
    //  float dir_y[N];
    //  float dir_z[N];
    //  float time[N];
    //
    //  float tfar[N];
    //  unsigned int mask[N];
    //  unsigned int id[N];
    //  unsigned int flags[N];
    //};
    //
    ///* Helper structure for a hit packet of compile-time size N */
    //template<int N>
    //struct RTCHitNt
    //{
    //  float Ng_x[N];
    //  float Ng_y[N];
    //  float Ng_z[N];
    //
    //  float u[N];
    //  float v[N];
    //
    //  unsigned int primID[N];
    //  unsigned int geomID[N];
    //  unsigned int instID[RTC_MAX_INSTANCE_LEVEL_COUNT][N];
    //};
    //
    ///* Helper structure for a combined ray/hit packet of compile-time size N */
    //template<int N>
    //struct RTCRayHitNt
    //{
    //  RTCRayNt<N> ray;
    //  RTCHitNt<N> hit;
    //};
    
    m.def("rtcGetRayFromRayN", [](RTCRayNWrapper rayN, unsigned int N, unsigned int i){ return rtcGetRayFromRayN(rayN.p, N, i);});
    m.def("rtcGetHitFromHitN", [](RTCHitNWrapper hitN, unsigned int N, unsigned int i){ return rtcGetHitFromHitN(hitN.p, N, i);});
    m.def("rtcCopyHitToHitN", [](RTCHitNWrapper hitN, const RTCHit* hit, unsigned int N, unsigned int i){ return rtcCopyHitToHitN(hitN.p, hit, N, i);});
    m.def("rtcGetRayHitFromRayHitN", [](RTCRayHitNWrapper rayhitN, unsigned int N, unsigned int i){ return rtcGetRayHitFromRayHitN(rayhitN.p, N, i);});





    m.def("rtcCreateRayHits", [](py::array_t<float> orgx, py::array_t<float> orgy, py::array_t<float> orgz, py::array_t<float> dirx, py::array_t<float> diry, py::array_t<float> dirz, py::array_t<unsigned int> ids){ 
        size_t N = orgx.shape()[0];
        assert(N == orgy.shape()[0]);
        assert(N == orgz.shape()[0]);
        assert(N == dirx.shape()[0]);
        assert(N == diry.shape()[0]);
        assert(N == dirz.shape()[0]);
        assert(N == ids.shape()[0]);
        float* ox = (float*)orgx.request().ptr;
        float* oy = (float*)orgy.request().ptr;
        float* oz = (float*)orgz.request().ptr;
        float* dx = (float*)dirx.request().ptr;
        float* dy = (float*)diry.request().ptr;
        float* dz = (float*)dirz.request().ptr;
        unsigned int* pids = (unsigned int*)ids.request().ptr;

        py::list lst;
        for (size_t i=0; i<N; ++i) {
            RTCRayHit rh;
            rh.ray.org_x = ox[i];
            rh.ray.org_y = oy[i];
            rh.ray.org_z = oz[i];
            rh.ray.dir_x = dx[i];
            rh.ray.dir_y = dy[i];
            rh.ray.dir_z = dz[i];
            rh.ray.tnear = 0;
            rh.ray.tfar = float('inf');
            rh.ray.mask = 0xffffffff;
            rh.ray.id = pids[i];
            rh.ray.flags = 0;
            rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            lst.append(rh);
        }
        return lst;
    });


    m.def("rtcTransformRayHits", [](py::list rayhits, std::function<std::pair<int, RTCRayHit>(RTCRayHit&)> transform){
        py::list out;
        for (auto rh = rayhits.begin(); rh != rayhits.end(); ++rh) {
            auto t = transform(*(rh->cast<RTCRayHit*>()));
            if (t.first != 0) {
                t.second.ray.tnear = 0;
                t.second.ray.tfar = float('inf');
                t.second.ray.mask = 0xffffffff;
                t.second.ray.flags = 0;
                t.second.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                t.second.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
                out.append(t.second);
            }
        }
        return out;
    });



#define RAYHIT_GET_PROPERTY(ty, rayhit, name)\
    [](py::list rayhits, bool filter_misses = true) {\
        size_t size = 0;\
        for (auto rh = rayhits.begin(); rh != rayhits.end(); ++rh) if (!filter_misses || rh->cast<RTCRayHit*>()->hit.geomID != RTC_INVALID_GEOMETRY_ID) size++;\
        py::array_t<ty> a(size);\
        ty* aptr = (ty*)a.request().ptr;\
        size_t i = 0;\
        for (auto rh = rayhits.begin(); rh != rayhits.end(); ++rh) if (!filter_misses || rh->cast<RTCRayHit*>()->hit.geomID != RTC_INVALID_GEOMETRY_ID) aptr[i++] = rh->cast<RTCRayHit*>()->rayhit.name;\
        return a;\
    }

    m.def("rtcRayHits_get_orgx", RAYHIT_GET_PROPERTY(float, ray, org_x));
    m.def("rtcRayHits_get_orgy", RAYHIT_GET_PROPERTY(float, ray, org_y));
    m.def("rtcRayHits_get_orgz", RAYHIT_GET_PROPERTY(float, ray, org_z));
    m.def("rtcRayHits_get_dirx", RAYHIT_GET_PROPERTY(float, ray, dir_x));
    m.def("rtcRayHits_get_diry", RAYHIT_GET_PROPERTY(float, ray, dir_y));
    m.def("rtcRayHits_get_dirz", RAYHIT_GET_PROPERTY(float, ray, dir_z));
    m.def("rtcRayHits_get_tfar", RAYHIT_GET_PROPERTY(float, ray, tfar));
    m.def("rtcRayHits_get_rayids", RAYHIT_GET_PROPERTY(unsigned int, ray, id));
    
    #endif
}