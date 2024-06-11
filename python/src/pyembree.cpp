#include "pyembree.h"

namespace py = pybind11;

    int add(int a, int b) {
        return a + b;
    }

PYBIND11_MODULE(pyembree, m) {

    m.def("add", &add, "AOEUAOEU");
    m.def("add2", [](int a, int b){return add(a,b);});

    py::class_<EmbreeVoidPtr>(m, "pvoid")
        .def("get_raw",  &EmbreeVoidPtr::get_raw)
        .def("as_float", &EmbreeVoidPtr::as_float)
        .def("as_uint",  &EmbreeVoidPtr::as_uint)
        .def("as_int",   &EmbreeVoidPtr::as_int);
    bind_embree_ptr<float>(m, "pfloat");
    bind_embree_ptr<uint32_t>(m, "puint");
    bind_embree_ptr<int32_t>(m, "pint");
    

    bind_rtcore_common(m);
    bind_rtcore_device(m);
    bind_rtcore_buffer(m);
    bind_rtcore_ray(m);
    bind_rtcore_geometry(m);
    bind_rtcore_scene(m);
    bind_rtcore_builder(m);
    bind_rtcore_quaternion(m);





#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif

}