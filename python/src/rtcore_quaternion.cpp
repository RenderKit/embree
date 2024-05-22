#include "pyembree.h"

namespace py = pybind11;

void bind_rtcore_quaternion(py::module &m) {
    /*
     * Structure for transformation representation as a matrix decomposition using
     * a quaternion
     */
    py::class_<RTCQuaternionDecomposition>(m, "RTCQuaternionDecomposition")
        .def_readwrite("scale_x", &RTCQuaternionDecomposition::scale_x)
        .def_readwrite("scale_y", &RTCQuaternionDecomposition::scale_y)
        .def_readwrite("scale_z", &RTCQuaternionDecomposition::scale_z)
        .def_readwrite("skew_xy", &RTCQuaternionDecomposition::skew_xy)
        .def_readwrite("skew_xz", &RTCQuaternionDecomposition::skew_xz)
        .def_readwrite("skew_yz", &RTCQuaternionDecomposition::skew_yz)
        .def_readwrite("shift_x", &RTCQuaternionDecomposition::shift_x)
        .def_readwrite("shift_y", &RTCQuaternionDecomposition::shift_y)
        .def_readwrite("shift_z", &RTCQuaternionDecomposition::shift_z)
        .def_readwrite("quaternion_r", &RTCQuaternionDecomposition::quaternion_r)
        .def_readwrite("quaternion_i", &RTCQuaternionDecomposition::quaternion_i)
        .def_readwrite("quaternion_j", &RTCQuaternionDecomposition::quaternion_j)
        .def_readwrite("quaternion_k", &RTCQuaternionDecomposition::quaternion_k)
        .def_readwrite("translation_x", &RTCQuaternionDecomposition::translation_x)
        .def_readwrite("translation_y", &RTCQuaternionDecomposition::translation_y)
        .def_readwrite("translation_z", &RTCQuaternionDecomposition::translation_z);

    m.def("rtcInitQuaternionDecomposition", &rtcInitQuaternionDecomposition);

    m.def("rtcQuaternionDecompositionSetQuaternion", &rtcQuaternionDecompositionSetQuaternion);

    m.def("rtcQuaternionDecompositionSetScale", &rtcQuaternionDecompositionSetScale);

    m.def("rtcQuaternionDecompositionSetSkew", &rtcQuaternionDecompositionSetSkew);
    
    m.def("rtcQuaternionDecompositionSetShift", &rtcQuaternionDecompositionSetShift);

    m.def("rtcQuaternionDecompositionSetTranslation", &rtcQuaternionDecompositionSetTranslation);
}