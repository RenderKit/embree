#include "pyembree.h"

namespace py = pybind11;

void bind_rtcore_builder(py::module &m) {
    /* Opaque BVH type */
    py::class_<RTCBVHWrapper>(m, "RTCBVHWrapper");

    /* Input build primitives for the builder */
    py::class_<RTCBuildPrimitive>(m, "RTCBuildPrimitive")
        .def_readwrite("lower_x", &RTCBuildPrimitive::lower_x)
        .def_readwrite("lower_y", &RTCBuildPrimitive::lower_y)
        .def_readwrite("lower_z", &RTCBuildPrimitive::lower_z)
        .def_readwrite("geomID", &RTCBuildPrimitive::geomID)
        .def_readwrite("upper_x", &RTCBuildPrimitive::upper_x)
        .def_readwrite("upper_y", &RTCBuildPrimitive::upper_y)
        .def_readwrite("upper_z", &RTCBuildPrimitive::upper_z)
        .def_readwrite("primID", &RTCBuildPrimitive::primID);

    /* Opaque thread local allocator type */
    py::class_<RTCThreadLocalAllocatorWrapper>(m, "RTCThreadLocalAllocatorWrapper");

    /* Build flags */
    py::enum_<RTCBuildFlags>(m, "RTCBuildFlags")
        .value("RTC_BUILD_FLAG_NONE", RTC_BUILD_FLAG_NONE)
        .value("RTC_BUILD_FLAG_DYNAMIC", RTC_BUILD_FLAG_DYNAMIC)
        .export_values();

    py::enum_<RTCBuildConstants>(m, "RTCBuildConstants")
        .value("RTC_BUILD_MAX_PRIMITIVES_PER_LEAF", RTC_BUILD_MAX_PRIMITIVES_PER_LEAF)
        .export_values();

    /* Input for builders */
    py::class_<RTCBuildArguments>(m, "RTCBuildArguments")
        .def_readwrite("byteSize", &RTCBuildArguments::byteSize)
        .def_readwrite("buildQuality", &RTCBuildArguments::buildQuality)
        .def_readwrite("buildFlags", &RTCBuildArguments::buildFlags)
        .def_readwrite("maxBranchingFactor", &RTCBuildArguments::maxBranchingFactor)
        .def_readwrite("maxDepth", &RTCBuildArguments::maxDepth)
        .def_readwrite("sahBlockSize", &RTCBuildArguments::sahBlockSize)
        .def_readwrite("minLeafSize", &RTCBuildArguments::minLeafSize)
        .def_readwrite("maxLeafSize", &RTCBuildArguments::maxLeafSize)
        .def_readwrite("traversalCost", &RTCBuildArguments::traversalCost)
        .def_readwrite("intersectionCost", &RTCBuildArguments::intersectionCost)
    
        .def_property("bvh", [](RTCBuildArguments& c)->RTCBVHWrapper { return {c.bvh};}, [](RTCBuildArguments& c, RTCBVHWrapper bvh){ c.bvh = bvh.p; })

        .def_readwrite("primitives", &RTCBuildArguments::primitives)
        .def_readwrite("primitiveCount", &RTCBuildArguments::primitiveCount)
        .def_readwrite("primitiveArrayCapacity", &RTCBuildArguments::primitiveArrayCapacity)
    
        //.def_readwrite("createNode", &RTCBuildArguments::createNode)
        //.def_readwrite("setNodeChildren", &RTCBuildArguments::setNodeChildren)
        //.def_readwrite("setNodeBounds", &RTCBuildArguments::setNodeBounds)
        //.def_readwrite("createLeaf", &RTCBuildArguments::createLeaf)
        //.def_readwrite("splitPrimitive", &RTCBuildArguments::splitPrimitive)
        //.def_readwrite("buildProgress", &RTCBuildArguments::buildProgress)
        .def_readwrite("userPtr", &RTCBuildArguments::userPtr);

    /* Returns the default build settings.  */
    m.def("rtcDefaultBuildArguments", &rtcDefaultBuildArguments);
    
    /* Creates a new BVH. */
    m.def("rtcNewBVH", [](RTCDeviceWrapper device)->RTCBVHWrapper { return {rtcNewBVH(device.d)}; });

    /* Builds a BVH. */
    m.def("rtcBuildBVH", &rtcBuildBVH);

    /* Allocates memory using the thread local allocator. */
    m.def("rtcThreadLocalAlloc", [](RTCThreadLocalAllocatorWrapper allocator, size_t bytes, size_t align){ return rtcThreadLocalAlloc(allocator.p, bytes, align); });

    /* Retains the BVH (increments reference count). */
    m.def("rtcRetainBVH", [](RTCBVHWrapper bvh){ rtcRetainBVH(bvh.p); });

    /* Releases the BVH (decrements reference count). */
    m.def("rtcReleaseBVH", [](RTCBVHWrapper bvh){ rtcReleaseBVH(bvh.p); });
}