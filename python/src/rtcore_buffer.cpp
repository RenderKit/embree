#include "pyembree.h"

namespace py = pybind11;


void bind_rtcore_buffer(py::module &m) {
    py::enum_<RTCBufferType>(m, "RTCBufferType")
        .value("RTC_BUFFER_TYPE_INDEX", RTC_BUFFER_TYPE_INDEX)
        .value("RTC_BUFFER_TYPE_VERTEX", RTC_BUFFER_TYPE_VERTEX)
        .value("RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE", RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
        .value("RTC_BUFFER_TYPE_NORMAL", RTC_BUFFER_TYPE_NORMAL)
        .value("RTC_BUFFER_TYPE_TANGENT", RTC_BUFFER_TYPE_TANGENT)
        .value("RTC_BUFFER_TYPE_NORMAL_DERIVATIVE", RTC_BUFFER_TYPE_NORMAL_DERIVATIVE)

        .value("RTC_BUFFER_TYPE_GRID", RTC_BUFFER_TYPE_GRID)

        .value("RTC_BUFFER_TYPE_FACE", RTC_BUFFER_TYPE_FACE)
        .value("RTC_BUFFER_TYPE_LEVEL", RTC_BUFFER_TYPE_LEVEL)
        .value("RTC_BUFFER_TYPE_EDGE_CREASE_INDEX", RTC_BUFFER_TYPE_EDGE_CREASE_INDEX)
        .value("RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT", RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT)
        .value("RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX", RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX)
        .value("RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT", RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT)
        .value("RTC_BUFFER_TYPE_HOLE", RTC_BUFFER_TYPE_HOLE)

        .value("RTC_BUFFER_TYPE_FLAGS", RTC_BUFFER_TYPE_FLAGS)
        .export_values();

    /* Opaque buffer type */
    py::class_<RTCBufferWrapper>(m, "RTCBufferWrapper");

    /* Creates a new buffer. */
    m.def("rtcNewBuffer", [](RTCDeviceWrapper device, size_t byteSize)->RTCBufferWrapper { return RTCBufferWrapper{rtcNewBuffer(device.d, byteSize)}; });

    /* Creates a new shared buffer. */
    m.def("rtcNewSharedBuffer", [](RTCDeviceWrapper device, void* ptr, size_t byteSize)->RTCBufferWrapper { return RTCBufferWrapper{rtcNewSharedBuffer(device.d, ptr, byteSize)}; });

    /* Returns a pointer to the buffer data. */
    m.def("rtcGetBufferData", [](RTCBufferWrapper buffer){ return EmbreeVoidPtr(rtcGetBufferData(buffer.b)); });

    /* Retains the buffer (increments the reference count). */
    m.def("rtcRetainBuffer", [](RTCBufferWrapper buffer){rtcRetainBuffer(buffer.b);});

    /* Releases the buffer (decrements the reference count). */
    m.def("rtcReleaseBuffer", [](RTCBufferWrapper buffer){rtcReleaseBuffer(buffer.b);});
}
