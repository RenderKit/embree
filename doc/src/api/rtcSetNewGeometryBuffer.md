% rtcSetNewGeometryBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetNewGeometryBuffer - creates and assigns a new data buffer to
      the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void* rtcSetNewGeometryBuffer(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot,
      enum RTCFormat format,
      size_t byteStride,
      size_t itemCount
    );

#### DESCRIPTION

The `rtcSetNewGeometryBuffer` function creates a new data buffer of
specified format (`format` argument), byte stride (`byteStride`
argument), and number of items (`itemCount` argument), and assigns it
to a geometry buffer slot (`type` and `slot` argument) of the
specified geometry (`geometry` argument). The buffer data is managed
internally and automatically freed when the geometry is destroyed.

The byte stride (`byteStride` argument) must be aligned to 4 bytes;
otherwise the `rtcSetNewGeometryBuffer` function will fail.

The allocated buffer will be automatically over-allocated slightly
when used as a vertex buffer, where a requirement is that each buffer
element should be readable using 16-byte SSE load instructions.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryBuffer], [rtcSetSharedGeometryBuffer]
