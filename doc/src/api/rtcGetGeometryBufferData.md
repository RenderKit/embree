% rtcGetGeometryBufferData(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryBufferData - gets pointer to
      the first buffer view element

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void* rtcGetGeometryBufferData(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot
    );

#### DESCRIPTION

The `rtcGetGeometryBufferData` function returns a pointer to the first
element of the buffer view attached to the specified buffer type and
slot (`type` and `slot` argument) of the geometry (`geometry`
argument).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryBuffer], [rtcSetSharedGeometryBuffer], [rtcSetNewGeometryBuffer]
