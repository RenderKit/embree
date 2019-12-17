% rtcUpdateGeometryBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcUpdateGeometryBuffer - marks a buffer view bound to the geometry
      as modified

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcUpdateGeometryBuffer(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot
    );

#### DESCRIPTION

The `rtcUpdateGeometryBuffer` function marks the buffer view bound to
the specified buffer type and slot (`type` and `slot` argument) of a
geometry (`geometry` argument) as modified.

If a data buffer is changed by the application, the
`rtcUpdateGeometryBuffer` call must be invoked for that buffer. Each
buffer view assigned to a buffer slot is initially marked as modified,
thus this function needs to be called only when doing buffer
modifications after the first `rtcCommitScene`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcCommitScene]
