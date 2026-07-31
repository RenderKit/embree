% rtcGetGeometryBufferDataDevice(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryBufferDataDevice - gets pointer to
      the first buffer view element on the device.

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void* rtcGetGeometryBufferDataDevice(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot
    );

#### DESCRIPTION

The `rtcGetGeometryBufferDataDevice` function returns a pointer to the first
element of the buffer view attached to the specified buffer type and
slot (`type` and `slot` argument) of the geometry (`geometry`
argument) on the device.

If the device associated with `geometry` is no SYCL device, or if Embree
is executed on a system with host unified memory (e.g., on an iGPU),
the returned pointer is the same as the one returned by `rtcGetGeometryBufferData`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetGeometryBufferData], [rtcSetGeometryBuffer], [rtcSetSharedGeometryBuffer], [rtcSetNewGeometryBuffer]
