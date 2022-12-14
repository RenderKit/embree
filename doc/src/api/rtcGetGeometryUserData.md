% rtcGetGeometryUserData(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryUserData - returns the user data pointer
      of the geometry

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void* rtcGetGeometryUserData(RTCGeometry geometry);

#### DESCRIPTION

The `rtcGetGeometryUserData` function queries the user data pointer
previously set with `rtcSetGeometryUserData`. When
`rtcSetGeometryUserData` was not called yet, `NULL` is returned.

This function is supposed to be used during rendering, but only
supported on the CPU and in SYCL on the GPU.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryUserData]
