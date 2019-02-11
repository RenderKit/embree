% rtcGetGeometryUserData(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryUserData - returns the user data pointer
      of the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void* rtcGetGeometryUserData(RTCGeometry geometry);

#### DESCRIPTION

The `rtcGetGeometryUserData` function queries the user data pointer
previously set with `rtcSetGeometryUserData`. When
`rtcSetGeometryUserData` was not called yet, `NULL` is returned.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryUserData]
