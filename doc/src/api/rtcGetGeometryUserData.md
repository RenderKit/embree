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
supported on the CPU and not inside SYCL kernels on the GPU. Inside a
SYCL kernel the `rtcGetGeometryUserDataFromScene` function has to get
used.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryUserData], [rtcGetGeometryUserDataFromScene]
