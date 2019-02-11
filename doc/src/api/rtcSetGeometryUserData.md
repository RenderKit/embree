% rtcSetGeometryUserData(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryUserData - sets the user-defined data pointer of the
      geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryUserData(RTCGeometry geometry, void* userPtr);

#### DESCRIPTION

The `rtcSetGeometryUserData` function sets the user-defined data
pointer (`userPtr` argument) for a geometry (`geometry` argument). This
user data pointer is intended to be pointing to the application's
representation of the geometry, and is passed to various callback
functions. The application can use this pointer inside the callback
functions to access its geometry representation.

The `rtcGetGeometryUserData` function can be used to query an already
set user data pointer of a geometry.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetGeometryUserData]
