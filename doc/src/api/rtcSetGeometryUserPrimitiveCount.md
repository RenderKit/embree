% rtcSetGeometryUserPrimitiveCount(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryUserPrimitiveCount - sets the number of primitives
      of a user-defined geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryUserPrimitiveCount(
      RTCGeometry geometry,
      unsigned int userPrimitiveCount
    );

#### DESCRIPTION

The `rtcSetGeometryUserPrimitiveCount` function sets the number of
user-defined primitives (`userPrimitiveCount` parameter) of the
specified user-defined geometry (`geometry` parameter).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_USER]
