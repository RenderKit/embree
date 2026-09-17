% RTC_GEOMETRY_TYPE_USER_ORIENTED(3) | Embree Ray Tracing Kernels 4

#### NAME

    RTC_GEOMETRY_TYPE_USER_ORIENTED - user geometry with oriented bounds

#### SYNOPSIS

    #include <embree4/rtcore.h>

    RTCGeometry geometry =
      rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER_ORIENTED);

#### DESCRIPTION

Oriented user geometry is user-defined geometry whose primitives provide
oriented bounding boxes. Embree uses these boxes when constructing oriented
BVH nodes, which can reduce overlap for rotated or strongly anisotropic
primitives.

The geometry is configured like `RTC_GEOMETRY_TYPE_USER`, except that
`rtcSetGeometryOrientedBoundsFunction` must be used instead of
`rtcSetGeometryBoundsFunction`.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be queried
using `rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_USER], [rtcNewGeometry],
[rtcSetGeometryOrientedBoundsFunction], [rtcSetGeometryIntersectFunction],
[rtcSetGeometryOccludedFunction]
