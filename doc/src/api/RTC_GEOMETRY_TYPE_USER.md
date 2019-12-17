% RTC_GEOMETRY_TYPE_USER(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_USER - user geometry type

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry geometry =
      rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

#### DESCRIPTION

User-defined geometries contain a number of user-defined primitives,
just like triangle meshes contain multiple triangles. The shape of the
user-defined primitives is specified through registered callback
functions, which enable extending Embree with arbitrary types of
primitives.

User-defined geometries are created by passing `RTC_GEOMETRY_TYPE_USER`
to the `rtcNewGeometry` function call. One has to set the number of
primitives (see `rtcSetGeometryUserPrimitiveCount`), a user data
pointer (see `rtcSetGeometryUserData`), a bounding function closure
(see `rtcSetGeometryBoundsFunction`), as well as user-defined
intersect (see `rtcSetGeometryIntersectFunction`) and occluded (see
`rtcSetGeometryOccludedFunction`) callback functions. The bounding
function is used to query the bounds of all time steps of a user
primitive, while the intersect and occluded callback functions are
called to intersect the primitive with a ray. The user data pointer is
passed to each callback invocation and can be used to point to the
application's representation of the user geometry.

The creation of a user geometry typically looks the following:

    RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
    rtcSetGeometryUserPrimitiveCount(geometry, numPrimitives);
    rtcSetGeometryUserData(geometry, userGeometryRepresentation);
    rtcSetGeometryBoundsFunction(geometry, boundsFunction);
    rtcSetGeometryIntersectFunction(geometry, intersectFunction);
    rtcSetGeometryOccludedFunction(geometry, occludedFunction);

Please have a look at the `rtcSetGeometryBoundsFunction`,
`rtcSetGeometryIntersectFunction`, and `rtcSetGeometryOccludedFunction`
functions on the implementation of the callback functions.

See tutorial [User Geometry] for an example of how to use the
user-defined geometries.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcSetGeometryUserPrimitiveCount],
[rtcSetGeometryUserData], [rtcSetGeometryBoundsFunction],
[rtcSetGeometryIntersectFunction], [rtcSetGeometryOccludedFunction]
