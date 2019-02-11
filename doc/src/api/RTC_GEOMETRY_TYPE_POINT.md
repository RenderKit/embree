% RTC_GEOMETRY_TYPE_*_POINT(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_SPHERE_POINT -
      point geometry spheres

    RTC_GEOMETRY_TYPE_DISC_POINT -
      point geometry with ray-oriented discs

    RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT -
      point geometry with normal-oriented discs

#### SYNOPSIS

    #include <embree3/rtcore.h>

    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SPHERE_POINT);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_DISC_POINT);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT);

#### DESCRIPTION

Points with per vertex radii are supported with sphere, ray-oriented
discs, and normal-oriented discs geometric represetntations. Such
point geometries are
created by passing `RTC_GEOMETRY_TYPE_SPHERE_POINT`,
`RTC_GEOMETRY_TYPE_DISC_POINT`, or
`RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT` to the `rtcNewGeometry`
function. The point vertices can be specified t through a vertex
buffer (`RTC_BUFFER_TYPE_VERTEX`). For the normal oriented discs
a normal buffer (`RTC_BUFFER_TYPE_NORMAL`) has to get specified
additionally. See `rtcSetGeometryBuffer` and
`rtcSetSharedGeometryBuffer` for more details on how to set buffers.

The vertex buffer stores each control vertex in the form of a single
precision position and radius stored in (`x`, `y`, `z`, `r`) order in
memory (`RTC_FORMAT_FLOAT4` format). The number of vertices is
inferred from the size of this buffer. Similarly, the normal buffer
stores a single precision normal per control vertex (`x`, `y`, `z`
order and `RTC_FORMAT_FLOAT3` format).

In the `RTC_GEOMETRY_TYPE_SPHERE_POINT` mode, a real geometric surface
is rendered for the curve, which is more expensive but allows closeup
views.

The `RTC_GEOMETRY_TYPE_DISC_POINT` flat mode is a fast mode designed to
render distant points. In this mode the point is rendered as a ray
facing disc.

The `RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT` mode is a mode designed as
a midpoint geometrically between ray facing discs and spheres. In this
mode the point is rendered as a normal oriented disc.

For all point types, only the hit distance and geometry normal is
returned as hit information, u and v are set to zero.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers must have the same stride and size.

Also see tutorial [Points] for an example of how to create and
use point geometries.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry]
