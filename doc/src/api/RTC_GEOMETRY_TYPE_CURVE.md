% RTC_GEOMETRY_TYPE_*_CURVE(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE -
      Bézier curve geometry type using a sweep surface

    RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE -
      B-spline curve geometry type using a sweep surface
    
    RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE -
      linear curve geometry type

    RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE -
      Bézier curve geometry type using a ray oriented ribbon approximation

    RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE - 
      B-spline curve geometry type using a ray oriented ribbon approximation

    RTC_GEOMETRY_TYPE_ORIENTED_FLAT_BEZIER_CURVE -
      flat Bézier curve geometry type oriented by normal

    RTC_GEOMETRY_TYPE_ORIENTED_FLAT_BSPLINE_CURVE - 
      flat B-spline curve geometry type oriented by normal

#### SYNOPSIS

    #include <embree3/rtcore.h>

    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ORIENTED_FLAT_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ORIENTED_FLAT_BSPLINE_CURVE);

#### DESCRIPTION

Curves with per vertex radii are supported with linear, cubic Bézier,
and cubic B-spline bases. Such curve geometries are created by passing
`RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_ORIENTED_FLAT_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_ORIENTED_FLAT_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE`, or
`RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE`, to the `rtcNewGeometry`
function. The curve indices can be specified through an index buffer
(`RTC_BUFFER_TYPE_INDEX`) and the curve vertices through a vertex
buffer (`RTC_BUFFER_TYPE_VERTEX`). For normal oriented curves a normal
buffer (`RTC_BUFFER_TYPE_NORMAL`) has to get specified. See
`rtcSetGeometryBuffer` and `rtcSetSharedGeometryBuffer` for more
details on how to set buffers.

The index buffer contains an array of 32-bit indices
(`RTC_FORMAT_UINT` format), each pointing to the ID of the first
control vertex. The vertex buffer stores each control vertex in the
form of a single precision position and radius stored in `x`, `y`,
`z`, `r` order in memory (`RTC_FORMAT_FLOAT4` format). The number of
vertices is inferred from the size of this buffer. The radii may be
smaller than zero for the B-Spline basis, but the interpolated radii
should always be greater or equal to zero. The normal buffer stores a
single precision normal per control vertex in `x`, `y`, `z` order.

The `RTC_GEOMETRY_TYPE_FLAT_*` flat mode is a fast mode designed to
render distant hair. In this mode the curve is rendered as a connected
sequence of ray facing quads. Individual quads are considered to have
subpixel size, and zooming onto the curve might show geometric
artifacts. The number of quads to subdivide into can be specified
through the `rtcSetGeometryTessellationRate` function. By default the
tessellation rate is 4.

The `RTC_GEOMETRY_TYPE_ORIENTED_FLAT_*` mode is a mode designed to
render grass blades. In this mode the curve is rendered as a flat band
whose center exactly follows the provided vertex spline, whose half
width approximately follows the provided radius spline, and whose
orientation approximately follows the provided normal spline.

More precisely, we perform a newton-raphson style intersection of a
ray with a tensor product surface of a linear basis (perpendicular to
the curve) and cubic Bezier basis (along the curve). We construct the
8 control points of this surface in Bezier basis by calculating a
normalized direction `d01=normalize(v1-v0,n0)` and
`d23=normalize(v3-v2,n3)`. These directions are perpendicular to the
direction of the center curve and evaluated normal curve at the curve
start and end. The 8 control vertices of the surface are constructed
as:

     p00 = v0-r0*d01, p10 = v0+r0*d01
     p01 = v1-r1*d01, p11 = v1+r1*d01
     p02 = v2-r2*d23, p12 = v2+r2*d23
     p03 = v3-r3*d23, p13 = v3+r3*d23

The center of this curve exactly follows the specified center spline,
the normal at the start (and end) exactly match the evaluated normal
spline at the start (and end), and the half width exactly matches the
evaluated radius spline at the start (and end). In-between the radius
and orientation of the curve smoothly changes. Note that the
construction does not work when the provided normals are parallel to
the curve direction, as then no well defined perpendicular direction
`d01` or `d23` are defined, thus the provided normals should best be
kept as perpendicular to the curve direction as possible.

In the `RTC_GEOMETRY_TYPE_ROUND_*` round mode, a real geometric
surface is rendered for the curve, which is more expensive but allows
closeup views. For the Bézier and B-spline bases, this mode renders a
sweep surface by sweeping a varying radius circle tangential along the
curve. As a limitation, the radius of the curve has to be smaller than
the curvature radius of the curve at each location on the curve. The
round mode is currently not supported for the linear basis.

The intersection with the curve segment stores the parametric hit
location along the curve segment as u-coordinate (range 0 to +1).

For Bézier and B-spline curves, the v-coordinate is set to the
normalized distance in the range -1 to +1. For the linear basis and in
round mode the v-coordinate is set to zero.

In flat mode, the geometry normal `Ng` is set to the tangent of the
curve at the hit location. In round mode, the geometry normal `Ng` is
set to the non-normalized geometric normal of the surface.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers must have the same stride and size.

Also see tutorials [Hair] and [Curves] for examples of how to create and
use curve geometries.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcDeviceGetError`.

#### SEE ALSO

[rtcNewGeometry]
