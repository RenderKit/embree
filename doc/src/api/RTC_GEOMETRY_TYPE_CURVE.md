% RTC_GEOMETRY_TYPE_*_CURVE(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE -
      flat curve geometry with linear basis

    RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE -
      flat curve geometry with cubic Bézier basis

    RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE - 
      flat curve geometry with cubic B-spline basis

    RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE - 
      flat curve geometry with cubic Hermite basis

    RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE - 
      flat curve geometry with Catmull-Rom basis

    RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE -
      flat normal oriented curve geometry with cubic Bézier basis

    RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE - 
      flat normal oriented curve geometry with cubic B-spline basis

    RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE - 
      flat normal oriented curve geometry with cubic Hermite basis

    RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE - 
      flat normal oriented curve geometry with Catmull-Rom basis

    RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE -
      capped cone curve geometry with linear basis - discontinuous at edge boundaries

    RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE -
      capped cone curve geometry with linear basis and spherical ending

    RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE -
      swept surface curve geometry with cubic Bézier basis

    RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE -
      swept surface curve geometry with cubic B-spline basis

    RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE -
      swept surface curve geometry with cubic Hermite basis

    RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE -
      swept surface curve geometry with Catmull-Rom basis

#### SYNOPSIS

    #include <embree3/rtcore.h>

    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE);

#### DESCRIPTION

Curves with per vertex radii are supported with linear, cubic Bézier,
cubic B-spline, and cubic Hermite bases. Such curve geometries are
created by passing `RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE`,
`RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_FLAT_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_FLAT_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_FLAT_HERMITE_CURVE`,
`RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_FLAT_CATMULL_ROM_CURVE`,
`RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE`, or
`RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE` to the `rtcNewGeometry`
function. The curve indices can be specified through an index buffer
(`RTC_BUFFER_TYPE_INDEX`) and the curve vertices through a vertex
buffer (`RTC_BUFFER_TYPE_VERTEX`). For the Hermite basis a tangent
buffer (`RTC_BUFFER_TYPE_TANGENT`), normal oriented curves a normal
buffer (`RTC_BUFFER_TYPE_NORMAL`), and for normal oriented Hermite
curves a normal derivative buffer
(`RTC_BUFFER_TYPE_NORMAL_DERIVATIVE`) has to get specified
additionally. See `rtcSetGeometryBuffer` and
`rtcSetSharedGeometryBuffer` for more details on how to set buffers.

The index buffer contains an array of 32-bit indices
(`RTC_FORMAT_UINT` format), each pointing to the first control vertex
in the vertex buffer, but also to the first tangent in the tangent
buffer, and first normal in the normal buffer if these buffers are
present.

The vertex buffer stores each control vertex in the form of a single
precision position and radius stored in (`x`, `y`, `z`, `r`) order in
memory (`RTC_FORMAT_FLOAT4` format). The number of vertices is
inferred from the size of this buffer. The radii may be smaller than
zero but the interpolated radii should always be greater or equal to
zero. Similarly, the tangent buffer stores the derivative of each
control vertex (`x`, `y`, `z`, `r` order and `RTC_FORMAT_FLOAT4`
format) and the normal buffer stores a single precision normal per
control vertex (`x`, `y`, `z` order and `RTC_FORMAT_FLOAT3` format).

##### Linear Basis

For the linear basis the indices point to the first of 2 consecutive
control points in the vertex buffer. The first control point is the
start and the second control point the end of the line segment. When
constructing hair strands in this basis, the end-point can be shared
with the start of the next line segment.

For the linear basis the user optionally can provide a flags buffer of
type `RTC_BUFFER_TYPE_FLAGS` which contains bytes that encode if the
left neighbor segment (`RTC_CURVE_FLAG_NEIGHBOR_LEFT` flag) and/or
right neighbor segment (`RTC_CURVE_FLAG_NEIGHBOR_RIGHT` flags) exist
(see [RTCCurveFlags]). If this buffer is not set, than the left/right
neighbor bits are automatically calculated base on the index buffer
(left segment exists if segment(id-1)+1 == segment(id) and right
segment exists if segment(id+1)-1 == segment(id)).

A left neighbor segment is assumed to end at the start vertex of the
current segment, and to start at the previous vertex in the vertex
buffer. Similarly, the right neighbor segment is assumed to start at
the end vertex of the current segment, and to end at the next vertex
in the vertex buffer.

Only when the left and right bits are properly specified the current
segment can properly attach to the left and/or right neighbor,
otherwise the touching area may not get rendered properly.

##### Bézier Basis

For the cubic Bézier basis the indices point to the first of 4
consecutive control points in the vertex buffer. These control points
use the cubic Bézier basis, where the first control point represents
the start point of the curve, and the 4th control point the end point
of the curve. The Bézier basis is interpolating, thus the curve does
go exactly through the first and fourth control vertex.

##### B-spline Basis

For the cubic B-spline basis the indices point to the first of 4
consecutive control points in the vertex buffer. These control points
make up a cardinal cubic B-spline (implicit equidistant knot
vector). This basis is not interpolating, thus the curve does in
general not go through any of the control points directly. A big
advantage of this basis is that 3 control points can be shared for two
continuous neighboring curve segments, e.g. the curves (p0,p1,p2,p3)
and (p1,p2,p3,p4) are C1 continuous. This feature makes this basis a
good choice to construct continuous multi-segment curves, as memory
consumption can be kept minimal.

##### Hermite Basis

For the cubic Hermite basis the indices point to the first of 2
consecutive points in the vertex buffer, and the first of 2
consecutive tangents in the tangent buffer. These two points and two
tangents make up a cubic Hermite curve. This basis is interpolating,
thus does exactly go through the first and second control point, and
the first order derivative at the begin and end matches exactly the
value specified in the tangent buffer. When connecting two segments
continuously, the end point and tangent of the previous segment can be
shared. Different versions of Catmull-Rom splines can be easily
constructed using the Hermite basis, by calculating a proper tangent
buffer from the control points.

##### Catmull-Rom Basis

For the Catmull-Rom basis the indices point to the first of 4
consecutive control points in the vertex buffer.  This basis goes
through p1 and p2, with tangents (p2-p0)/2 and (p3-p1)/2.

##### Flat Curves

The `RTC_GEOMETRY_TYPE_FLAT_*` flat mode is a fast mode designed to
render distant hair. In this mode the curve is rendered as a connected
sequence of ray facing quads. Individual quads are considered to have
subpixel size, and zooming onto the curve might show geometric
artifacts. The number of quads to subdivide into can be specified
through the `rtcSetGeometryTessellationRate` function. By default the
tessellation rate is 4.

##### Normal Oriented Curves

The `RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_*` mode is a mode designed to
render blades of grass. In this mode a vertex spline has to get
specified as for the previous modes, but additionally a normal spline
is required. If the Hermite basis is used, the
`RTC_BUFFER_TYPE_NORMAL` and `RTC_BUFFER_TYPE_NORMAL_DERIVATIVE`
buffers have both to be set.

The curve is rendered as a flat band whose center approximately
follows the provided vertex spline, whose half width approximately
follows the provided radius spline, and whose normal orientation
approximately follows the provided normal spline.

To intersect the normal oriented curve, we perform a newton-raphson
style intersection of a ray with a tensor product surface of a linear
basis (perpendicular to the curve) and cubic Bézier basis (along the
curve). We use a guide curve and its derivatives to construct the
control points of that surface. The guide curve is defined by a sweep
surface defined by sweeping a line centered at the vertex spline
location along the curve. At each parameter value the half width of
the line matches the radius spline, and the direction matches the
cross product of the normal from the normal spline and tangent of the
vertex spline. Note that this construction does not work when the
provided normals are parallel to the curve direction. For this reason
the provided normals should best be kept as perpendicular to the curve
direction as possible. We further assume second order derivatives of
the center curve to be zero for this construction, as otherwise very
large curvatures occurring in corner cases, can thicken the constructed
curve significantly.

##### Round Curves

In the `RTC_GEOMETRY_TYPE_ROUND_*` round mode, a real geometric
surface is rendered for the curve, which is more expensive but allows
closeup views.

For the linear basis the round mode renders a cone that tangentially
touches a start-sphere and end-sphere. The start sphere is rendered
when no previous segments is indicated by the neighbor bits. The end
sphere is always rendered but parts that lie inside the next segment
are clipped away (if that next segment exists). This way a curve is
closed on both ends and the interior will render properly as long as
only neighboring segments penetrate into a segment. For this to work
properly it is important that the flags buffer is properly populated
with neighbor information.

For the cubic polynomial bases, the round mode renders a sweep surface
by sweeping a varying radius circle tangential along the curve. As a
limitation, the radius of the curve has to be smaller than the
curvature radius of the curve at each location on the curve.

The intersection with the curve segment stores the parametric hit
location along the curve segment as u-coordinate (range 0 to +1).

For flat curves, the v-coordinate is set to the normalized distance in
the range -1 to +1. For normal oriented curves the v-coordinate is in
the range 0 to 1. For the linear basis and in round mode the
v-coordinate is set to zero.

In flat mode, the geometry normal `Ng` is set to the tangent of the
curve at the hit location. In round mode and for normal oriented
curves, the geometry normal `Ng` is set to the non-normalized
geometric normal of the surface.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers must have the same stride and size. For the Hermite
basis also a tangent buffer has to be set for each time step and for
normal oriented curves a normal buffer has to get specified for each
time step.

Also see tutorials [Hair] and [Curves] for examples of how to create and
use curve geometries.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [RTCCurveFlags]
