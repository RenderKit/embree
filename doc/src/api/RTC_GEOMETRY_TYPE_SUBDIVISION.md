% RTC_GEOMETRY_TYPE_SUBDIVISION(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_SUBDIVISION - subdivision geometry type

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry geometry =
      rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SUBDIVISION);

#### DESCRIPTION

Catmull-Clark subdivision meshes are supported, including support for
edge creases, vertex creases, holes, non-manifold geometry, and
face-varying interpolation. The number of vertices per face can be in
the range of 3 to 15 vertices (triangles, quadrilateral, pentagons,
etc).

Subdivision meshes are created by passing
`RTC_GEOMETRY_TYPE_SUBDIVISION` to the `rtcNewGeometry` function.
Various buffers need to be set by the application to set up the
subdivision mesh. See `rtcSetGeometryBuffer` and
`rtcSetSharedGeometryBuffer` for more details on how to set
buffers. The face buffer (`RTC_BUFFER_TYPE_FACE` type and
`RTC_FORMAT_UINT` format) contains the number of edges/indices of each
face (3 to 15), and the number of faces is inferred from the size of
this buffer. The index buffer (`RTC_BUFFER_TYPE_INDEX` type) contains
multiple (3 to 15) 32-bit vertex indices (`RTC_FORMAT_UINT` format)
for each face, and the number of edges is inferred from the size of
this buffer. The vertex buffer (`RTC_BUFFER_TYPE_VERTEX` type) stores
an array of single precision `x`, `y`, `z` floating point coordinates
(`RTC_FORMAT_FLOAT3` format), and the number of vertices is inferred
from the size of this buffer.

Optionally, the application may set additional index buffers using
different buffer slots if multiple topologies are required for
face-varying interpolation. The standard vertex buffers
(`RTC_BUFFER_TYPE_VERTEX`) are always bound to the geometry topology
(topology 0) thus use `RTC_BUFFER_TYPE_INDEX` with buffer slot 0.
User vertex data interpolation may use different topologies as
described later.

Optionally, the application can set up the hole buffer
(`RTC_BUFFER_TYPE_HOLE`) which contains an array of 32-bit indices
(`RTC_FORMAT_UINT` format) of faces that should be considered
non-existing in all topologies. The number of holes is inferred from
the size of this buffer.

Optionally, the application can fill the level buffer
(`RTC_BUFFER_TYPE_LEVEL`) with a tessellation rate for each of the
edges of each face. This buffer must have the same size as the index
buffer. The tessellation level is a positive floating point value
(`RTC_FORMAT_FLOAT` format) that specifies how many quads along the
edge should be generated during tessellation. If no level buffer is
specified, a level of 1 is used. The maximally supported edge level is
4096, and larger levels are clamped to that value. Note that edges may
be shared between (typically 2) faces. To guarantee a watertight
tessellation, the level of these shared edges should be identical. A
uniform tessellation rate for an entire subdivision mesh can be set by
using the `rtcSetGeometryTessellationRate` function. The existence of
a level buffer has precedence over the uniform tessellation rate.

Optionally, the application can fill the sparse edge crease buffers to
make edges appear sharper. The edge crease index buffer
(`RTC_BUFFER_TYPE_EDGE_CREASE_INDEX`) contains an array of pairs of
32-bit vertex indices (`RTC_FORMAT_UINT2` format) that specify
unoriented edges in the geometry topology. The edge crease weight
buffer (`RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT`) stores for each of
these crease edges a positive floating point weight (`RTC_FORMAT_FLOAT`
format). The number of edge creases is inferred from the size of these
buffers, which has to be identical. The larger a weight, the sharper
the edge. Specifying a weight of infinity is supported and marks an
edge as infinitely sharp. Storing an edge multiple times with the same
crease weight is allowed, but has lower performance. Storing an edge
multiple times with different crease weights results in undefined
behavior. For a stored edge (i,j), the reverse direction edges (j,i) do
not have to be stored, as both are considered the same unoriented edge.
Edge crease features are shared between all topologies.

Optionally, the application can fill the sparse vertex crease buffers
to make vertices appear sharper. The vertex crease index buffer
(`RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX`), contains an array of 32-bit
vertex indices (`RTC_FORMAT_UINT` format) to specify a set of vertices
from the geometry topology. The vertex crease weight buffer
(`RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT`) specifies for each of these
vertices a positive floating point weight (`RTC_FORMAT_FLOAT`
format). The number of vertex creases is inferred from the size of
these buffers, and has to be identical. The larger a weight, the
sharper the vertex. Specifying a weight of infinity is supported and
makes the vertex infinitely sharp. Storing a vertex multiple times
with the same crease weight is allowed, but has lower performance.
Storing a vertex multiple times with different crease weights results
in undefined behavior. Vertex crease features are shared between all
topologies.

Subdivision modes can be used to force linear interpolation for parts
of the subdivision mesh; see `rtcSetGeometrySubdivisionMode` for more
details.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers have to have the same stride and size.

Also see tutorial [Subdivision Geometry] for an example of how to create
subdivision surfaces.

#### Parametrization

The parametrization for subdivision faces is different for
quadrilaterals and non-quadrilateral faces.

The parametrization of a quadrilateral face uses the first vertex `p0`
as base point, and the vector `p1 - p0` as u-direction and `p3 - p0`
as v-direction.

The parametrization for all other face types (with number of vertices
not equal 4), have a special parametrization where the subpatch ID `n`
(of the `n`-th quadrilateral that would be obtained by a single
subdivision step) and the local hit location inside this quadrilateral
are encoded in the UV coordinates. The following code extracts the
sub-patch ID `i` and local UVs of this subpatch:

    unsigned int l = floorf(0.5f*U);
    unsigned int h = floorf(0.5f*V);
    unsigned int i = 4*h+l;
    float u = 2.0f*fracf(0.5f*U)-0.5f;
    float v = 2.0f*fracf(0.5f*V)-0.5f;

This encoding allows local subpatch UVs to be in the range `[-0.5,1.5[`
thus negative subpatch UVs can be passed to `rtcInterpolate` to sample
subpatches slightly out of bounds.  This can be useful to calculate
derivatives using finite differences if required. The encoding further
has the property that one can just move the value `u` (or `v`) on a
subpatch by adding `du` (or `dv`) to the special UV encoding as long as
it does not fall out of the `[-0.5,1.5[` range.

To smoothly interpolate vertex attributes over the subdivision surface
we recommend using the `rtcInterpolate` function, which will apply the
standard subdivision rules for interpolation and automatically takes
care of the special UV encoding for non-quadrilaterals.

#### Face-Varying Data

Face-varying interpolation is supported through multiple topologies
per subdivision mesh and binding such topologies to vertex attribute
buffers to interpolate. This way, texture coordinates may use a
different topology with additional boundaries to construct separate UV
regions inside one subdivision mesh.

Each such topology `i` has a separate index buffer (specified using
`RTC_BUFFER_TYPE_INDEX` with buffer slot `i`) and separate subdivision
mode that can be set using `rtcSetGeometrySubdivisionMode`. A vertex
attribute buffer `RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE` bound to a buffer
slot `j` can be assigned to use a topology for interpolation using the
`rtcSetGeometryVertexAttributeTopology` call.

The face buffer (`RTC_BUFFER_TYPE_FACE` type) is shared between all
topologies, which means that the `n`-th primitive always has the same
number of vertices (e.g. being a triangle or a quad) for each topology.
However, the indices of the topologies themselves may be different.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry]
