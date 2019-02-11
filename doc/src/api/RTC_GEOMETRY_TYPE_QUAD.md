% RTC_GEOMETRY_TYPE_QUAD(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_QUAD - quad geometry type

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry geometry =
      rtcNewGeometry(device, RTC_GEOMETRY_TYPE_QUAD);

#### DESCRIPTION

Quad meshes are created by passing `RTC_GEOMETRY_TYPE_QUAD` to the
`rtcNewGeometry` function call. The quad indices can be specified by
setting an index buffer (`RTC_BUFFER_TYPE_INDEX` type) and the quad
vertices by setting a vertex buffer (`RTC_BUFFER_TYPE_VERTEX`
type). See `rtcSetGeometryBuffer` and `rtcSetSharedGeometryBuffer` for
more details on how to set buffers. The index buffer contains an array
of four 32-bit indices per quad (`RTC_FORMAT_UINT4` format), and the
number of primitives is inferred from the size of that buffer. The
vertex buffer contains an array of single precision `x`, `y`, `z`
floating point coordinates (`RTC_FORMAT_FLOAT3` format), and the number
of vertices is inferred from the size of that buffer. The vertex buffer
can be at most 16 GB large.

A quad is internally handled as a pair of two triangles `v0,v1,v3` and
`v2,v3,v1`, with the `u'`/`v'` coordinates of the second triangle
corrected by `u = 1-u'` and `v = 1-v'` to produce a quad
parametrization where `u` and `v` are in the range 0 to 1. Thus the
parametrization of a quad uses the first vertex `p0` as base point,
and the vector `p1 - p0` as `u`-direction, and `p3 - p0` as
v-direction. Thus vertex attributes `t0,t1,t2,t3` can be bilinearly
interpolated over the quadrilateral the following way:

    t_uv = (1-v)((1-u)*t0 + u*t1) + v*((1-u)*t3 + u*t2)

Mixed triangle/quad meshes are supported by encoding a triangle as a
quad, which can be achieved by replicating the last triangle vertex
(`v0,v1,v2` -> `v0,v1,v2,v2`). This way the second triangle is a line
(which can never get hit), and the parametrization of the first
triangle is compatible with the standard triangle parametrization.

A quad whose vertices are laid out counter-clockwise has its
geometry normal pointing upwards outside the front face, like
illustrated in the following picture.

``` {image=imgQuadUV}
```

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers must have the same stride and size.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry]
