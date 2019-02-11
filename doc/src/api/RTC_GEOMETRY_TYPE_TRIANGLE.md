% RTC_GEOMETRY_TYPE_TRIANGLE(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_TRIANGLE - triangle geometry type

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry geometry =
      rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

#### DESCRIPTION

Triangle meshes are created by passing `RTC_GEOMETRY_TYPE_TRIANGLE` to
the `rtcNewGeometry` function call. The triangle indices can be
specified by setting an index buffer (`RTC_BUFFER_TYPE_INDEX` type)
and the triangle vertices by setting a vertex buffer
(`RTC_BUFFER_TYPE_VERTEX` type). See `rtcSetGeometryBuffer` and
`rtcSetSharedGeometryBuffer` for more details on how to set
buffers. The index buffer contains an array of three 32-bit indices
per triangle (`RTC_FORMAT_UINT` format) and the number of primitives is
inferred from the size of that buffer. The vertex buffer contains an
array of single precision `x`, `y`, `z` floating point coordinates
(`RTC_FORMAT_FLOAT3` format), and the number of vertices are inferred
from the size of that buffer. The vertex buffer can be at most 16 GB
large.

The parametrization of a triangle uses the first vertex `p0` as base
point, the vector `p1 - p0` as u-direction and the vector `p2 - p0` as
v-direction. Thus vertex attributes `t0,t1,t2` can be linearly
interpolated over the triangle the following way:

    t_uv = (1-u-v)*t0 + u*t1 + v*t2
         = t0 + u*(t1-t0) + v*(t2-t0)

A triangle whose vertices are laid out counter-clockwise has its
geometry normal pointing upwards outside the front face, like
illustrated in the following picture:

``` {image=imgTriangleUV}
```

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers have to have the same stride and size.

Also see tutorial [Triangle Geometry] for an example of how to create
triangle meshes.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that be get
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry]
