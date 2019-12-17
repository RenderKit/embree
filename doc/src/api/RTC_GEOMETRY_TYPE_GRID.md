% RTC_GEOMETRY_TYPE_GRID(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_GRID - grid geometry type

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry geometry =
      rtcNewGeometry(device, RTC_GEOMETRY_TYPE_GRID);

#### DESCRIPTION

Grid meshes are created by passing `RTC_GEOMETRY_TYPE_GRID` to the
`rtcNewGeometry` function call, and contain an array of grid
primitives. This array of grids can be specified by setting up a
grid buffer (with `RTC_BUFFER_TYPE_GRID` type and `RTC_FORMAT_GRID` format)
and the grid mesh vertices by setting a vertex buffer
(`RTC_BUFFER_TYPE_VERTEX` type). See `rtcSetGeometryBuffer` and
`rtcSetSharedGeometryBuffer` for more details on how to set
buffers. The number of grid primitives in the grid mesh is inferred
from the size of the grid buffer.

The vertex buffer contains an array of single precision `x`, `y`, `z`
floating point coordinates (`RTC_FORMAT_FLOAT3` format), and the
number of vertices is inferred from the size of that buffer.

Each grid in the grid buffer is of the type `RTCGrid`:

    struct RTCGrid
    {
      unsigned int startVertexID;
      unsigned int stride;
      unsigned short width,height; 
    };

The `RTCGrid` structure describes a 2D grid of vertices (with respect
to the vertex buffer of the grid mesh). The `width` and `height`
members specify the number of vertices in u and v direction,
e.g. setting both `width` and `height` to 3 sets up a 3×3 vertex
grid. The maximum allowed `width` and `height` is 32767. The
`startVertexID` specifies the ID of the top-left vertex in the vertex
grid, while the `stride` parameter specifies a stride (in number of
vertices) used to step to the next row.

A vertex grid of dimensions `width` and `height` is treated as a
`(width-1)` x `(height-1)` grid of `quads` (triangle-pairs), with the
same shared edge handling as for regular quad meshes. However, the
`u`/`v` coordinates have the uniform range `[0..1]` for an entire
vertex grid. The `u` direction follows the `width` of the grid while
the `v` direction the `height`.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers must have the same stride and size.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry]
