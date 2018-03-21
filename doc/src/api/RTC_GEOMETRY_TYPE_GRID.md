% RTC_GEOMETRY_TYPE_GRID(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_GRID - grid geometry type

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry geometry =
      rtcNewGeometry(device, RTC_GEOMETRY_TYPE_GRID);

#### DESCRIPTION

Grid meshes are created by passing `RTC_GEOMETRY_TYPE_GRID` to the
`rtcNewGeometry` function call, and contain an array of `RTCGrid`
primitives. This array of grids and can be specified by setting up an
grid buffer (`RTC_BUFFER_TYPE_GRID` type and `RTC_FORMAT_GRID` format)
and the grid vertices by setting a vertex buffer
(`RTC_BUFFER_TYPE_VERTEX` type). See `rtcSetGeometryBuffer` and
`rtcSetSharedGeometryBuffer` for more details on how to set
buffers. The number of primitives in the grid mesh is inferred from
the size of the grid buffer. The vertex buffer contains an array of
single precision `x`, `y`, `z` floating point coordinates
(`RTC_FORMAT_FLOAT3` format), and the number of vertices is inferred
from the size of that buffer. The vertex buffer can be at most 16 GB
large.

Each grid in the grid mesh is of the type `RTCGrid`

struct RTCGrid
{
  unsigned int startVertexID;
  unsigned int stride;
  unsigned short width,height; 
};

which describes a 2D grid structure of vertices (with respect to the
vertex buffer of the grid mesh). The `width` and `height` members of
the structure specifying the resolution of the grid, e.g. setting both
`width` and `height` to 3 sets up a 3x3 vertex grid. The maximum
allowed `width` and `height` is 32767. The `startVertexID` specifies
the ID of the first top-left vertex in the vertex grid with
coordinates (0,0), while the `stride` parameter specifies the number
of vertices in a single row.

A vertex grid of dimensions X and Y is treated as a (X-1) x (Y-1) grid
of `quads` (triangle-pairs), with the same shared edge handling as for
regular quad meshes. However, the `u`/`v` coordinates are specified
with respect to the entire vertex grid. The `u` direction follows the
`width` of the grid while the `v` direction the `height`.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers must have the same stride and size.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcDeviceGetError`.

#### SEE ALSO

[rtcNewGeometry]
