% RTCBufferType(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCFormat - specifies format of data in buffers

#### SYNOPSIS

    #include <embree3/rtcore_ray.h>

    enum RTCBufferType
    {
      RTC_BUFFER_TYPE_INDEX            = 0,
      RTC_BUFFER_TYPE_VERTEX           = 1,
      RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE = 2,
      RTC_BUFFER_TYPE_NORMAL           = 3,
      RTC_BUFFER_TYPE_TANGENT          = 4,
      RTC_BUFFER_TYPE_NORMAL_DERIVATIVE = 5,
    
      RTC_BUFFER_TYPE_GRID                 = 8,
    
      RTC_BUFFER_TYPE_FACE                 = 16,
      RTC_BUFFER_TYPE_LEVEL                = 17,
      RTC_BUFFER_TYPE_EDGE_CREASE_INDEX    = 18,
      RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT   = 19,
      RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX  = 20,
      RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT = 21,
      RTC_BUFFER_TYPE_HOLE                 = 22,
    
      RTC_BUFFER_TYPE_FLAGS = 32
    };

#### DESCRIPTION

The `RTBufferType` structure defines slots to assign data buffers to
using the [rtcSetGeometryBuffer], [rtcSetSharedGeometryBuffer], and
[rtcSetNewGeometryBuffer] API calls.

For most geometry types the `RTC_BUFFER_TYPE_INDEX` slot is used to
assign an index buffer, while the `RTC_BUFFER_TYPE_VERTEX` is used to
assign the corresponding vertex buffer.

The `RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE` slot can get used to assign
arbitrary additional vertex data which can get interpolated using the
[rtcInterpolate] API call.

The `RTC_BUFFER_TYPE_NORMAL`, `RTC_BUFFER_TYPE_TANGENT`, and
`RTC_BUFFER_TYPE_NORMAL_DERIVATIVE` are special buffers required to
assign per vertex normals, tangents, and normal derivatives for some
curve types.

The `RTC_BUFFER_TYPE_GRID` buffer is used to assign the grid primitive
buffer for grid geometries (see [RTC_GEOMETRY_TYPE_GRID]).

The `RTC_BUFFER_TYPE_FACE`, `RTC_BUFFER_TYPE_LEVEL`,
`RTC_BUFFER_TYPE_EDGE_CREASE_INDEX`,
`RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT`,
`RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX`,
`RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT`, and `RTC_BUFFER_TYPE_HOLE` are
special buffers required to create subdivision meshes (see
[RTC_GEOMETRY_TYPE_SUBDIVISION]).

The `RTC_BUFFER_TYPE_FLAGS` can get used to add additional flag per
primitive of a geometry, and is currently only used for linear curves.

#### EXIT STATUS

#### SEE ALSO

[rtcSetGeometryBuffer], [rtcSetSharedGeometryBuffer],
[rtcSetNewGeometryBuffer]
