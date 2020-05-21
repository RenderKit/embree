% RTCFormat(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCFormat - specifies format of data in buffers

#### SYNOPSIS

    #include <embree3/rtcore_ray.h>

    enum RTCFormat
    {
      RTC_FORMAT_UINT,
      RTC_FORMAT_UINT2,
      RTC_FORMAT_UINT3,
      RTC_FORMAT_UINT4,

      RTC_FORMAT_FLOAT,
      RTC_FORMAT_FLOAT2,
      RTC_FORMAT_FLOAT3,
      RTC_FORMAT_FLOAT4,
      RTC_FORMAT_FLOAT5,
      RTC_FORMAT_FLOAT6,
      RTC_FORMAT_FLOAT7,
      RTC_FORMAT_FLOAT8,
      RTC_FORMAT_FLOAT9,
      RTC_FORMAT_FLOAT10,
      RTC_FORMAT_FLOAT11,
      RTC_FORMAT_FLOAT12,
      RTC_FORMAT_FLOAT13,
      RTC_FORMAT_FLOAT14,
      RTC_FORMAT_FLOAT15,
      RTC_FORMAT_FLOAT16,

      RTC_FORMAT_FLOAT3X4_ROW_MAJOR,
      RTC_FORMAT_FLOAT4X4_ROW_MAJOR,

      RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,
      RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,

      RTC_FORMAT_GRID,
};

#### DESCRIPTION

The `RTFormat` structure defines the data format stored in data
buffers provided to Embree using the [rtcSetGeometryBuffer],
[rtcSetSharedGeometryBuffer], and [rtcSetNewGeometryBuffer] API calls.

The `RTC_FORMAT_UINT/2/3/4` format are used to specify that data
buffers store unsigned integers, or unsigned integer vectors of size
2,3 or 4. This format has typically to get used when specifying index
buffers, e.g. `RTC_FORMAT_UINT3` for triangle meshes.

The `RTC_FORMAT_FLOAT/2/3/4...` format are used to specify that data
buffers store single precision floating point values, or vectors there
of (size 2,3,4, etc.). This format is typcally used to specify to
format of vertex buffers, e.g. the `RTC_FORMAT_FLOAT3` type for vertex
buffers of triangle meshes.

The `RTC_FORMAT_FLOAT3X4_ROW_MAJOR` and
`RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR` formats, specify a 3x4 floating
point matrix layed out either row major or column major. The
`RTC_FORMAT_FLOAT4X4_ROW_MAJOR` and `RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR`
formats, specify a 4x4 floating point matrix layed out either row
major or column major. These matrix formats are used in the
[rtcSetGeometryTransform] function in order to set a transformation
matrix for geometries.

The `RTC_FORMAT_GRID` is a special data format used to specify grid
primitives of layout RTCGrid when creating grid geometries
(see [RTC_GEOMETRY_TYPE_GRID]).

#### EXIT STATUS

#### SEE ALSO

[rtcSetGeometryBuffer], [rtcSetSharedGeometryBuffer],
[rtcSetNewGeometryBuffer], [rtcSetGeometryTransform]
