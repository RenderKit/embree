% rtcGetGeometryTransform(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryTransform - returns the interpolated instance
      transformation for the specified time

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcGetGeometryTransform(
      RTCGeometry geometry,
      float time,
      enum RTCFormat format,
      void* xfm
    );

#### DESCRIPTION

The `rtcGetGeometryTransform` function returns the interpolated local
to world transformation (`xfm` parameter) of an instance geometry
(`geometry` parameter) for a particular time (`time` parameter in range
$[0,1]$) in the specified format (`format` parameter).

Possible formats for the returned matrix are:

+ `RTC_FORMAT_FLOAT3X4_ROW_MAJOR`: The 3×4 float matrix is laid out
  in row-major form.

+ `RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR`: The 3×4 float matrix is laid out
  in column-major form.

+ `RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR`: The 3×4 float matrix is laid out
  in column-major form as a 4×4 homogeneous matrix with last row equal
  to (0, 0, 0, 1).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_INSTANCE], [rtcSetGeometryTransform]
