% rtcGetGeometryTransformEx(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryTransformEx - returns the interpolated instance
    transformation for an instance of an instance array geometry for the
    specified time.

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcGetGeometryTransformEx(
      RTCGeometry geometry,
      unsigned int instPrimID,
      float time,
      enum RTCFormat format,
      void* xfm
    );

#### DESCRIPTION

The `rtcGetGeometryTransformEx` function returns the interpolated local
to world transformation (`xfm` parameter) of the `instPrimID`-th instance of an
instance array geometry (`geometry` parameter) for a particular time (`time`
parameter in range $[0,1]$) in the specified format (`format` parameter).
The function can also be used when `geometry` refers to a regular instance, but
then the `instPrimID` has to be $0$.

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

[RTC_GEOMETRY_TYPE_INSTANCE_ARRAY]
