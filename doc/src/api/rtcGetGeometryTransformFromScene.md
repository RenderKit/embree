% rtcGetGeometryTransformFromScene(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryTransformFromScene - returns the interpolated instance
      transformation for the specified time

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcGetGeometryTransformFromScene(
      RTCScene scene,
      unsigned int geomID,
      float time,
      enum RTCFormat format,
      void* xfm
    );

#### DESCRIPTION

The `rtcGetGeometryTransformFromScene` function returns the
interpolated local to world transformation (`xfm` output parameter) of
an instance geometry specified by its geometry ID (`geomID` parameter)
of a scene (`scene` parameter) for a particular time (`time` parameter
in range $[0,1]$) in the specified format (`format` parameter).

Possible formats for the returned matrix are:

+ `RTC_FORMAT_FLOAT3X4_ROW_MAJOR`: The 3×4 float matrix is laid out
  in row-major form.

+ `RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR`: The 3×4 float matrix is laid out
  in column-major form.

+ `RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR`: The 3×4 float matrix is laid out
  in column-major form as a 4×4 homogeneous matrix with last row equal
  to (0, 0, 0, 1).

In contrast to the `rtcGetGeometryTransform` function, the
`rtcGetGeometryTransformFromScene` function can get used during
rendering inside a SYCL kernel.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_INSTANCE], [rtcSetGeometryTransform], [rtcGetGeometryTransform]
