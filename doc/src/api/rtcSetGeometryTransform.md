% rtcSetGeometryTransform(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryTransform - sets the transformation for a particular
      time step of an instance geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryTransform(
      RTCGeometry geometry,
      unsigned int timeStep,
      enum RTCFormat format,
      const float* xfm
    );

#### DESCRIPTION

The `rtcSetGeometryTransform` function sets the local-to-world affine
transformation (`xfm` parameter) of an instance geometry (`geometry`
parameter) for a particular time step (`timeStep` parameter). The
transformation is specified as a 3×4 matrix (3×3 linear transformation
plus translation), for which the following formats (`format` parameter)
are supported:

+ `RTC_FORMAT_FLOAT3X4_ROW_MAJOR`: The 3×4 float matrix is laid out
  in row-major form.

+ `RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR`: The 3×4 float matrix is laid out
  in column-major form.

+ `RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR`: The 3×4 float matrix is laid out
  in column-major form as a 4×4 homogeneous matrix with the last row
  being equal to (0, 0, 0, 1).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_INSTANCE]
