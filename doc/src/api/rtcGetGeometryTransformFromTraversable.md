% rtcGetGeometryTransformFromTraversable(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryTransformFromTraversable - returns the interpolated instance
      transformation for the specified time

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcGetGeometryTransformFromTraversable(
      RTCTraversable traversable,
      unsigned int geomID,
      float time,
      enum RTCFormat format,
      void* xfm
    );

#### DESCRIPTION

The `rtcGetGeometryTransformFromTraversable` function is equivalent to
`rtcGetGeometryTransformFromScene` but takes traversable object (`traversable` argument)
instead of a scene object. Using this method is optional on CPU but it is required for SYCL.

For more details, refer to the documentation of `rtcGetGeometryTransformFromScene`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetGeometryTransformFromScene], [rtcGetSceneTraversable]
