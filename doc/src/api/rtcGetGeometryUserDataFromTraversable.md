% rtcGetGeometryUserDataFromTraversable(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryUserDataFromTraversable - returns the user data pointer
      of the geometry through the scene object

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void* rtcGetGeometryUserDataFromTraversable(
        RTCTraversable traversable,
        unsigned int geomID);

#### DESCRIPTION

The `rtcGetGeometryUserDataFromTraversable` function is equivalent to
`rtcGetGeometryUserDataFromScene` but takes traversable object
(`traversable` argument) instead of a scene object.
Using this method is optional on CPU but it is required for SYCL.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetGeometryUserDataFromScene], [rtcGetSceneTraversable]
