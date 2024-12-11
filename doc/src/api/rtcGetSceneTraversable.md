% rtcGetSceneTraversable(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetSceneTraversable - gets a read-only hand to a RTCScene object
      that is also valid on a SYCL device and can be used for ray queries.

#### SYNOPSIS

    #include <embree4/rtcore.h>

    RTCTraversable rtcGetSceneTraversable(RTCScene scene);

#### DESCRIPTION

The `rtcGetSceneTraversable` function returns a read-only handle
to a RTCScene object (`scene` argument). Traversable objects are
used for ray queries on a SYCL device.

Traversable objects can also be used in host/CPU code and Embree provides
other API calls such as the `rtcTraversablePointQuery` to help
write portable code compatible with CPU and SYCL device execution.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcTraversableIntersect1]
