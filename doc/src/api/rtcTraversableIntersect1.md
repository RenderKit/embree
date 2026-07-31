% rtcTraversableIntersect1(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversableIntersect1 - finds the closest hit for a single ray

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableIntersect1(
      RTCTraversable traversable,
      struct RTCRayHit* rayhit
      struct RTCIntersectArguments* args = NULL
    );

#### DESCRIPTION

The `rtcTraversableIntersect` function is equivalent to `rtcIntersect1`
but takes traversable object (`traversable` argument) instead of a scene
object. Using this method is optional on CPU but it is required for SYCL.

For more details, refer to the documentation of `rtcIntersect1`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcIntersect1], [rtcGetSceneTraversable]
