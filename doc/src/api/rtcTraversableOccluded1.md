% rtcTraversableOccluded1(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversableOccluded1 - finds any hit for a single ray

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableOccluded1(
      RTCTraversable traversable,
      struct RTCRay* ray,
      struct RTCOccludedArguments* args = NULL
    );

#### DESCRIPTION

The `rtcTraversableOccluded1` function is equivalent to `rtcOccluded1`
but takes traversable object (`traversable` argument) instead of a scene
object. Using this method is optional on CPU but it is required for SYCL.

For more details, refer to the documentation of `rtcOccluded1`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded1], [rtcGetSceneTraversable]