% rtcTraversableOccluded4/8/16(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversableOccluded4/8/16 - finds any hits for a ray packet

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableOccluded4(
      const int* valid,
      RTCTraversable traversable,
      struct RTCRay4* ray,
      struct RTCOccludedArguments* args = NULL
    );

    void rtcTraversableOccluded8(
      const int* valid,
      RTCTraversable traversable,
      struct RTCRay8* ray,
      struct RTCOccludedArguments* args = NULL
    );

    void rtcTraversableOccluded16(
      const int* valid,
      RTCTraversable traversable,
      struct RTCRay16* ray,
      struct RTCOccludedArguments* args = NULL
    );

#### DESCRIPTION

The `rtcTraversableOccluded4/8/16` functions are equivalent to `rtcOccluded4/8/16`
but take a traversable object (`traversable` argument) instead of a scene
object.

For more details, refer to the documentation of `rtcOccluded4/8/16`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded4/8/16], [rtcGetSceneTraversable]
