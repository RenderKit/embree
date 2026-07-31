% rtcTraversableForwardOccluded1/Ex(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversableForwardOccluded1/Ex - forwards a single ray to new scene
      from user geometry callback

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableForwardOccluded1(
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay* ray,
      unsigned int instID
    );

    void rtcTraversableForwardOccluded1Ex(
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay* ray,
      unsigned int instID,
      unsigned int instPrimID,
    );

#### DESCRIPTION

The `rtcTraversalbeForwardOccluded1` and `rtcTraversableForwardOccluded1Ex` functions are
equivalent to `rtcForwardOccluded1` and `rtcForwardOccluded1Ex` respectively
but take a traversable object (`traversable` argument) instead of a scene
object.

For more details, refer to the documentation of `rtcForwardOccluded1/Ex`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcForwardOccluded1], [rtcGetSceneTraversable]
